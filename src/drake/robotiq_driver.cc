#include <pthread.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <optional>

#include "robotiq/lcmt_robotiq_command.hpp"
#include "robotiq/lcmt_robotiq_status.hpp"
#include "robotiq_gripper.h"

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

using namespace driver::robotiq;

DEFINE_string(gripper_ip_address, "192.168.1.100",
              "IP address of the Robotiq gripper");
DEFINE_string(lcm_url, "", "LCM URL for Robotiq driver");
DEFINE_string(lcm_command_channel, "ROBOTIQ_COMMAND",
              "Channel to listen for gripper command messages on");
DEFINE_string(lcm_status_channel, "ROBOTIQ_STATUS",
              "Channel to publish gripper status messages on");
DEFINE_int32(gripper_port, 63352, "TCP port for gripper communication");
DEFINE_double(
    expire_sec, 0.1,
    "How much delay is allowed for messages to be allowed. Converted to "
    "usec, must be non-negative + finite.");
DEFINE_int32(command_stop_limit, 5,
             "Maximum number of messages before stopping commands.");
DEFINE_int32(status_publish_rate, 100, "Rate to publish status messages (Hz)");

class RobotiqDriverRunner {
 public:
  RobotiqDriverRunner(const std::string& gripper_ip_address, int gripper_port,
                      const std::string& lcm_url,
                      const std::string& lcm_command_channel,
                      const std::string& lcm_status_channel,
                      uint32_t expire_usec)
      : lcm_(lcm_url),
        lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel),
        expire_usec_(expire_usec) {
    gripper_ = std::make_unique<RobotiqGripper>(gripper_ip_address,
                                                gripper_port, -1, 100);
  }

  ~RobotiqDriverRunner() { stop(); }

  int run() {
    if (!lcm_.good()) {
      drake::log()->error("Failed to initialize LCM");
      return 1;
    }

    // Try to connect to the gripper
    if (!gripper_->connect()) {
      drake::log()->error("Failed to connect to gripper at {}",
                          "gripper_ip_address");
      return 1;
    }
    drake::log()->info("Successfully connected to gripper");

    // Activate the gripper
    if (!gripper_->activate()) {
      drake::log()->error("Failed to activate gripper");
      gripper_->disconnect();
      return 1;
    }
    drake::log()->info("Successfully activated gripper");

    // Subscribe to command channel
    lcm::Subscription* sub = lcm_.subscribe(
        lcm_command_channel_, &RobotiqDriverRunner::getCommandMessage, this);
    sub->setQueueCapacity(100);

    // Start status thread
    stop_status_thread_ = false;
    status_thread_ =
        std::thread(&RobotiqDriverRunner::handleStatusMessage, this);

    // Start command thread
    stop_command_thread_ = false;
    command_thread_ =
        std::thread(&RobotiqDriverRunner::handleCommandMessage, this);

    // Wait for signal to stop
    {
      std::unique_lock<std::mutex> lock(handlers_mutex_);
      handlers_cv_.wait(lock);
      stop();
    }

    return 0;
  }

  int stop() {
    stop_command_thread_ = true;
    if (command_thread_.joinable()) {
      command_thread_.join();
    }

    stop_status_thread_ = true;
    if (status_thread_.joinable()) {
      status_thread_.join();
    }

    if (gripper_->isConnected()) {
      gripper_->disconnect();
    }

    return 0;
  }

 private:
  /**
   * Callback to receive command messages from LCM.
   */
  void getCommandMessage(const lcm::ReceiveBuffer*, const std::string&,
                         const robotiq::lcmt_robotiq_command* command) {
    const int64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    if (std::abs(now - command->utime) > expire_usec_) {
      drake::log()->warn("Gripper command packet too old [{} usec], skipping",
                         now - command->utime);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_ = *command;
    }
  }

  /**
   * Handles incoming LCM command messages and sends them to the gripper.
   */
  void handleCommandMessage() {
    while (!stop_command_thread_) {
      // Block until a command message is available
      lcm_.handleTimeout(0);  // Block indefinitely until message arrives

      // Get the command to send
      robotiq::lcmt_robotiq_command cmd;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!command_.has_value()) {
          continue;
        }

        cmd = command_.value();
      }

      // Check if command is too old
      const int64_t now =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
      if (std::abs(now - cmd.utime) > expire_usec_) {
        drake::log()->warn("Gripper command packet too old [{} usec], skipping",
                           now - cmd.utime);
        continue;
      }

      // Clamp values to valid range [0, 255]
      int position = static_cast<int>(
          std::clamp(static_cast<double>(cmd.position), 0.0, 255.0));
      int speed = static_cast<int>(
          std::clamp(static_cast<double>(cmd.speed), 0.0, 255.0));
      int force = static_cast<int>(
          std::clamp(static_cast<double>(cmd.force), 0.0, 255.0));

      if (!gripper_->moveToPosition(position, speed, force, true)) {
        drake::log()->error("Failed to send movement command to gripper");
        stop_command_thread_ = true;
        break;
      }

      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        prev_command_ = command_;
        command_.reset();
      }
    }

    handlers_cv_.notify_one();
  }

  /**
   * Handles publishing gripper status messages periodically.
   */
  void handleStatusMessage() {
    const std::chrono::duration<double> status_period(
        1.0 / FLAGS_status_publish_rate);
    auto next_publish_time = std::chrono::steady_clock::now();

    while (!stop_status_thread_) {
      auto now = std::chrono::steady_clock::now();

      if (now >= next_publish_time) {
        // Read gripper status
        robotiq::lcmt_robotiq_status status_msg;
        status_msg.utime =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();

        try {
          status_msg.position = static_cast<uint8_t>(
              gripper_->getIntValue(RobotiqCommand::POSITION));
          status_msg.speed = static_cast<uint8_t>(
              gripper_->getIntValue(RobotiqCommand::SPEED));
          status_msg.force = static_cast<uint8_t>(
              gripper_->getIntValue(RobotiqCommand::FORCE));

          // Read status flags
          status_msg.activation_status =
              (gripper_->getIntValue(RobotiqCommand::ACTIVATION_STATUS) != 0);
          status_msg.gripper_mode =
              (gripper_->getIntValue(RobotiqCommand::GRIPPER_MODE) != 0);
          status_msg.goto_status =
              (gripper_->getIntValue(RobotiqCommand::GOTO_STATUS) != 0);

          // Publish status message
          publishStatusMessage(status_msg);

          // Check for fault - if activation fails or gripper mode is invalid
          if (!status_msg.activation_status || !status_msg.gripper_mode) {
            drake::log()->error(
                "Gripper fault detected: activation={}, mode={}",
                status_msg.activation_status, status_msg.gripper_mode);
            stop_status_thread_ = true;
            break;
          }
        } catch (const std::exception& e) {
          drake::log()->error("Error reading gripper status: {}", e.what());
          stop_status_thread_ = true;
          break;
        }

        next_publish_time +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                status_period);
      }

      // Sleep briefly to avoid busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    handlers_cv_.notify_one();
  }

  /**
   * Publishes gripper status to LCM.
   */
  void publishStatusMessage(const robotiq::lcmt_robotiq_status& status) {
    drake::log()->debug(
        "Gripper status - Position: {}, Speed: {}, Force: {}, "
        "Activation: {}, Mode: {}, GoTo: {}",
        status.position, status.speed, status.force, status.activation_status,
        status.gripper_mode, status.goto_status);

    // Publish status message via LCM
    lcm_.publish(lcm_status_channel_, &status);
  }

  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;

  std::unique_ptr<RobotiqGripper> gripper_;

  std::optional<robotiq::lcmt_robotiq_command> command_;
  std::optional<robotiq::lcmt_robotiq_command> prev_command_;

  std::thread status_thread_;
  std::atomic<bool> stop_status_thread_{false};
  std::thread command_thread_;
  std::atomic<bool> stop_command_thread_{false};

  std::mutex handlers_mutex_;
  std::condition_variable handlers_cv_;
  std::mutex command_mutex_;

  const uint32_t expire_usec_{};
};

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_gripper_ip_address != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_command_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_status_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_expire_sec >= 0.0 &&
                     std::isfinite(FLAGS_expire_sec));

  const uint32_t expire_usec = static_cast<uint32_t>(FLAGS_expire_sec * 1e6);

  RobotiqDriverRunner runner(FLAGS_gripper_ip_address, FLAGS_gripper_port,
                             FLAGS_lcm_url, FLAGS_lcm_command_channel,
                             FLAGS_lcm_status_channel, expire_usec);
  return runner.run();
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return DoMain();
}
