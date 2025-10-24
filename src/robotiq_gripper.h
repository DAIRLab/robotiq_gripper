#pragma once

#include <iostream>
#include <stdexcept>
#include <string>

#include <optional>

#define PORT 63352

namespace drivers {
namespace robotiq {

enum class RobotiqCommand {
  ACTIVATION_STATUS,
  GRIPPER_MODE,
  GOTO_STATUS,
  GRIPPER_STATUS,
  OBJECT_DETECTION_STATUS,
  FAULT_STATUS,
  POSITION_REQUEST_ECHO,
  POSITION,
  SPEED,
  FORCE,
  CURRENT,
  SERIAL_NUMBER,
  PRODUCTION_YEAR,
  FIRMWARE_VERSION,
  DRIVER_VERSION,
  SOCKET_SLAVE_ID,
};

inline constexpr const char* RobotiqCommandToString(
    RobotiqCommand cmd) noexcept {
  switch (cmd) {
    case RobotiqCommand::ACTIVATION_STATUS:
      return "ACT";
    case RobotiqCommand::GRIPPER_MODE:
      return "MOD";
    case RobotiqCommand::GOTO_STATUS:
      return "GTO";
    case RobotiqCommand::GRIPPER_STATUS:
      return "STA";
    case RobotiqCommand::OBJECT_DETECTION_STATUS:
      return "OBJ";
    case RobotiqCommand::FAULT_STATUS:
      return "FLT";
    case RobotiqCommand::POSITION_REQUEST_ECHO:
      return "PRE";
    case RobotiqCommand::POSITION:
      return "POS";
    case RobotiqCommand::SPEED:
      return "SPE";
    case RobotiqCommand::FORCE:
      return "FOR";
    case RobotiqCommand::CURRENT:
      return "CUR";
    case RobotiqCommand::SERIAL_NUMBER:
      return "SNU";
    case RobotiqCommand::PRODUCTION_YEAR:
      return "PYE";
    case RobotiqCommand::FIRMWARE_VERSION:
      return "FWV";
    case RobotiqCommand::DRIVER_VERSION:
      return "VER";
    case RobotiqCommand::SOCKET_SLAVE_ID:
      return "SID";
    default:
      std::cerr << "Unknown RobotiqCommand enum value" << std::endl;
      return "";
  }
}

class RobotiqGripper {
 public:
  RobotiqGripper(std::string ip_address, int port = PORT, int timeout = -1,
                 int poll_interval = 100);
  ~RobotiqGripper();
  bool connect();
  void disconnect();
  bool isConnected() const;
  bool activate();
  std::string getValue(RobotiqCommand cmd) const;
  int getIntValue(RobotiqCommand cmd) const;
  bool setValue(RobotiqCommand cmd, int value);

  bool moveToPosition(int position, std::optional<int> speed = std::nullopt,
                      std::optional<int> force = std::nullopt,
                      bool blocking = false);
  bool openGripper(bool blocking = false);
  bool closeGripper(bool blocking = false);
  int getCurrentPosition() const;

 private:
  // Private members
  std::string ip_address_;
  int port_;
  int timeout_;
  int socket_fd_;
  bool connected_;
  int poll_interval_;

  // Private methods
  std::string sendCommand(const std::string& command) const;
  std::string receiveResponse() const;
  bool checkValueInRange(RobotiqCommand cmd, int value) const;
  bool commandIsWriteable(RobotiqCommand cmd) const;
  void blockUntilStatus(RobotiqCommand cmd, int desired_status,
                           bool set = true);
};

}  // namespace robotiq
}  // namespace drivers