#include "robotiq_gripper.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

using namespace std;

namespace drivers {
namespace robotiq {

// RobotiqGripper constructor
RobotiqGripper::RobotiqGripper(std::string ip_address, int port, int timeout,
                               int poll_interval)
    : ip_address_(std::move(ip_address)),
      port_(port),
      timeout_(timeout),
      socket_fd_(-1),
      connected_(false),
      poll_interval_(poll_interval) {}

// RobotiqGripper destructor
RobotiqGripper::~RobotiqGripper() {
  if (connected_) {
    disconnect();
  }
}

// Connect to the Robotiq gripper
bool RobotiqGripper::connect() {
  if (connected_) {
    return true;
  }

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "Error creating socket" << std::endl;
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  server_addr.sin_addr.s_addr = INADDR_ANY;

  if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr.sin_addr) <= 0) {
    std::cerr << "Invalid address/ Address not supported" << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (::connect(socket_fd_, (struct sockaddr*)&server_addr,
                sizeof(server_addr)) < 0) {
    std::cerr << "Connection failed" << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set timeout if specified
  if (timeout_ > 0) {
    struct timeval tv;
    tv.tv_sec = timeout_;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,
               sizeof(tv));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv,
               sizeof(tv));
  }

  std::cout << "Connected to Robotiq gripper at " << ip_address_ << ":" << port_
            << std::endl;

  connected_ = true;
  auto serial_number = getValue(RobotiqCommand::SERIAL_NUMBER);
  auto production_year = getValue(RobotiqCommand::PRODUCTION_YEAR);
  auto firmware_version = getValue(RobotiqCommand::FIRMWARE_VERSION);
  auto driver_version = getValue(RobotiqCommand::DRIVER_VERSION);
  std::cout << "Gripper Serial Number: " << serial_number
            << "Production Year: " << production_year
            << "Firmware Version: " << firmware_version
            << "Driver Version: " << driver_version << std::endl;
  return true;
}

// Disconnect from the Robotiq gripper
void RobotiqGripper::disconnect() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

// Check if connected to the gripper
bool RobotiqGripper::isConnected() const { return connected_; }

// Activate the gripper
bool RobotiqGripper::activate() {
  if (!connected_) {
    std::cerr << "Not connected to gripper" << std::endl;
    return false;
  }

  std::cout << "Activating Robotiq gripper..." << std::endl;

  if (getIntValue(RobotiqCommand::ACTIVATION_STATUS) == 1) {
    std::cout << "Gripper already activated." << std::endl;
    return true;
  }

  if (!setValue(RobotiqCommand::ACTIVATION_STATUS, 1)) {
    std::cerr << "Failed to activate gripper" << std::endl;
    return false;
  }
  blockUntilStatus(RobotiqCommand::GRIPPER_STATUS, 3);
  return true;
}

// Get a value from the gripper
std::string RobotiqGripper::getValue(RobotiqCommand cmd) const {
  if (!connected_) {
    throw std::runtime_error("Not connected to gripper");
  }

  std::string sCmd = RobotiqCommandToString(cmd);
  std::string command = "GET " + sCmd + "\n";
  std::string response = sendCommand(command);
  std::string rCmd = response.substr(0, 3);
  if (rCmd != sCmd) {
    throw std::runtime_error("Invalid response from gripper: " + response);
  }

  std::string value = response.substr(4, response.length() - 4);
  return value;
}

int RobotiqGripper::getIntValue(RobotiqCommand cmd) const {
  auto value_str = getValue(cmd);
  try {
    return std::stoi(value_str);
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to parse integer value: " + value_str);
  }
}

// Set a value on the gripper
bool RobotiqGripper::setValue(RobotiqCommand cmd, int value) {
  if (!connected_) {
    std::cerr << "Not connected to gripper" << std::endl;
    return false;
  }

  if (!checkValueInRange(cmd, value)) {
    std::cerr << "Value out of range for command" << std::endl;
    return false;
  }
  if (!commandIsWriteable(cmd)) {
    std::cerr << "Command is read-only" << std::endl;
    return false;
  }
  try {
    std::string command = "SET " + std::string(RobotiqCommandToString(cmd)) +
                          " " + std::to_string(value) + "\n";
    std::string response = sendCommand(command);
    return response == "ack";
  } catch (const std::exception& e) {
    std::cerr << "Error setting value: " << e.what() << std::endl;
    return false;
  }
}

// Move to a specific position
bool RobotiqGripper::moveToPosition(int position, std::optional<int> speed,
                                    std::optional<int> force, bool blocking) {
  if (!connected_) {
    std::cerr << "Not connected to gripper" << std::endl;
    return false;
  }

  // Set speed if provided
  if (speed.has_value()) {
    if (!setValue(RobotiqCommand::SPEED, speed.value())) {
      return false;
    }
  }

  // Set force if provided
  if (force.has_value()) {
    if (!setValue(RobotiqCommand::FORCE, force.value())) {
      return false;
    }
  }

  if (getIntValue(RobotiqCommand::GOTO_STATUS) == 0) {
    if (!setValue(RobotiqCommand::GOTO_STATUS, 1)) {
      return false;
    }
  }

  // Set position
  if (!setValue(RobotiqCommand::POSITION, position)) {
    return false;
  }

  if (blocking) {
    std::cout << "Blocking until position " << position << " is reached..."
              << std::endl;
    blockUntilStatus(RobotiqCommand::OBJECT_DETECTION_STATUS, 0, false);
  }

  return true;
}

bool RobotiqGripper::openGripper(bool blocking) {
  return moveToPosition(0, std::nullopt, std::nullopt, blocking);
}
bool RobotiqGripper::closeGripper(bool blocking) {
  return moveToPosition(255, std::nullopt, std::nullopt, blocking);
}

// Get current position
int RobotiqGripper::getCurrentPosition() const {
  return getIntValue(RobotiqCommand::POSITION);
}

// Private helper: Send command and receive response
std::string RobotiqGripper::sendCommand(const std::string& command) const {
  if (send(socket_fd_, command.c_str(), command.length(), 0) < 0) {
    throw std::runtime_error("Failed to send command");
  }

  return receiveResponse();
}

// Private helper: Receive response
std::string RobotiqGripper::receiveResponse() const {
  char buffer[1024] = {0};
  ssize_t bytes_received = recv(socket_fd_, buffer, 1024, 0);

  if (bytes_received < 0) {
    throw std::runtime_error("Failed to receive response");
  }

  return std::string(buffer, bytes_received);
}

// Private helper: Check if value is in valid range for command
bool RobotiqGripper::checkValueInRange(RobotiqCommand cmd, int value) const {
  switch (cmd) {
    case RobotiqCommand::ACTIVATION_STATUS:
    case RobotiqCommand::GRIPPER_MODE:
    case RobotiqCommand::GOTO_STATUS:
      return value >= 0 && value <= 1;
    case RobotiqCommand::POSITION:
    case RobotiqCommand::POSITION_REQUEST_ECHO:
    case RobotiqCommand::SPEED:
    case RobotiqCommand::FORCE:
    case RobotiqCommand::SOCKET_SLAVE_ID:
      return value >= 0 && value <= 255;
    default:
      return true;  // For read-only commands, any value is acceptable for
                    // internal use
  }
}

bool RobotiqGripper::commandIsWriteable(RobotiqCommand cmd) const {
  switch (cmd) {
    case RobotiqCommand::ACTIVATION_STATUS:
    case RobotiqCommand::GRIPPER_MODE:
    case RobotiqCommand::GOTO_STATUS:
    case RobotiqCommand::POSITION:
    case RobotiqCommand::SPEED:
    case RobotiqCommand::FORCE:
    case RobotiqCommand::SOCKET_SLAVE_ID:
      return true;
    default:
      return false;
  }
}

void RobotiqGripper::blockUntilStatus(RobotiqCommand cmd,
                                         int desired_status, bool set) {
  // Poll until status is first cleared, then set
  auto start = std::chrono::steady_clock::now();
  // Wait for status to clear
  if ((getIntValue(cmd) == desired_status) == set) {
    while (std::chrono::steady_clock::now() - start <
           std::chrono::seconds(timeout_ > 0 ? timeout_ : 10)) {
      if ((getIntValue(cmd) == desired_status) != set) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_));
    }
  }
  // Wait for status to set
  while (std::chrono::steady_clock::now() - start <
         std::chrono::seconds(timeout_ > 0 ? timeout_ : 10)) {
    if ((getIntValue(cmd) == desired_status) == set) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_));
  }
}

}  // namespace robotiq
}  // namespace drivers