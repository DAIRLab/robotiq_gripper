/*
 * Robotiq Gripper Driver Header
 *
 * This header file contains the interface for controlling Robotiq grippers
 * via TCP/IP communication. The driver provides high-level control functions
 * for connecting, activating, and controlling gripper movement.
 *
 * Author: DAIR Lab
 * Version: 1.0.0
 */

#pragma once

#include <iostream>
#include <stdexcept>
#include <string>

#include <optional>

/* Default port for Robotiq gripper communication */
#define PORT 63352

/* Default polling interval for blocking operations in milliseconds */
#define BLOCKING_POLL_INTERVAL_MS 100

namespace driver {
namespace robotiq {

/*
 * Enumeration of available Robotiq gripper commands.
 *
 * These commands correspond to the gripper's internal registers and control
 * various aspects of gripper operation including activation, movement,
 * status reporting, and configuration.
 */
enum class RobotiqCommand {
  ACTIVATION_STATUS,       /* Gripper activation status (0=reset, 1=activate) */
  GRIPPER_MODE,            /* Gripper operation mode */
  GOTO_STATUS,             /* Movement command (0=stop, 1=go to position) */
  GRIPPER_STATUS,          /* Current gripper status (read-only) */
  OBJECT_DETECTION_STATUS, /* Object detection state (read-only) */
  FAULT_STATUS,            /* Fault/error status (read-only) */
  POSITION_REQUEST_ECHO,   /* Echo of requested position */
  POSITION,                /* Target or current position (0-255) */
  SPEED,                   /* Movement speed (0-255) */
  FORCE,                   /* Gripping force (0-255) */
  CURRENT,                 /* Motor current (read-only) */
  SERIAL_NUMBER,           /* Gripper serial number (read-only) */
  PRODUCTION_YEAR,         /* Manufacturing year (read-only) */
  FIRMWARE_VERSION,        /* Firmware version (read-only) */
  DRIVER_VERSION,          /* Driver version (read-only) */
  SOCKET_SLAVE_ID,         /* Socket slave identifier */
};

/*
 * Convert RobotiqCommand enum to string representation.
 *
 * Converts the command enumeration values to their corresponding
 * string representations used in the gripper communication protocol.
 *
 * @param cmd The RobotiqCommand enum value to convert
 * @return Const char pointer to the string representation
 * @note This function is noexcept and constexpr for compile-time evaluation
 */
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

/*
 * RobotiqGripper Class
 *
 * Main interface class for controlling Robotiq grippers via TCP/IP.
 * This class provides high-level methods for connecting to, activating,
 * and controlling Robotiq grippers. It handles socket communication,
 * command formatting, and response parsing.
 *
 * Features:
 * - TCP/IP socket communication with configurable timeout
 * - Automatic connection management and error handling
 * - High-level gripper control (activate, move, open, close)
 * - Low-level register access for advanced control
 * - Blocking and non-blocking movement operations
 * - Comprehensive status and fault reporting
 *
 * Usage:
 *   RobotiqGripper gripper("192.168.1.100");
 *   gripper.connect();
 *   gripper.activate();
 *   gripper.moveToPosition(100, 50, 100);  // position, speed, force
 *   gripper.disconnect();
 */
class RobotiqGripper {
 public:
  /*
   * Constructor for RobotiqGripper.
   *
   * Creates a new gripper instance configured for the specified IP address
   * and communication parameters.
   *
   * @param ip_address IP address of the gripper (e.g., "192.168.1.100")
   * @param port TCP port number (default: 63352)
   * @param timeout Socket timeout in seconds (-1 for no timeout)
   * @param poll_interval Polling interval for blocking operations in
   * milliseconds
   */
  RobotiqGripper(std::string ip_address, int port = PORT, int timeout = -1,
                 int poll_interval = BLOCKING_POLL_INTERVAL_MS);

  /*
   * Destructor for RobotiqGripper.
   *
   * Automatically disconnects from the gripper if still connected.
   */
  ~RobotiqGripper();

  /*
   * Establish connection to the gripper.
   *
   * Creates a TCP socket and attempts to connect to the gripper at the
   * specified IP address and port. Sets up socket timeouts if configured.
   *
   * @return true if connection successful, false otherwise
   */
  bool connect();

  /*
   * Disconnect from the gripper.
   *
   * Closes the TCP socket connection and cleans up resources.
   */
  void disconnect();

  /*
   * Check if connected to the gripper.
   *
   * @return true if currently connected, false otherwise
   */
  bool isConnected() const;

  /*
   * Activate the gripper.
   *
   * Sends the activation command to initialize the gripper for operation.
   * The gripper must be activated before any movement commands will work.
   *
   * @return true if activation command sent successfully, false otherwise
   */
  bool activate();

  /*
   * Get a value from the gripper as a string.
   *
   * Reads the current value of the specified command/register from the gripper
   * and returns it as a string. Useful for debugging or when exact string
   * representation is needed.
   *
   * @param cmd The command/register to read
   * @return String value from the gripper
   * @throws std::runtime_error if not connected or communication fails
   */
  std::string getValue(RobotiqCommand cmd) const;

  /*
   * Get a value from the gripper as an integer.
   *
   * Reads the current value of the specified command/register from the gripper
   * and converts it to an integer. This is the most commonly used method for
   * reading gripper status and sensor values.
   *
   * @param cmd The command/register to read
   * @return Integer value from the gripper
   * @throws std::runtime_error if not connected or communication fails
   */
  int getIntValue(RobotiqCommand cmd) const;

  /*
   * Set a value on the gripper.
   *
   * Writes the specified value to the given command/register on the gripper.
   * Performs range checking to ensure the value is valid for the command.
   *
   * @param cmd The command/register to write to
   * @param value The value to write (range depends on command)
   * @return true if value set successfully, false otherwise
   */
  bool setValue(RobotiqCommand cmd, int value);

  /*
   * Move gripper to a specific position.
   *
   * Commands the gripper to move to the specified position with optional
   * speed and force parameters. Position 0 is fully open, 255 is fully closed.
   *
   * @param position Target position (0-255, 0=open, 255=closed)
   * @param speed Movement speed (0-255, nullopt for default)
   * @param force Gripping force (0-255, nullopt for default)
   * @param blocking If true, blocks until movement completes
   * @return true if movement command sent successfully, false otherwise
   */
  bool moveToPosition(int position, std::optional<int> speed = std::nullopt,
                      std::optional<int> force = std::nullopt,
                      bool blocking = false);

  /*
   * Open the gripper completely.
   *
   * Convenience method to move the gripper to the fully open position (0).
   *
   * @param blocking If true, blocks until movement completes
   * @return true if command sent successfully, false otherwise
   */
  bool openGripper(bool blocking = false);

  /*
   * Close the gripper completely.
   *
   * Convenience method to move the gripper to the fully closed position (255).
   *
   * @param blocking If true, blocks until movement completes
   * @return true if command sent successfully, false otherwise
   */
  bool closeGripper(bool blocking = false);

  /*
   * Get the current position of the gripper.
   *
   * @return Current position value (0-255, 0=open, 255=closed)
   * @throws std::runtime_error if not connected or communication fails
   */
  int getCurrentPosition() const;

  /*
   * Check if the gripper is in a fault state.
   *
   * @return true if gripper has reported a fault, false otherwise
   * @throws std::runtime_error if not connected or communication fails
   */
  bool IsFaulted() const;

 private:
  /* Private member variables */
  std::string ip_address_; /* IP address of the gripper */
  int port_;               /* TCP port number for communication */
  int timeout_;            /* Socket timeout in seconds */
  int socket_fd_;          /* File descriptor for TCP socket */
  bool connected_;         /* Connection state flag */
  int poll_interval_;      /* Polling interval for blocking operations (ms) */

  /*
   * Send a command string to the gripper.
   *
   * Transmits the command over the TCP socket and returns the response.
   *
   * @param command Command string to send
   * @return Response string from gripper
   * @throws std::runtime_error if send/receive fails
   */
  std::string sendCommand(const std::string& command) const;

  /*
   * Receive a response from the gripper.
   *
   * Reads data from the TCP socket with timeout handling.
   *
   * @return Response string from gripper
   * @throws std::runtime_error if receive fails or times out
   */
  std::string receiveResponse() const;

  /*
   * Check if a value is within valid range for a command.
   *
   * Validates that the given value is appropriate for the specified
   * command based on the gripper's register specifications.
   *
   * @param cmd Command to check value for
   * @param value Value to validate
   * @return true if value is valid, false otherwise
   */
  bool checkValueInRange(RobotiqCommand cmd, int value) const;

  /*
   * Check if a command supports write operations.
   *
   * Determines whether the specified command can be written to or
   * if it is read-only.
   *
   * @param cmd Command to check
   * @return true if command is writeable, false if read-only
   */
  bool commandIsWriteable(RobotiqCommand cmd) const;

  /*
   * Block execution until gripper reaches desired status.
   *
   * Polls the specified command until it reaches the desired value.
   * Used for implementing blocking movement operations.
   *
   * @param cmd Command to monitor
   * @param desired_status Target status value to wait for
   * @param set If true, waits for status to be set; if false, waits for clear
   */
  void blockUntilStatus(RobotiqCommand cmd, int desired_status,
                        bool set = true);
};

}  // namespace robotiq
}  // namespace drivers