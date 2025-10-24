#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "src/robotiq_gripper.h"

namespace py = pybind11;

PYBIND11_MODULE(robotiq_cpp, m) {
  m.doc() = "Python bindings for Robotiq Gripper C++ driver";

  // Bind the RobotiqCommand enum
  py::enum_<drivers::robotiq::RobotiqCommand>(m, "RobotiqCommand")
      .value("ACTIVATION_STATUS",
             drivers::robotiq::RobotiqCommand::ACTIVATION_STATUS)
      .value("GRIPPER_MODE", drivers::robotiq::RobotiqCommand::GRIPPER_MODE)
      .value("GOTO_STATUS", drivers::robotiq::RobotiqCommand::GOTO_STATUS)
      .value("GRIPPER_STATUS", drivers::robotiq::RobotiqCommand::GRIPPER_STATUS)
      .value("OBJECT_DETECTION_STATUS",
             drivers::robotiq::RobotiqCommand::OBJECT_DETECTION_STATUS)
      .value("FAULT_STATUS", drivers::robotiq::RobotiqCommand::FAULT_STATUS)
      .value("POSITION_REQUEST_ECHO",
             drivers::robotiq::RobotiqCommand::POSITION_REQUEST_ECHO)
      .value("POSITION", drivers::robotiq::RobotiqCommand::POSITION)
      .value("SPEED", drivers::robotiq::RobotiqCommand::SPEED)
      .value("FORCE", drivers::robotiq::RobotiqCommand::FORCE)
      .value("CURRENT", drivers::robotiq::RobotiqCommand::CURRENT)
      .value("SERIAL_NUMBER", drivers::robotiq::RobotiqCommand::SERIAL_NUMBER)
      .value("PRODUCTION_YEAR",
             drivers::robotiq::RobotiqCommand::PRODUCTION_YEAR)
      .value("FIRMWARE_VERSION",
             drivers::robotiq::RobotiqCommand::FIRMWARE_VERSION)
      .value("DRIVER_VERSION", drivers::robotiq::RobotiqCommand::DRIVER_VERSION)
      .value("SOCKET_SLAVE_ID",
             drivers::robotiq::RobotiqCommand::SOCKET_SLAVE_ID)
      .export_values();

  // Bind the RobotiqCommandToString function
  m.def("robotiq_command_to_string", &drivers::robotiq::RobotiqCommandToString,
        "Convert RobotiqCommand enum to string representation", py::arg("cmd"));

  // Bind the RobotiqGripper class
  py::class_<drivers::robotiq::RobotiqGripper>(m, "RobotiqGripper")
      .def(py::init<std::string, int, int, int>(),
           "Constructor for RobotiqGripper", py::arg("ip_address"),
           py::arg("port") = PORT, py::arg("timeout") = -1,
           py::arg("poll_interval") = 100)
      .def("connect", &drivers::robotiq::RobotiqGripper::connect,
           "Connect to the Robotiq gripper")
      .def("disconnect", &drivers::robotiq::RobotiqGripper::disconnect,
           "Disconnect from the Robotiq gripper")
      .def("is_connected", &drivers::robotiq::RobotiqGripper::isConnected,
           "Check if connected to the gripper")
      .def("activate", &drivers::robotiq::RobotiqGripper::activate,
           "Activate the gripper")
      .def("get_value", &drivers::robotiq::RobotiqGripper::getValue,
           "Get a value from the gripper", py::arg("cmd"))
      .def("set_value", &drivers::robotiq::RobotiqGripper::setValue,
           "Set a value on the gripper", py::arg("cmd"), py::arg("value"))
      .def("move_to_position",
           &drivers::robotiq::RobotiqGripper::moveToPosition,
           "Move gripper to a specific position", py::arg("position"),
           py::arg("speed") = py::none(), py::arg("force") = py::none(),
           py::arg("blocking") = false)
      .def("open_gripper", &drivers::robotiq::RobotiqGripper::openGripper,
           "Open the gripper", py::arg("blocking") = false)
      .def("close_gripper", &drivers::robotiq::RobotiqGripper::closeGripper,
           "Close the gripper", py::arg("blocking") = false)
      .def("get_current_position",
           &drivers::robotiq::RobotiqGripper::getCurrentPosition,
           "Get current position of the gripper");

  // Add PORT constant
  m.attr("PORT") = PORT;
}