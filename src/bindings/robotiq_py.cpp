#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "src/robotiq_gripper.h"

namespace py = pybind11;

PYBIND11_MODULE(robotiq_cpp, m) {
  m.doc() = "Python bindings for Robotiq Gripper C++ driver";

  // Bind the RobotiqCommand enum
  py::enum_<driver::robotiq::RobotiqCommand>(m, "RobotiqCommand")
      .value("ACTIVATION_STATUS",
             driver::robotiq::RobotiqCommand::ACTIVATION_STATUS)
      .value("GRIPPER_MODE", driver::robotiq::RobotiqCommand::GRIPPER_MODE)
      .value("GOTO_STATUS", driver::robotiq::RobotiqCommand::GOTO_STATUS)
      .value("GRIPPER_STATUS", driver::robotiq::RobotiqCommand::GRIPPER_STATUS)
      .value("OBJECT_DETECTION_STATUS",
             driver::robotiq::RobotiqCommand::OBJECT_DETECTION_STATUS)
      .value("FAULT_STATUS", driver::robotiq::RobotiqCommand::FAULT_STATUS)
      .value("POSITION_REQUEST_ECHO",
             driver::robotiq::RobotiqCommand::POSITION_REQUEST_ECHO)
      .value("POSITION", driver::robotiq::RobotiqCommand::POSITION)
      .value("SPEED", driver::robotiq::RobotiqCommand::SPEED)
      .value("FORCE", driver::robotiq::RobotiqCommand::FORCE)
      .value("CURRENT", driver::robotiq::RobotiqCommand::CURRENT)
      .value("SERIAL_NUMBER", driver::robotiq::RobotiqCommand::SERIAL_NUMBER)
      .value("PRODUCTION_YEAR",
             driver::robotiq::RobotiqCommand::PRODUCTION_YEAR)
      .value("FIRMWARE_VERSION",
             driver::robotiq::RobotiqCommand::FIRMWARE_VERSION)
      .value("DRIVER_VERSION", driver::robotiq::RobotiqCommand::DRIVER_VERSION)
      .value("SOCKET_SLAVE_ID",
             driver::robotiq::RobotiqCommand::SOCKET_SLAVE_ID)
      .export_values();

  // Bind the RobotiqCommandToString function
  m.def("robotiq_command_to_string", &driver::robotiq::RobotiqCommandToString,
        "Convert RobotiqCommand enum to string representation", py::arg("cmd"));

  // Bind the RobotiqGripper class
  py::class_<driver::robotiq::RobotiqGripper>(m, "RobotiqGripper")
      .def(py::init<std::string, int, int, int>(),
           "Constructor for RobotiqGripper", py::arg("ip_address"),
           py::arg("port") = PORT, py::arg("timeout") = -1,
           py::arg("poll_interval") = 100)
      .def("connect", &driver::robotiq::RobotiqGripper::connect,
           "Connect to the Robotiq gripper")
      .def("disconnect", &driver::robotiq::RobotiqGripper::disconnect,
           "Disconnect from the Robotiq gripper")
      .def("is_connected", &driver::robotiq::RobotiqGripper::isConnected,
           "Check if connected to the gripper")
      .def("activate", &driver::robotiq::RobotiqGripper::activate,
           "Activate the gripper")
      .def("get_value", &driver::robotiq::RobotiqGripper::getValue,
           "Get a value from the gripper", py::arg("cmd"))
      .def("get_int_value", &driver::robotiq::RobotiqGripper::getIntValue,
           "Get an integer value from the gripper", py::arg("cmd"))
      .def("set_value", &driver::robotiq::RobotiqGripper::setValue,
           "Set a value on the gripper", py::arg("cmd"), py::arg("value"))
      .def("move_to_position",
           &driver::robotiq::RobotiqGripper::moveToPosition,
           "Move gripper to a specific position", py::arg("position"),
           py::arg("speed") = py::none(), py::arg("force") = py::none(),
           py::arg("blocking") = false)
      .def("open_gripper", &driver::robotiq::RobotiqGripper::openGripper,
           "Open the gripper", py::arg("blocking") = false)
      .def("close_gripper", &driver::robotiq::RobotiqGripper::closeGripper,
           "Close the gripper", py::arg("blocking") = false)
      .def("get_current_position",
           &driver::robotiq::RobotiqGripper::getCurrentPosition,
           "Get current position of the gripper")
      .def("is_faulted", &driver::robotiq::RobotiqGripper::IsFaulted,
           "Check if the gripper is in a faulted state");

  // Add PORT constant
  m.attr("PORT") = PORT;
}