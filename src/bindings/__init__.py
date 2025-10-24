"""
Robotiq Gripper Python Package.

This package provides Python bindings for controlling Robotiq grippers.
"""

# Import the C++ extension module
try:
    from . import robotiq_cpp
except ImportError:
    # Fallback for when running directly
    import robotiq_cpp

# Re-export the main classes and functions
RobotiqGripper = robotiq_cpp.RobotiqGripper
RobotiqCommand = robotiq_cpp.RobotiqCommand
robotiq_command_to_string = robotiq_cpp.robotiq_command_to_string
PORT = robotiq_cpp.PORT

__version__ = "1.0.0"
__all__ = [
    "RobotiqGripper",
    "RobotiqCommand", 
    "robotiq_command_to_string",
    "PORT",
]
