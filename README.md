# Robotiq Gripper Driver

A comprehensive C++ driver with Python bindings for controlling Robotiq grippers via TCP/IP communication.

## Quick Start

### Prerequisites

- **Bazel 6.0+**: [Install Bazel](https://bazel.build/install)
- **C++17 compatible compiler**: GCC 8+, Clang 10+
- **Python 3.7+**: For Python bindings (optional)
- **Linux**: Currently supports Linux systems

### Building the Project

```bash
# Clone the repository
git clone https://github.com/DAIRLab/robotiq_gripper.git
cd robotiq_gripper

# Build all targets
bazel build //...
```

## Usage

### C++ Interface

```cpp
#include "src/robotiq_gripper.h"

using namespace driver::robotiq;

int main() {
    // Create gripper instance
    RobotiqGripper gripper("192.168.1.100");
    
    // Connect and activate
    if (gripper.connect()) {
        gripper.activate();
        
        // Move to half-open position
        gripper.moveToPosition(100, 50, 100);  // position, speed, force
        
        // Close gripper with blocking operation
        gripper.closeGripper(true);  // blocking=true
        
        // Check if object is grasped
        if (!gripper.IsFaulted()) {
            std::cout << "Current position: " << gripper.getCurrentPosition() << std::endl;
        }
        
        gripper.disconnect();
    }
    return 0;
}
```

### Python Interface

```python
from robotiq_cpp import RobotiqGripper

# Manual connection management
gripper = RobotiqGripper("192.168.1.100", port=63352, timeout=10)
gripper.connect()
gripper.activate()
gripper.openGripper()
gripper.disconnect()
```

## API Reference

### Core Classes

#### `RobotiqGripper`

The main interface class for gripper control.

**Constructor:**
```cpp
RobotiqGripper(std::string ip_address, int port = 63352, int timeout = -1, int poll_interval = 100)
```

**Key Methods:**
- `bool connect()` - Establish TCP connection
- `void disconnect()` - Close connection
- `bool activate()` - Initialize gripper for operation
- `bool moveToPosition(int pos, optional<int> speed, optional<int> force, bool blocking)` - Move to position
- `bool openGripper(bool blocking)` - Open gripper completely
- `bool closeGripper(bool blocking)` - Close gripper completely
- `int getCurrentPosition()` - Get current position (0-255)
- `bool IsFaulted()` - Check fault status

#### `RobotiqCommand` (Enum)

Available gripper commands for low-level access:

| Command | Description | Range | Access |
|---------|-------------|-------|--------|
| `ACTIVATION_STATUS` | Gripper activation | 0-1 | R/W |
| `POSITION` | Target/current position | 0-255 | R/W |
| `SPEED` | Movement speed | 0-255 | R/W |
| `FORCE` | Gripping force | 0-255 | R/W |
| `GRIPPER_STATUS` | Current status | - | R |
| `OBJECT_DETECTION_STATUS` | Object detection | - | R |
| `FAULT_STATUS` | Error status | - | R |

### Position Values

- **0**: Fully open
- **255**: Fully closed
- **100-150**: Typical object grasping range

### Speed Values

- **0**: Minimum speed (may not move)
- **255**: Maximum speed
- **50-150**: Recommended range for most applications

### Force Values

- **0**: Minimum force
- **255**: Maximum force
- **100-200**: Typical range for object manipulation

## Configuration

### Network Setup

Ensure your Robotiq gripper is configured for TCP/IP communication:

1. **Default Settings:**
   - IP Address: `192.168.1.100` (configurable on gripper)
   - Port: `63352` (standard Robotiq port)
   - Protocol: TCP/IP

2. **Network Configuration:**
   ```bash
   # Ensure network connectivity
   ping 192.168.1.100
   ```
