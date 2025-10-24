import robotiq_cpp as rg

if __name__ == "__main__":
    gripper = rg.RobotiqGripper(ip_address="192.168.199.101")
    gripper.connect()
    gripper.activate()
    position = gripper.get_current_position()
    if position > 100:
        position = 20
    else:
        position = 150
    gripper.move_to_position(position, speed=100, force=None, blocking=True)
    print(f"Current Gripper Position: {position}")
    gripper.open_gripper(True)
    gripper.close_gripper(True)
    gripper.disconnect()