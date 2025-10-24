from robotiq_cpp import RobotiqGripper

gripper = RobotiqGripper("192.168.199.101", port=63352, timeout=2)
gripper.connect()
gripper.activate()
gripper.close_gripper(True)
gripper.open_gripper(True)
print("Is faulted:", gripper.is_faulted())
gripper.disconnect()