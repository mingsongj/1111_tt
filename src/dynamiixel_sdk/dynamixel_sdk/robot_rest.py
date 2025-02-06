import robot_mod

robot = robot_mod.USB2Dynamixel_Device(servo_ids=list(range(0, 12)), DEVICENAME = '/dev/ttyUSB0')
robot.disable_torque(robot.servo_ids)
