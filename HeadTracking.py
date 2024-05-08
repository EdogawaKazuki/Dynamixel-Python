
import time
import DynamixelArmClient
DYNAMIXEL_ADDR = "COM12"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [14, 15]
arm = RobotArmClient.DynamixelArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)
time.sleep(0.5)
while True:
    tmp = arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)
    angles = [tmp[0][0] - 180, tmp[1][0] - 180]
    print(angles)
    time.sleep(0.1)