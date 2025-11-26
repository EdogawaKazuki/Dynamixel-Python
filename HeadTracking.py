import time
import DynamixelArmClient
import UDPServer


def set_joint_angle_group(angle_list):
    cmd_list = angle_list[:2]
    cmd_list[0] = 180 - cmd_list[0]
    cmd_list[1] = cmd_list[1] + 180
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, cmd_list)


DYNAMIXEL_ADDR = "COM16"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [15, 14]
arm = DynamixelArmClient.DynamixelArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)
arm.lock_joint_group(DYNAMIXEL_SERVO_ID_LIST)
arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, [180, 180])
time.sleep(0.5)

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 1235
receiver = UDPServer.UDPServer(SERVER_HOST, SERVER_PORT)
receiver.cmd_callback = set_joint_angle_group
receiver.start_server()
