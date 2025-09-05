import time
import FeetechArmClient

FEETECH_ADDR = "COM9"
FEETECH_BAUD_RATE = 1000000
FEETECH_SERVO_ID_LIST = [1, 2, 3, 4, 5, 6]
arm = FeetechArmClient.FeetechArmClient()
arm.connect(FEETECH_ADDR, FEETECH_BAUD_RATE, FEETECH_SERVO_ID_LIST)
# arm.lock_joint_group(FEETECH_SERVO_ID_LIST)
arm.release_joint_group(FEETECH_SERVO_ID_LIST)
# arm.set_joint_angle_group(FEETECH_SERVO_ID_LIST, [180, 180, 180, 180, 180, 180])
time.sleep(0.5)

while True:
    joint_angle = arm.get_joint_angle_group(FEETECH_SERVO_ID_LIST)
    # joint_angle = arm.get_joint_angle(6)
    print(joint_angle)
    time.sleep(0.02)
