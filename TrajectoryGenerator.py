import math
import time

from pyfirmata import ArduinoMega

import RobotArmClient

DYNAMIXEL_ADDR = "COM10"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [11, 12, 14]


# init robot arm
arm = RobotArmClient.RobotArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)

locked_servo_id = []
is_locked = False
for arm_id in DYNAMIXEL_SERVO_ID_LIST:
    if arm.lock_servo(arm_id):
        locked_servo_id.append(arm_id)

if len(locked_servo_id) != len(DYNAMIXEL_SERVO_ID_LIST):
    print("Please check dynamixel servo status! Exiting...")
    arm.disconnect()
    exit(-1)
else:
    print("All servo connected!")

# init the LED Controller
board = ArduinoMega('COM8')

r = board.get_pin("d:8:p")
g = board.get_pin("d:9:p")
b = board.get_pin("d:10:p")

def set_color(color):
    r.write(color[0])
    g.write(color[1])
    b.write(color[2])

# This function converts task space to joint space
# Input: Task space coord. For example TaskSpace2JointSpace(10, 20, 10)
# Output: A list of joint angles. For example [-26.56505117707799, 43.33991400145487, -65.29185017692065]
def task_space_2_joint_space(x, y, z):
    y = -y
    angles = [0 for i in range(3)]
    l1 = 7.7
    l2 = 12.7
    l3 = 2.5
    l23 = math.sqrt(l2 * l2 + l3 * l3)
    l4 = 12.3
    alpha = math.atan(l3/l2)

    if x == 0:
        angles[0] = math.pi / 2
    else:
        if x > 0:
            angles[0] = math.atan(-y/x)
        else:
            angles[0] = math.pi - math.atan(y/x)

    A = -y * math.sin(angles[0]) + x * math.cos(angles[0])

    B = z - l1

    tmp = (A * A + B * B - (l23 * l23 + l4 * l4)) / (2 * l23 * l4);
    if tmp < -1:
        tmp = -0.999999
    if tmp > 1:
        tmp = 0.99999
    angles[2] = -math.acos(tmp)
    if (A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])) > 0:
        angles[1] = math.atan((B * (l23 + l4 * math.cos(angles[2])) - A * l4 * math.sin(angles[2])) /
                               (A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])))
    else:
        angles[1] = math.pi - math.atan((B * (l23 + l4 * math.cos(angles[2])) - A * l4 * math.sin(angles[2])) /
                                          -(A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])))

    angles[0] = angles[0] / math.pi * 180 - 90
    angles[1] = (angles[1] + alpha) / math.pi * 180
    angles[2] = (angles[2] - alpha) / math.pi * 180
    return angles


# Modify this function to generate trajectories
# Frequency is 50Hz (i.e. each frame costs 0.02s)
def generate_trajectory():
    joint_0_list = []
    joint_1_list = []
    joint_2_list = []
    light_list = []

    # check from here
    # Example Code
    # move from (0, 10, 0) to (10, 20, 10) in 1 second by linear
    # from 0s to 1s, 1(second) times 50(frequency) is number of total frame,
    # then divided by 50 for converting it back to second
    duration = 1
    for t in [i / 50 for i in range(duration * 50)]:
        # init lighting flag
        light = 0

        # ToDo
        # update coords
        x = 0 + 10 * t
        y = 10 + 10 * t
        z = 0 + 10 * t

        # convert task space to joint space
        angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)

        # ToDo
        # update light,
        if 0.25 <= t <= 0.5:
            light = [1, 0, 1]
        else:
            light = [0, 1, 1]

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
        light_list.append(light)

    # part 2
    duration = 1
    for t in [i / 50 for i in range(duration * 50)]:
        # init lighting flag
        light = 0

        # ToDo
        # update coords
        x = 10 - 10 * t
        y = 20 - 10 * t
        z = 10 - 10 * t

        # convert task space to joint space
        angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)

        # ToDo
        # update light,
        if 0.25 <= t <= 0.5 or 0.75 <= t <= 1:
            light = [1, 0, 1]
        else:
            light = [0, 1, 1]

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
        light_list.append(light)
        # end




    return [joint_0_list, joint_1_list, joint_2_list, light_list]


trajs = generate_trajectory()
print("Moving to start Position")
arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, [180 + trajs[0][0], 270 - trajs[1][0], 180 + trajs[2][0]])
time.sleep(2)
print("Go")
for i in range(len(trajs[0])):
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3],
                              [180 + trajs[0][i], 270 - trajs[1][i], 180 + trajs[2][i]])
    set_color(trajs[3][i])
    time.sleep(0.02)



# result = "angle;"
# trajs = generate_trajectory()
# for traj in trajs:
#     result += ",".join(traj)
#     result += ";"

# f = open("traj.txt", mode="w")
# f.write(result)
# f.close()


