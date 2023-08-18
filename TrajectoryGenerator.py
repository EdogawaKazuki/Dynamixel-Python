from math import *
import time

from pyfirmata import ArduinoMega

import RobotArmClient

board = ArduinoMega('COM16')
DYNAMIXEL_ADDR = "COM10"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [11, 12, 14]


# # init robot arm
arm = RobotArmClient.RobotArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)
time.sleep(0.5)
tmp = arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)
print(tmp)
angles = [tmp[0][0] - 180, 270 - tmp[1][0], tmp[2][0] - 180]

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

r = board.get_pin("d:8:p")
g = board.get_pin("d:9:p")
b = board.get_pin("d:10:p")

def set_color(color):
    # return
    r.write((1 - color[0] / 50))
    g.write((1 - color[1] / 50))
    b.write((1 - color[2] / 50))
    # print((1 - color[0] / 3), (1 - color[1] / 3), (1 - color[2] / 3))
# set_color([0,0,0])
# This function converts task space to joint space
# Input: Task space coord. For example TaskSpace2JointSpace(10, 20, 10)
# Output: A list of joint angles. For example [-26.56505117707799, 43.33991400145487, -65.29185017692065]
l1 = 7.7
l2 = 12.7
l3 = 2.5
l4 = 11.6
def task_space_2_joint_space(x, y, z):
    y = -y
    angles = [0 for i in range(3)]
    l23 = sqrt(l2 * l2 + l3 * l3)
    alpha = atan(l3/l2)

    if x == 0:
        angles[0] = pi / 2
    else:
        if x > 0:
            angles[0] = atan(-y/x)
        else:
            angles[0] = pi - atan(y/x)

    A = -y * sin(angles[0]) + x * cos(angles[0])

    B = z - l1

    tmp = (A * A + B * B - (l23 * l23 + l4 * l4)) / (2 * l23 * l4);
    if tmp < -1:
        tmp = -0.999999
    if tmp > 1:
        tmp = 0.99999
    angles[2] = -acos(tmp)
    if (A * (l23 + l4 * cos(angles[2])) + B * l4 * sin(angles[2])) > 0:
        angles[1] = atan((B * (l23 + l4 * cos(angles[2])) - A * l4 * sin(angles[2])) /
                               (A * (l23 + l4 * cos(angles[2])) + B * l4 * sin(angles[2])))
    else:
        angles[1] = pi - atan((B * (l23 + l4 * cos(angles[2])) - A * l4 * sin(angles[2])) /
                                          -(A * (l23 + l4 * cos(angles[2])) + B * l4 * sin(angles[2])))

    angles[0] = angles[0] / pi * 180 - 90
    angles[1] = (angles[1] + alpha) / pi * 180
    angles[2] = (angles[2] - alpha) / pi * 180
    return angles

def joint_space_2_task_space(joints):
    Angle1Sin = sin(radians(joints[0]))
    Angle1Cos = cos(radians(joints[0]))
    Angle2Sin = sin(radians(joints[1]))
    Angle2Cos = cos(radians(joints[1]))
    Angle23Sin = sin(radians(joints[1] + joints[2]))
    Angle23Cos = cos(radians(joints[1] + joints[2]))
    x = -Angle1Sin * Angle23Cos * l4 + (
                -l2 * Angle1Sin * Angle2Cos - l3 * Angle1Sin * Angle2Sin) + 0.001
    y = Angle1Cos * Angle23Cos * l4 + (
                l2 * Angle1Cos * Angle2Cos + l3 * Angle1Cos * Angle2Sin) + 0.001
    z = Angle23Sin * l4 + (l2 * Angle2Sin - l3 * Angle2Cos) + l1
    return [x,y,z]

def move_to(init_pos, end_pos, light_on):
    joint_0_list = []
    joint_1_list = []
    joint_2_list = []
    light_list = []

    current_pos = init_pos
    init_pos = end_pos
    duration = 1
    for t in [i / 50 for i in range(duration * 50)]:
        # init lighting flag
        light = [light_on for i in range(3)]

        # ToDo
        # update coords
        x = current_pos[0] + (init_pos[0] - current_pos[0]) * t
        y = current_pos[1] + (init_pos[1] - current_pos[1]) * t
        z = current_pos[2] + (init_pos[2] - current_pos[2]) * t

        # convert task space to joint space
        angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)

        # ToDo
        # update light,

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
        light_list.append(light)
    return [joint_0_list, joint_1_list, joint_2_list, light_list]

    

# Modify this function to generate trajectories
# Frequency is 50Hz (i.e. each frame costs 0.02s)
def generate_trajectory():
    joint_0_list = []
    joint_1_list = []
    joint_2_list = []
    light_list = []
    # following part 

    def get_x(t):

        return 5 * (cos(16*t) + (cos(6*t))/2 + (sin(10*t))/3)

    def get_y(t):

        return 7.5

    def get_z(t):

        return 5 * (sin(16*t) + (sin(6*t))/2 + (cos(10*t))/3) + 20

    duration = 10

    for t in [i / 100 for i in range(int(duration * 100))]:

        # init lighting flag

        light = [0, 1, 0]

    

        # convert task space to joint space

        angle0, angle1, angle2 = task_space_2_joint_space(get_x(t), get_y(t), get_z(t))

    

        # push all frame to the frame list

        joint_0_list.append(angle0)

        joint_1_list.append(angle1)

        joint_2_list.append(angle2)

        light_list.append(light)

    return [joint_0_list, joint_1_list, joint_2_list, light_list]


trajs = generate_trajectory()
trajs = move_to(joint_space_2_task_space(angles), joint_space_2_task_space([trajs[0][0], trajs[1][0], trajs[2][0]]), 0)
print("Moving to start Position")

for i in range(len(trajs[0])):
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3],
                              [180 + trajs[0][i], 270 - trajs[1][i], 180 + trajs[2][i]])
    time.sleep(0.02)
set_color([0, 0, 0])
# arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, [180 + trajs[0][0], 270 - trajs[1][0], 180 + trajs[2][0]])
# set_color([0, 0, 0])
time.sleep(2)


trajs = generate_trajectory()
print("Go")

for i in range(len(trajs[0])):
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3],
                              [180 + trajs[0][i], 270 - trajs[1][i], 180 + trajs[2][i]])
    set_color(trajs[3][i])
    time.sleep(0.02)
set_color([0, 0, 0])



# result = "angle;"
# trajs = generate_trajectory()
# for traj in trajs[:-1]:
#     strs = []
#     for ele in traj:
#         strs.append(str(ele))
#     result += ",".join(strs)
#     result += ";"

# f = open("traj.txt", mode="w")
# f.write(result)
# f.close()


