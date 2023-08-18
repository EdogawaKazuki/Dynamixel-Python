import math
import time

from pyfirmata import ArduinoMega

import RobotArmClient

board = ArduinoMega('COM6')
DYNAMIXEL_ADDR = "COM7"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [11, 12, 14]


# init robot arm
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
    r.write((1 - color[0] / 10))
    g.write((1 - color[1] / 10))
    b.write((1 - color[2] / 10))
    print((1 - color[0] / 3), (1 - color[1] / 3), (1 - color[2] / 3))
set_color([0,0,0])
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
    l23 = math.sqrt(l2 * l2 + l3 * l3)
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

def joint_space_2_task_space(joints):
    Angle1Sin = math.sin(math.radians(joints[0]))
    Angle1Cos = math.cos(math.radians(joints[0]))
    Angle2Sin = math.sin(math.radians(joints[1]))
    Angle2Cos = math.cos(math.radians(joints[1]))
    Angle23Sin = math.sin(math.radians(joints[1] + joints[2]))
    Angle23Cos = math.cos(math.radians(joints[1] + joints[2]))
    x = -Angle1Sin * Angle23Cos * l4 + (
                -l2 * Angle1Sin * Angle2Cos - l3 * Angle1Sin * Angle2Sin) + 0.001
    y = Angle1Cos * Angle23Cos * l4 + (
                l2 * Angle1Cos * Angle2Cos + l3 * Angle1Cos * Angle2Sin) + 0.001
    z = Angle23Sin * l4 + (l2 * Angle2Sin - l3 * Angle2Cos) + l1
    return [x,y,z]

def move_to(init_pos, end_pos, light_on):

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


# Modify this function to generate trajectories
# Frequency is 50Hz (i.e. each frame costs 0.02s)
joint_0_list = []
joint_1_list = []
joint_2_list = []
light_list = []
def generate_trajectory():

    # prepare step
    move_to(joint_space_2_task_space(angles), [-10, 5, 13], 0)
    #
    # move_to([-10, 5, 13], [-10, 10, 13], 1)
    # move_to([-10, 10, 13], [-10, 5, 13], 0)
    #
    # move_to([-10, 5, 13], [10, 5, 13], 1)
    #
    # move_to([10, 5, 13], [10, 10, 13], 1)
    # move_to([10, 10, 13], [10, 5, 13], 0)
    #
    # move_to([10, 5, 13], [10, 5, 27], 1)
    #
    # move_to([10, 5, 27], [10, 10, 27], 1)
    # move_to([10, 10, 27], [10, 5, 27], 0)
    #
    # move_to([10, 5, 27], [-10, 5, 27], 1)
    #
    # move_to([-10, 5, 27], [-10, 5, 13], 1)
    # move_to([-10, 5, 13], [-10, 5, 27], 0)
    #
    # move_to([-10, 5, 27], [-10, 10, 27], 1)
    #
    # move_to([-10, 10, 27], [-10, 10, 13], 1)
    # move_to([-10, 10, 13], [10, 10, 13], 1)
    # move_to([10, 10, 13], [10, 10, 27], 1)
    # move_to([10, 10, 27], [-10, 10, 27], 1)



    #
    # # check from here
    # # Example Code
    # # move from (0, 10, 0) to (10, 20, 10) in 1 second by linear
    # # from 0s to 1s, 1(second) times 50(frequency) is number of total frame,
    # # then divided by 50 for converting it back to second
    # duration = 1
    # for t in [i / 100 for i in range(duration * 100)]:
    #     # init lighting flag
    #     light = 0
    #
    #     # ToDo
    #     # update coords
    #     x = 10 - 60 * t * t + 40 * t * t * t
    #     y = 5
    #     z = 15
    #
    #     # convert task space to joint space
    #     angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)
    #
    #     # ToDo
    #     # update light,
    #     if 0.25 <= t <= 0.5:
    #         #R, G, B
    #         # 1 for full power, 0 for no power
    #         light = [0.5, 0, 0]
    #     else:
    #         light = [0, 0, 1]
    #
    #     # push all frame to the frame list
    #     joint_0_list.append(angle0)
    #     joint_1_list.append(angle1)
    #     joint_2_list.append(angle2)
    #     light_list.append(light)
    #
    # # part 2
    # duration = 1
    # for t in [i / 100 for i in range(duration * 100)]:
    #     # init lighting flag
    #     light = 0
    #
    #     # ToDo
    #     # update coords
    #     x = -10
    #     y = 5
    #     z = 15 + 30 * t * t - 20 * t * t * t
    #
    #     # convert task space to joint space
    #     angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)
    #
    #     # ToDo
    #     # update light,
    #     if 0.25 <= t <= 0.5 or 0.75 <= t <= 1:
    #         light = [0.5, 0, 0]
    #     else:
    #         light = [0, 0, 1]
    #
    #     # push all frame to the frame list
    #     joint_0_list.append(angle0)
    #     joint_1_list.append(angle1)
    #     joint_2_list.append(angle2)
    #     light_list.append(light)
    #     # end
    #
    # # part 3
    # duration = 1
    # for t in [i / 100 for i in range(duration * 100)]:
    #     # init lighting flag
    #     light = 0
    #
    #     # ToDo
    #     # update coords
    #     x = -10 + 60 * t * t - 40 * t * t * t
    #     y = 5
    #     z = 25
    #
    #     # convert task space to joint space
    #     angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)
    #
    #     # ToDo
    #     # update light,
    #     if 0.25 <= t <= 0.5 or 0.75 <= t <= 1:
    #         light = [0.5, 0, 0]
    #     else:
    #         light = [0, 0, 1]
    #
    #     # push all frame to the frame list
    #     joint_0_list.append(angle0)
    #     joint_1_list.append(angle1)
    #     joint_2_list.append(angle2)
    #     light_list.append(light)
    #     # end
    #
    #
    # # part 4
    # duration = 1
    # for t in [i / 50 for i in range(duration * 50)]:
    #     # init lighting flag
    #     light = 0
    #
    #     # ToDo
    #     # update coords
    #     x = 10
    #     y = 5
    #     z = 25 - 30 * t * t +  20 * t * t * t
    #
    #     # convert task space to joint space
    #     angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)
    #
    #     # ToDo
    #     # update light,
    #     if 0.25 <= t <= 0.5 or 0.75 <= t <= 1:
    #         light = [0.5, 0, 0]
    #     else:
    #         light = [0, 0, 1]
    #
    #     # push all frame to the frame list
    #     joint_0_list.append(angle0)
    #     joint_1_list.append(angle1)
    #     joint_2_list.append(angle2)
    #     light_list.append(light)
    #     # end



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
set_color([0, 0, 0])



# result = "angle;"
# trajs = generate_trajectory()
# for traj in trajs:
#     result += ",".join(traj)
#     result += ";"

# f = open("traj.txt", mode="w")
# f.write(result)
# f.close()


