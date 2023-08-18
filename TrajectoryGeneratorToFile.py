from math import *

save_to_file = False
draw_graph = False

try:
    import matplotlib.pyplot as plt
    import numpy as np
    draw_graph = True
except:
    print("Cannot import matplotlib, graph will not be drawn.")

l1 = 7.7
l2 = 12.7
l3 = 2.5
l4 = 11.6

joint_0_list = []
joint_1_list = []
joint_2_list = []
light_list = []


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


# This function converts task space to joint space
# Input: Task space coord. For example TaskSpace2JointSpace(10, 20, 10)
# Output: A list of joint angles. For example [-26.56505117707799, 43.33991400145487, -65.29185017692065]
# Modify this function to generate trajectories
# Frequency is 50Hz (i.e. each frame costs 0.02s)
def generate_trajectory():

    # check from here
    # Example Code
    # move from (0, 10, 0) to (10, 20, 10) in 1 second
    # from 0s to 1s, 1(second) times 50(frequency) is number of total frame,
    # then divided by 50 for converting it back to second

    # part 1 (0, 10, 0) to (10, 20, 10) in 1 second
    duration = 1  # in seconds
    for t in [i / 50 for i in range(duration * 50)]:

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
            # color format: R, G, B
            # 1 for full power, 0 for no power
            light = [0.5, 0, 0]  # green in half brightness
        else:
            light = [0, 0, 1]  # blue in full brightness

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
        light_list.append(light)
    # end of part 1

    # part 2
    duration = 1
    for t in [i / 50 for i in range(duration * 50)]:

        # ToDo
        # update coords
        x = -sin(t)
        y = 5
        z = 15 + 30 * t * t - 20 * t * t * t

        # convert task space to joint space
        angle0, angle1, angle2 = task_space_2_joint_space(x, y, z)

        # ToDo
        # update light,
        if 0.25 <= t <= 0.5 or 0.75 <= t <= 1:
            light = [0.5, 0, 0]
        else:
            light = [0, 0, 1]

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
        light_list.append(light)
    # end of part 2

    return [joint_0_list, joint_1_list, joint_2_list, light_list]


result = "angle;"
trajectories = generate_trajectory()
for trajectory in trajectories[:-1]:
    result += ",".join([str(x) for x in trajectory])
    result += ";"

print(result)

if save_to_file:
    f = open("trajectory.txt", mode="w")
    f.write(result)
    f.close()

if draw_graph:
    x = [i for i in range(len(trajectories[0]))]
    for i in range(len(trajectories) - 1):
        plt.plot(x, trajectories[i])
    plt.show()
