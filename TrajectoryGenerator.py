import math
import time
import DynamixelArmClient
import serial

board = serial.Serial('COM9', 9600)
board.setDTR(False)
time.sleep(0.01)
board.setDTR(True)
time.sleep(0.01)
board.reset_input_buffer()
board.reset_output_buffer()
# board.setRTS(False) # Drop DTR
# time.sleep(0.022)    # Read some backwhere that 22ms is what the UI does.
# board.setRTS(True)  # UP the DTR
DYNAMIXEL_ADDR = "COM8"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [11, 12, 13]


# init robot arm
arm = DynamixelArmClient.DynamixelArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)
time.sleep(0.5)
tmp = arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)
print(tmp)
angles = [tmp[0][0] - 180, 180 - tmp[1][0], tmp[2][0] - 180]

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

def set_color(index, r, g, b):
    if r > 255:
        r = 255
    if g > 255:
        g = 255
    if b > 255:
        b = 255
    if r < 0:
        r = 0
    if g < 0:
        g = 0
    if b < 0:
        b = 0
    r = r / 25
    g = g / 25
    b = b / 25
    rgb_command = f"{index},{r},{g},{b};"
    print(f"Sending rgb: {rgb_command.strip()}")
    board.write(bytes(rgb_command, 'utf-8'))
    board.flush()

for i in range(3):
    set_color(i,0,0,0)

# This function converts task space to joint space
# Input: Task space coord. For example TaskSpace2JointSpace(10, 20, 10)
# Output: A list of joint angles. For example [-26.56505117707799, 43.33991400145487, -65.29185017692065]
l1 = 7.7
l2 = 12.7
l3 = 2.5
l4 = 11.6
def IK(x, y, z):
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

    tmp = (A * A + B * B - (l23 * l23 + l4 * l4)) / (2 * l23 * l4)
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

def FK(joints):
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
        # angle0, angle1, angle2 = IK(x, y, z)
        angle0 = x
        angle1 = y
        angle2 = z

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
    move_to(FK(angles), [-10, 5, 13], 0)
    move_to([-10, 5, 13], [10, 5, 13], 0)
    move_to([10, 5, 13], [10, 5, 18], 0)
    move_to([10, 5, 18], [-10, 5, 18], 0)
    move_to([-10, 5, 18], [-10, 5, 13], 0)
    move_to([-10, 5, 13], [10, 5, 13], 0)
    move_to([10, 5, 13], [10, 5, 18], 0)
    move_to([10, 5, 18], [-10, 5, 18], 0)
    move_to([-10, 5, 18], [-10, 5, 13], 0)
    move_to([-10, 5, 13], [10, 5, 13], 0)
    return [joint_0_list, joint_1_list, joint_2_list,]


def read_traj(filename):
    with open(filename, 'r') as file:
        content = file.read()
        content = content.split(';')[1:-1]
        for i in range(len(content)):
            content[i] = content[i].split(',')
            for j in range(len(content[i])):
                content[i][j] = float(content[i][j])
        return content


traj_file_path = "trajectory_joint_space_with_color.txt"

trajs = read_traj(traj_file_path)
joint_trajs = trajs[:3]
color_trajs = trajs[3:]


angles = [tmp[0][0] - 180, 180 - tmp[1][0], tmp[2][0] - 180]
# angles = [tmp[0][0], tmp[1][0], tmp[2][0]]
# trajs = generate_trajectory()
print("Moving to start Position")
# arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, angles)
time.sleep(2)
print("Go")
move_to(angles, [joint_trajs[0][0], joint_trajs[1][0], joint_trajs[2][0]], 0)
for i in range(len(joint_0_list)):
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3],
                              [180 + joint_0_list[i], 180 - joint_1_list[i], 180 + joint_2_list[i]])
    time.sleep(0.02)

for i in range(len(joint_trajs[0])):
    arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3],
                              [180 + joint_trajs[0][i], 180 - joint_trajs[1][i], 180 + joint_trajs[2][i]])
    set_color(1, int(color_trajs[0][i]), int(color_trajs[1][i]), int(color_trajs[2][i]))
    # print(i)
    time.sleep(0.02)

set_color(1, 0, 0, 0)
board.close()



# result = "angle;"
# trajs = generate_trajectory()
# for traj in trajs:
#     result += ",".join(traj)
#     result += ";"

# f = open("traj.txt", mode="w")
# f.write(result)
# f.close()


