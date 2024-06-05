import threading

import numpy as np

import OpenManipulatorProROSClient
import time
import struct
import socket


def bytes_to_float(byte_array, byte_order='<'):
    # Assuming the byte array represents a 32-bit floating-point number (single precision)
    float_size = 4

    # Ensure the byte array length matches the float size
    if len(byte_array) != float_size:
        raise ValueError(f"Invalid byte array length. Expected {float_size} bytes.")

    # Convert byte array to float
    format_str = f"{byte_order}f"
    return struct.unpack(format_str, byte_array)[0]


def float_to_bytes(float_num, byte_order='<'):
    # Convert float to bytes
    # print(float_num)
    byte_array = struct.pack(f"{byte_order}f", float_num)
    return byte_array


def bytes_to_int(byte_array, byte_order='little'):
    return int.from_bytes(byte_array, byte_order)


def int_to_bytes(integer, byte_order='little', signed=True):
    # Convert integer to bytes
    byte_size = 2  # Calculate the number of bytes needed
    byte_array = integer.to_bytes(byte_size, byte_order, signed=signed)
    return byte_array


def check_joint_space(joints):
    task_space_x, task_space_y, task_space_z = forward_kinematics(joints)
    return check_task_space(task_space_x, task_space_y, task_space_z)


def check_task_space(task_space_x, task_space_y, task_space_z):
    limit_left = -20
    limit_right = 20
    limit_up = 20
    limit_down = -0
    limit_front = 20
    limit_back = -20
    if abs(task_space_x) <= limit_left or abs(task_space_x) >= limit_right or \
            abs(task_space_y) <= limit_down or abs(task_space_y) >= limit_up or \
            task_space_z <= limit_back or task_space_z >= limit_front:
        return False
    return True


def forward_kinematics(joints):
    l1 = 159
    l2 = 264
    l3 = 30
    l4 = 258
    theta1 = joints[0] * np.pi / 180
    theta2 = joints[1] * np.pi / 180
    theta3 = joints[2] * np.pi / 180 + np.pi/2
    theta4 = joints[3] * np.pi / 180
    theta5 = joints[4] * np.pi / 180
    theta6 = joints[5] * np.pi / 180
    T_01_matrix = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                            [np.sin(theta1), np.cos(theta1), 0, 0, ],
                            [0, 0, 1, l1],
                            [0, 0, 0, 1]])
    T_12_matrix = np.array([[np.cos(theta2), 0, np.sin(theta2), 0, ],
                            [0, 1, 0, 0],
                            [-np.sin(theta2), 0, np.cos(theta2), 0],
                            [0, 0, 0, 1]])
    T_23_matrix = np.array([[np.cos(theta3), 0, np.sin(theta3), l3, ],
                            [0, 1, 0, 0],
                            [-np.sin(theta3), 0, np.cos(theta3), l2],
                            [0, 0, 0, 1]])
    T_34_matrix = np.array([[np.cos(theta4), -np.sin(theta4), 0, -l3],
                            [np.sin(theta4), np.cos(theta4), 0, 0, ],
                            [0, 0, 1, l4],
                            [0, 0, 0, 1]])
    T_45_matrix = np.array([[np.cos(theta5), 0, np.sin(theta5), 0, ],
                            [0, 1, 0, 0],
                            [-np.sin(theta5), 0, np.cos(theta5), 0],
                            [0, 0, 0, 1]])
    T_56_matrix = np.array([[np.cos(theta6), -np.sin(theta6), 0, 0],
                            [np.sin(theta6), np.cos(theta6), 0, 0, ],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    T_06_matrix = np.dot(
        np.dot(np.dot(np.dot(np.dot(T_01_matrix, T_12_matrix), T_23_matrix), T_34_matrix), T_45_matrix),
        T_56_matrix)

    coordinate_6 = np.array([[0], [0], [123], [1]])
    end_effector_position = np.dot(T_06_matrix, coordinate_6)
    end_effector_position_x = end_effector_position[1]
    end_effector_position_y = end_effector_position[0]
    end_effector_position_z = end_effector_position[2]

    return end_effector_position_x, end_effector_position_y, end_effector_position_z


def sender():
    while True:
        if not is_connected or client_addr is None or shut_down:
            print("STOP Sending")
            break
        # print("Connected: " + str(is_connected))
        send_data = []
        tmp = arm.get_joint_angle_group()
        if tmp == -1:
            continue
        for i in range(len(servo_angle_list)):
            tmp_servo_angle_list[i] = tmp[i]
        servo_angle_list[0] = tmp_servo_angle_list[0]
        servo_angle_list[1] = tmp_servo_angle_list[1]
        servo_angle_list[2] = tmp_servo_angle_list[2]
        servo_angle_list[3] = tmp_servo_angle_list[3]
        servo_angle_list[4] = tmp_servo_angle_list[4]
        servo_angle_list[5] = tmp_servo_angle_list[5]
        for pwm in servo_pwm_list:
            send_data += int_to_bytes(pwm)
        for angle in servo_angle_list:
            send_data += float_to_bytes(angle)
        # print(servo_angle_list)
        # print(send_data)
        # print(len(send_data))
        udp_socket.sendto(bytes(send_data), client_addr)
        time.sleep(0.01)


def receiver():
    global udp_socket
    global is_connected
    global client_addr
    global is_locked
    global servo_angle_list
    global tmp_servo_angle_list
    send_thread = None
    angle_cmd_frame = 0
    timer = time.time()
    while True:
        if shut_down:
            break
        try:
            data, addr = udp_socket.recvfrom(1024)  # Adjust buffer size as needed
            cmd_type = data[0]
            if cmd_type == 0:
                print("Connect Message: ")
                cmd_data = data[1]
                if cmd_data == 1:
                    print("Connect")
                    is_connected = True
                    client_addr = addr
                    tmp = arm.get_joint_angle_group()
                    # print(tmp)
                    if tmp == -1:
                        continue
                    for i in range(len(servo_angle_list)):
                        tmp_servo_angle_list[i] = tmp[i]
                    servo_angle_list[0] = tmp_servo_angle_list[0]
                    servo_angle_list[1] = tmp_servo_angle_list[1]
                    servo_angle_list[2] = tmp_servo_angle_list[2]
                    servo_angle_list[3] = tmp_servo_angle_list[3]
                    servo_angle_list[4] = tmp_servo_angle_list[4]
                    servo_angle_list[5] = tmp_servo_angle_list[5]
                    send_thread = threading.Thread(target=sender)
                    send_thread.start()
                else:
                    print("Disconnect")
                    is_connected = False
                    if send_thread is not None:
                        send_thread.join()
                        arm.release_joint_group()
            elif cmd_type == 1:
                print("Unlock Message: ")
                cmd_data = data[1]
                if cmd_data == 0:
                    print("Unlock")
                    # if send_thread is not None:
                    #     send_thread.join()
                    arm.release_joint_group()
                    is_locked = False
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                    #     arm.release_servo(arm_id)
                else:
                    print("Lock")
                    # send_thread = threading.Thread(target=sender)
                    # send_thread.start()
                    is_locked = True
                    arm.lock_joint_group()
                    # time.sleep(1)
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                    #     arm.lock_servo(arm_id)
            elif cmd_type == 2:
                # print("Angle Command")
                # print(data)
                angle_cmd_frame += 1
                tmp_angle_list = [0, 0, 0, 0, 0, 0]
                cmd_angle_list = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    # print(data[1 + i * 4: 1 + (i + 1) * 4])
                    tmp_angle_list[i] = bytes_to_float(data[1 + i * 4: 1 + (i + 1) * 4], '<')
                cmd_angle_list[0] = tmp_angle_list[0]
                cmd_angle_list[1] = tmp_angle_list[1]
                cmd_angle_list[2] = tmp_angle_list[2]
                cmd_angle_list[3] = tmp_angle_list[3]
                cmd_angle_list[4] = tmp_angle_list[4]
                cmd_angle_list[5] = tmp_angle_list[5]
                # print(cmd_angle_list)
                print(forward_kinematics(cmd_angle_list))
                arm.set_joint_angle_group(OpenManipulatorProROS_SERVO_ID_LIST, cmd_angle_list, 0.01)
                # for i in range(3):
                #     print(f"Angle {i}: {cmd_angle_list[i]}", end="; ")
                #     # arm.set_joint_angle(DYNAMIXEL_SERVO_ID_LIST[i], cmd_angle_list[i])
                # print()
            # time.sleep(0.05)
            if time.time() - timer > 0.5:
                print("frequency", angle_cmd_frame / 0.5)
                angle_cmd_frame = 0
                timer = time.time()
            tmp = arm.get_joint_angle_group()
            if tmp == -1:
                continue
            for i in range(len(servo_angle_list)):
                tmp_servo_angle_list[i] = tmp[i]
            servo_angle_list[0] = tmp_servo_angle_list[0]
            servo_angle_list[1] = tmp_servo_angle_list[1]
            servo_angle_list[2] = tmp_servo_angle_list[2]
            servo_angle_list[3] = tmp_servo_angle_list[3]
            servo_angle_list[4] = tmp_servo_angle_list[4]
            servo_angle_list[5] = tmp_servo_angle_list[5]
        except Exception as e:
            print(e)
            raise e
            continue
shut_down = False

OpenManipulatorProROS_IP = "192.168.2.104"
OpenManipulatorProROS_Port = 9090
OpenManipulatorProROS_SERVO_ID_LIST = [0, 1, 2, 3, 4, 5]

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 1234

# init robot arm
arm = OpenManipulatorProROSClient.OpenManipulatorProROSClient()
arm.connect(OpenManipulatorProROS_IP, OpenManipulatorProROS_Port)

# check servo
locked_servo_id = []
is_locked = False
# for arm_id in OpenManipulatorProROS_SERVO_ID_LIST:
#     if arm.lock_servo(arm_id):
#         locked_servo_id.append(arm_id)
# if len(locked_servo_id) != len(OpenManipulatorProROS_SERVO_ID_LIST):
#     print("Please check dynamixel servo status! Exiting...")
#     arm.disconnect()
#     exit(-1)
# else:
#     print("All servo connected!")
# for arm_id in locked_servo_id:
#     arm.release_servo(arm_id)

# init socket
udp_socket = None
try:
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((SERVER_HOST, SERVER_PORT))
    print(f"UDP receiver started on {SERVER_HOST}:{SERVER_PORT}")
except Exception as e:
    print(f"UDP receiver start failed. Error: {e}")
    exit(-1)
is_connected = False
client_addr = None

receiver_thread = threading.Thread(target=receiver)
receiver_thread.start()
servo_angle_list = [0.0 for i in range(len(OpenManipulatorProROS_SERVO_ID_LIST))]
servo_pwm_list = [0 for i in range(len(OpenManipulatorProROS_SERVO_ID_LIST))]
tmp_servo_angle_list = [0.0 for i in range(len(OpenManipulatorProROS_SERVO_ID_LIST))]
tmp_servo_pwm_list = [0 for i in range(len(OpenManipulatorProROS_SERVO_ID_LIST))]
# while True:
#     try:
#         tmp = arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)
#         if tmp == -1:
#             continue
#         for i in range(len(servo_angle_list)):
#             tmp_servo_angle_list[i], tmp_servo_pwm_list[i] = tmp[i]
#         servo_angle_list[0] = tmp_servo_angle_list[0] - 180
#         servo_angle_list[1] = 270 - tmp_servo_angle_list[1]
#         servo_angle_list[2] = tmp_servo_angle_list[2] - 180
#     except KeyboardInterrupt:
#         shut_down = True
#         break
#     time.sleep(1)
# print()

receiver_thread.join()

time.sleep(1)
for arm_id in OpenManipulatorProROS_SERVO_ID_LIST:
    arm.release_servo(arm_id)
    time.sleep(0.1)

time.sleep(2)
arm.disconnect()
