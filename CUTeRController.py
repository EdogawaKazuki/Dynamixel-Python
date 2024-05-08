import threading

import DynamixelArmClient
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


def read_trajectory_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
        joint_trajectory_str = content.split(';')
        joint_trajectory = [[float(x) for x in traj.split(',') if len(x) > 0]
                            for traj in joint_trajectory_str if len(traj) > 0]
        return joint_trajectory


def move_to(init_pos, end_pos):
    print("from: ", init_pos)
    print("to: ", end_pos)
    joint_0_list = []
    joint_1_list = []
    joint_2_list = []
    current_pos = init_pos
    init_pos = end_pos
    duration = 1
    for t in [i / 50 for i in range(duration * 50)]:

        # update coords
        angle0 = current_pos[0] + (init_pos[0] - current_pos[0]) * t
        angle1 = current_pos[1] + (init_pos[1] - current_pos[1]) * t
        angle2 = current_pos[2] + (init_pos[2] - current_pos[2]) * t

        # push all frame to the frame list
        joint_0_list.append(angle0)
        joint_1_list.append(angle1)
        joint_2_list.append(angle2)
    return [joint_0_list, joint_1_list, joint_2_list]


cuter_ip = "192.168.4.1"
cuter_port = 1234
freq = 50
joint_number = 3
client_addr = (cuter_ip, cuter_port)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

trajectory = read_trajectory_file('E:\ENGG5402 Traj\short_trajectory_box3_3dof_numIK.txt')

# connect to CUTeR
print("Connecting to CUTeR...")
udp_socket.sendto(b'\x00\x01', client_addr)
print("Connected...")

# get initial position
print("Getting initial position...")
data, addr = udp_socket.recvfrom(1024)
# print(data)
init_pos = [bytes_to_float(data[2 * 3 + i * 4: 2 * 3 + (i + 1) * 4], '<') for i in range(joint_number)]
print(init_pos)

# lock joints
print("Locking joints...")
udp_socket.sendto(b'\x01\x01', client_addr)
print("Locked...")

# move to initial position
# print("Moving to initial position...")
# init_traj = move_to(init_pos, [trajectory[x][0] for x in range(joint_number)])
# for i in range(len(init_traj[0])):
#     # print(f"Sending trajectory {i}...")
#     servo_angle_list = [init_traj[x][i] for x in range(joint_number)]
#     send_data = [2]
#     for angle in servo_angle_list[:joint_number]:
#         send_data += float_to_bytes(angle)
#     # print(servo_angle_list[:3])
#     udp_socket.sendto(bytes(send_data), client_addr)
#     time.sleep(1 / freq)
#
# print("Inited. Waiting for 1 seconds...")
time.sleep(1)


# send trajectory
for i in range(len(trajectory[0])):
    # print(f"Sending trajectory {i}...")
    servo_angle_list = [trajectory[x][i] for x in range(joint_number)]
    send_data = [2]
    for angle in servo_angle_list[:joint_number]:
        send_data += float_to_bytes(angle)
    # print(servo_angle_list[:3])
    # print(send_data)
    udp_socket.sendto(bytes(send_data), client_addr)
    time.sleep(1 / freq)


# unlock joints
print("Unlocking joints...")
udp_socket.sendto(b'\x01\x00', client_addr)
print("Unlocked...")

# disconnect from CUTeR
print("Disconnecting from CUTeR...")
udp_socket.sendto(b'\x00\x00', client_addr)
print("Disconnected...")

udp_socket.close()
print("Finished. Exiting...")


