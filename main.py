import threading

import RobotArmClient
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


def sender():
    while True:
        if not is_connected or client_addr is None or shut_down or is_locked:
            break
        send_data = []
        for pwm in servo_pwm_list[:3]:
            send_data += int_to_bytes(pwm)
        for angle in servo_angle_list[:3]:
            send_data += float_to_bytes(angle)
        # print(servo_angle_list)
        # print(send_data)
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
                    send_thread = threading.Thread(target=sender)
                    send_thread.start()
                else:
                    print("Disconnect")
                    is_connected = False
                    if send_thread is not None:
                        send_thread.join()
                        arm.release_joint_group(DYNAMIXEL_SERVO_ID_LIST)
            elif cmd_type == 1:
                print("Unlock Message: ")
                cmd_data = data[1]
                if cmd_data == 0:
                    print("Unlock")
                    # if send_thread is not None:
                    #     send_thread.join()
                    arm.release_joint_group(DYNAMIXEL_SERVO_ID_LIST)
                    # is_locked = False
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                    #     arm.release_servo(arm_id)
                else:
                    print("Lock")
                    # send_thread = threading.Thread(target=sender)
                    # send_thread.start()
                    # is_locked = True
                    arm.lock_joint_group(DYNAMIXEL_SERVO_ID_LIST)
                    # time.sleep(1)
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                    #     arm.lock_servo(arm_id)
            elif cmd_type == 2:
                # print("Angle Command")
                # print(data)
                angle_cmd_frame += 1
                tmp_angle_list = [0, 0, 0]
                cmd_angle_list = [0, 0, 0]
                for i in range(3):
                    # print(data[1 + i * 4: 1 + (i + 1) * 4])
                    tmp_angle_list[i] = bytes_to_float(data[1 + i * 4: 1 + (i + 1) * 4], '<')
                cmd_angle_list[0] = 180 + tmp_angle_list[0]
                cmd_angle_list[1] = 270 - tmp_angle_list[1]
                cmd_angle_list[2] = 180 + tmp_angle_list[2]
                print(cmd_angle_list)
                arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST[:3], cmd_angle_list)
                # for i in range(3):
                #     print(f"Angle {i}: {cmd_angle_list[i]}", end="; ")
                #     # arm.set_joint_angle(DYNAMIXEL_SERVO_ID_LIST[i], cmd_angle_list[i])
                # print()
            # time.sleep(0.05)
            if time.time() - timer > 0.5:
                print("frequency", angle_cmd_frame / 0.5)
                angle_cmd_frame = 0
                timer = time.time()
            tmp = arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)
            if tmp == -1:
                continue
            for i in range(len(servo_angle_list)):
                tmp_servo_angle_list[i], tmp_servo_pwm_list[i] = tmp[i]
            servo_angle_list[0] = tmp_servo_angle_list[0] - 180
            servo_angle_list[1] = 270 - tmp_servo_angle_list[1]
            servo_angle_list[2] = tmp_servo_angle_list[2] - 180
        except:
            pass
shut_down = False

DYNAMIXEL_ADDR = "COM6"
DYNAMIXEL_PROTOCOL = 2.0
DYNAMIXEL_BAUD_RATE = 1000000
DYNAMIXEL_SERVO_ID_LIST = [11, 12, 14]

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 1234

# init robot arm
arm = RobotArmClient.RobotArmClient()
arm.connect(DYNAMIXEL_ADDR, DYNAMIXEL_PROTOCOL, DYNAMIXEL_BAUD_RATE)

# check servo
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
for arm_id in locked_servo_id:
    arm.release_servo(arm_id)

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
servo_angle_list = [0.0 for i in range(len(DYNAMIXEL_SERVO_ID_LIST))]
servo_pwm_list = [0 for i in range(len(DYNAMIXEL_SERVO_ID_LIST))]
tmp_servo_angle_list = [0.0 for i in range(len(DYNAMIXEL_SERVO_ID_LIST))]
tmp_servo_pwm_list = [0 for i in range(len(DYNAMIXEL_SERVO_ID_LIST))]
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
for arm_id in DYNAMIXEL_SERVO_ID_LIST:
    arm.release_servo(arm_id)
    time.sleep(0.1)

time.sleep(2)
arm.disconnect()
