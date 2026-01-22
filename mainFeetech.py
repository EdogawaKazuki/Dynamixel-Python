import platform
import serial.tools.list_ports
import threading
import traceback

import FeetechArmClient
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

def wrap_angle(angle):
    return (angle + 180) % 360 - 180

def sender():
    global shut_down
    print("sender")
    while not shut_down:
        time.sleep(0.02)
        if not is_connected or client_addr is None or shut_down or is_locked:
            break
        send_data = []
        send_data += int_to_bytes(len(SERVO_ID_LIST))
        send_data += int_to_bytes(1) # has PWM
        tmp = arm.get_joint_angle_group(SERVO_ID_LIST)
        # print(tmp)
        if not tmp:
            continue
        for i in range(len(SERVO_ID_LIST)):
            servo_angle_list[i] = tmp[i] * SCALE_FACTOR[i] + OFFSET[i]
            servo_pwm_list[i] = int(tmp[i] * 1024 / 90)
            # print(i, servo_pwm_list[i])
        for angle in servo_angle_list:
            send_data += float_to_bytes(angle)
        for pwm in servo_pwm_list:
            send_data += int_to_bytes(pwm)
        print("Current servo angle list: ", tmp)
        # print(send_data)
        udp_socket.sendto(bytes(send_data), client_addr)
    print("sender stopped")

def receiver():
    global udp_socket
    global is_connected
    global client_addr
    global is_locked
    global servo_angle_list
    global tmp_servo_angle_list
    global shut_down
    send_thread = None
    angle_cmd_frame = 0
    timer = time.time()
    
    # Set socket timeout once at the start so we can check shut_down periodically
    if udp_socket:
        udp_socket.settimeout(0.5)
    
    while not shut_down:
        try:
            if udp_socket is None:
                break
            data, addr = udp_socket.recvfrom(1024)  # Adjust buffer size as needed
            cmd_type = data[0]
            if cmd_type == 0:
                print("Connect Message: ")
                cmd_data = data[1]
                if cmd_data == 1:
                    if is_connected:
                        print("Already connected")
                        print("Restarting sender")
                        is_connected = False
                        if send_thread is not None:
                            send_thread.join()
                    print("Connect")
                    is_connected = True
                    client_addr = addr
                    tmp = arm.get_joint_angle_group(SERVO_ID_LIST)
                    print(f"get joint angle result: {tmp}")
                    if not tmp:
                        print("get joint angle failed")
                        arm.reconnect()
                        continue
                    print("get joint angle success")
                    for i in range(len(servo_angle_list)):
                        tmp_servo_angle_list[i] = tmp[i]
                        servo_angle_list[i] = tmp_servo_angle_list[i] * SCALE_FACTOR[i] + OFFSET[i]
                    send_thread = threading.Thread(target=sender)
                    send_thread.start()
                    print("connect success, sender started`")
                else:
                    print("Disconnect")
                    is_connected = False
                    if send_thread is not None:
                        send_thread.join()
                        arm.release_joint_group(SERVO_ID_LIST)
            elif cmd_type == 1:
                print("Unlock Message: ")
                cmd_data = data[1]
                if cmd_data == 0:
                    print("Unlock")
                    # if send_thread is not None:
                    #     send_thread.join()
                    time.sleep(1)
                    arm.release_joint_group(SERVO_ID_LIST)
                    is_locked = False
                    time.sleep(1)
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                        # arm.release_servo(arm_id)
                else:
                    print("Lock")
                    # send_thread = threading.Thread(target=sender)
                    # send_thread.start()
                    time.sleep(1)
                    arm.lock_joint_group(SERVO_ID_LIST)
                    is_locked = True
                    time.sleep(1)
                    # for arm_id in DYNAMIXEL_SERVO_ID_LIST:
                    #     arm.lock_servo(arm_id)
            elif cmd_type == 2:
                # print("Angle Command")
                # print(data)
                angle_cmd_frame += 1
                tmp_angle_list = [0 for i in range(len(SERVO_ID_LIST))]
                cmd_angle_list = [0 for i in range(len(SERVO_ID_LIST))]
                for i in range(len(SERVO_ID_LIST)):
                    # print(data[1 + i * 4: 1 + (i + 1) * 4])
                    tmp_angle_list[i] = bytes_to_float(data[1 + i * 4: 1 + (i + 1) * 4], '<')
                    # cmd_angle_list[i] = tmp_angle_list[i] * SCALE_FACTOR[i] + OFFSET[i]
                    cmd_angle_list[i] = (tmp_angle_list[i] - OFFSET[i]) / SCALE_FACTOR[i]
                print("Calibrated commanded servo angle list: ", cmd_angle_list)
                result = arm.set_joint_angle_group(SERVO_ID_LIST, cmd_angle_list)
                if not result:
                    print("set joint angle group failed")
                    arm.reconnect()
                    continue
            if time.time() - timer > 0.5:
                print("frequency", angle_cmd_frame / 0.5)
                angle_cmd_frame = 0
                timer = time.time()
        except socket.timeout:
            # Timeout is expected, just continue to check shut_down
            continue
        except OSError as e:
            # Socket closed or error
            if shut_down:
                print("Receiver: Socket closed, exiting")
                break
            else:
                print(f"Receiver: Socket error: {e}")
                time.sleep(0.1)
        except Exception as e:
            traceback.print_exc()
            if not shut_down:
                print(f"Error in receiver: {e}")
            else:
                break
    print("Receiver thread exiting")

def find_com_port():
    com_port_list = []
    if platform.system() == "Windows":
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.device.startswith("COM"):
                com_port_list.append((port.device, port.description))
                print(f"Found COM port: {port.device}, {port.description}")

    com_port = None
    if len(com_port_list) == 0:
        print("No COM port found")
        exit(-1)
    for tmp_com_port in com_port_list:
        if "CH343" in tmp_com_port[1]:
            com_port = tmp_com_port[0]
            break
    else:
        print("No CH343 COM port found")
        exit(-1)
    
    user_input = input(f"Enter the last digit of the COM port (Default:{com_port}): ")
    if user_input != "":
        com_port = com_port[:-1] + user_input
        print(f"Using COM port: {com_port}")
    else:
        print(f"Using default COM port: {com_port}")
    return com_port

if __name__ == "__main__":

    shut_down = False

    SERVO_BAUD_RATE = 1000000
    SERVO_ID_LIST = [1, 2, 3, 4, 5, 6]
    SCALE_FACTOR = [-1, -1, -1, -1, -1, -1]
    OFFSET = [180, 180, 180, 180, 180, 180]

    SERVER_HOST = "0.0.0.0"
    SERVER_PORT = 12345

    # init robot arm
    arm = FeetechArmClient.FeetechArmClient()
    com_port = find_com_port()
    arm.connect(com_port, SERVO_BAUD_RATE, SERVO_ID_LIST)

    print("Servo connected!")

    # check servo
    locked_servo_id = []
    is_locked = False
    for arm_id in SERVO_ID_LIST:
        if arm.lock_servo(arm_id):
            locked_servo_id.append(arm_id)
    if len(locked_servo_id) != len(SERVO_ID_LIST):
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
    
    servo_angle_list = [0.0 for i in range(len(SERVO_ID_LIST))]
    servo_pwm_list = [0 for i in range(len(SERVO_ID_LIST))]
    tmp_servo_angle_list = [0.0 for i in range(len(SERVO_ID_LIST))]
    tmp_servo_pwm_list = [0 for i in range(len(SERVO_ID_LIST))]
    
    # Start receiver thread as daemon so it dies when main exits
    receiver_thread = threading.Thread(target=receiver, daemon=True)
    receiver_thread.start()
    
    try:
        # Poll instead of blocking join - this allows Ctrl+C to work
        while receiver_thread.is_alive() and not shut_down:
            receiver_thread.join(timeout=0.1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt - Shutting down...")
        shut_down = True
        is_connected = False
        if udp_socket:
            try:
                udp_socket.close()
            except Exception:
                pass
        # Give thread a moment to exit
        receiver_thread.join(timeout=1.0)
        if receiver_thread.is_alive():
            print("Warning: Receiver thread did not exit cleanly")
    finally:
        # Ensure cleanup happens
        shut_down = True
        is_connected = False
        if udp_socket:
            try:
                udp_socket.close()
            except Exception:
                pass

    time.sleep(1)
    for arm_id in SERVO_ID_LIST:
        arm.release_servo(arm_id)
        time.sleep(0.1)

    time.sleep(2)
    arm.disconnect()
