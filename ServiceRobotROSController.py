import roslibpy
import keyboard
import time
import DynamixelArmClient

def main():
    # Connect to ROS
    client = roslibpy.Ros(host='192.168.37.203', port=9090)
    client.run()

    # Create a publisher for joystick messages
    joy_publisher = roslibpy.Topic(client, '/joy', 'sensor_msgs/Joy')

    # Initialize button and axis values
    joy_msg = {
        'buttons': [0] * 11,  # Assuming a joystick with 11 buttons
        'axes': [0.0] * 8     # Assuming a joystick with 8 axes
    }

    DYNAMIXEL_ADDR = "COM9"
    DYNAMIXEL_PROTOCOL = 2.0
    DYNAMIXEL_BAUD_RATE = 57600
    DYNAMIXEL_SERVO_ID_LIST = [14, 15]

    SERVER_HOST = "0.0.0.0"
    SERVER_PORT = 1234

    # init robot arm
    arm = DynamixelArmClient.DynamixelArmClient()
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
    # for arm_id in locked_servo_id:
    #     arm.release_servo(arm_id)

    try:
        while True:
            angles = [x[0] for x in arm.get_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST)]
            # angles =
            # print(angles)
            # Reset joystick inputs
            joy_msg['buttons'] = [0] * 11
            joy_msg['axes'] = [0.0] * 8

            # Check for keyboard inputs
            if keyboard.is_pressed('w'):
                joy_msg['axes'][1] = 0.5  # Forward
            elif keyboard.is_pressed('s'):
                joy_msg['axes'][1] = -0.5  # Backward

            if keyboard.is_pressed('a'):
                joy_msg['axes'][0] = 1.0  # Left
            elif keyboard.is_pressed('d'):
                joy_msg['axes'][0] = -1.0   # Right

            if keyboard.is_pressed('j'):
                angles[1] += 2
                arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, angles)
            elif keyboard.is_pressed('l'):
                angles[1] -= 2
                arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, angles)
            if keyboard.is_pressed('i'):
                angles[0] += 2
                arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, angles)
            elif keyboard.is_pressed('k'):
                angles[0] -= 2
                arm.set_joint_angle_group(DYNAMIXEL_SERVO_ID_LIST, angles)

            # if keyboard.is_pressed('space'):
            #     joy_msg['buttons'][4] = 1   # Button 0 pressed
            joy_msg['buttons'][4] = 1
            # Publish the message
            joy_publisher.publish(joy_msg)
            time.sleep(0.1)  # Adjust the loop rate

    except KeyboardInterrupt:
        print("Publisher stopped.")

    finally:
        joy_publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    main()