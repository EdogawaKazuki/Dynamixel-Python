#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time


from scservo_sdk import *


class FeetechArmClient:
    def __init__(self):
        self.MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
        self.MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual
        self.LEN_GOAL_POSITION = 4
        self.LEN_PRESENT_POSITION = 4
        self.LEN_TORQUE_ENABLE = 1

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

        self.portHandler = None
        self.packetHandler = None
        self.connected = False
        self.groupSyncRead = None
        self.joint_angles = [0 for _ in range(6)]

        self.timer = time.time()
        self.send_cmd_count = 0

        self.joint_angle_buffer = [0 for _ in range(6)]
        self.device_name = None
        self.baud_rate = None
        self.ids = None

    def connect(self, device_name, baud_rate, ids):
        print("Connecting...")
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)
        self.device_name = device_name
        self.baud_rate = baud_rate
        self.ids = ids
        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = sms_sts(self.portHandler)


        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            # getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            # getch()
            quit()
        self.connected = True
        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
        self.joint_angles = self.get_joint_angle_group(ids)
        for joint_id in ids[:1]:
            self.packetHandler.set_position_P_gain(joint_id, 16)
            self.packetHandler.set_position_I_gain(joint_id, 16)
        return self.portHandler, self.packetHandler

    def disconnect(self):
        print("Disconnecting...")
        self.connected = False
        if self.portHandler and self.portHandler.is_open:
            try:
                self.portHandler.closePort()
            except Exception as e:
                print(f"Error closing port: {e}")
        self.portHandler = None
        self.packetHandler = None
        self.groupSyncRead = None

    def reconnect(self):
        print("Reconnecting...")
        # Disconnect and wait for OS to release the port
        self.disconnect()
        time.sleep(0.5)  # Give OS time to release the port
        
        # Recreate PortHandler (don't reuse the old one)
        try:
            self.connect(self.device_name, self.baud_rate, self.ids)
            print("Reconnection successful")
            return self.portHandler, self.packetHandler
        except Exception as e:
            print(f"Reconnection failed: {e}")
            # Try one more time after a longer wait
            time.sleep(1.0)
            try:
                self.connect(self.device_name, self.baud_rate, self.ids)
                print("Reconnection successful on second attempt")
                return self.portHandler, self.packetHandler
            except Exception as e2:
                print(f"Reconnection failed again: {e2}")
                raise

    def lock_servo(self, joint_id):

        # Enable Dynamixel Torque
        scs_comm_result, scs_error = self.packetHandler.torque_enable(joint_id)

        if scs_comm_result == COMM_SUCCESS:
            print(f"Feetech Servo {joint_id} torque enabled.")
            return True
        else:
            self.print_error(joint_id, scs_comm_result, scs_error)
            print(f"Feetech Servo {joint_id} torque enable failed.")
            return False

    def release_servo(self, joint_id):
        # Disable Dynamixel Torque
        scs_comm_result, scs_error = self.packetHandler.torque_disable(joint_id)

        if scs_comm_result == COMM_SUCCESS:
            print(f"Feetech Servo {joint_id} torque disabled.")
            return True
        else:
            self.print_error(joint_id, scs_comm_result, scs_error)
            return False
        
    def set_joint_angle(self, joint_id, angle):
        target_position = int(angle / 90 * 1024)
        # Write goal position
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(joint_id, target_position, 0, 0)
        if scs_error != 0 or scs_comm_result != COMM_SUCCESS:
            self.print_error(joint_id, scs_comm_result, scs_error)
            return False
        return True

    def get_joint_angle(self, joint_id):
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(joint_id)
        if scs_error != 0 or scs_comm_result != COMM_SUCCESS:
            self.print_error(joint_id, scs_comm_result, scs_error)
            return False
        return scs_present_position / 1024 * 90, scs_present_position

    def set_joint_angle_group(self, ids, angles):
        try:
            self.send_cmd_count += 1
            if time.time() - self.timer > 0.5:
                print(f"Frequency: {self.send_cmd_count / 0.5}")
                self.send_cmd_count = 0
                self.timer = time.time()
            for index, joint_id in enumerate(ids):
                scs_addparam_result = self.packetHandler.SyncWritePosEx(joint_id, int(angles[index] / 90 * 1024), 1024, 254)
                if scs_addparam_result is not True:
                    print(f"[ID:{joint_id}] groupSyncWrite addparam failed with a result of {scs_addparam_result}")
            scs_comm_result =self.packetHandler.groupSyncWrite.txPacket()
            if scs_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(scs_comm_result))
                return False
            self.packetHandler.groupSyncWrite.clearParam()
            # angles = self.get_joint_angle_group(ids)
            # print(f"Angles: {angles}")
            return True
        except Exception as e:
            print(f"Error in set_joint_angle_group: {e}")
            return False

    def get_joint_angle_group(self, ids):
        try:
            for index, joint_id in enumerate(ids):
                scs_addparam_result = self.groupSyncRead.addParam(joint_id)
                if scs_addparam_result is not True:
                    print(f"[ID:{joint_id}] groupSyncRead addparam failed with a result of {scs_addparam_result}")
                    return False
            scs_comm_result = self.groupSyncRead.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(scs_comm_result))
                return False
            result = []
            index = 0
            for joint_id in ids:
                scs_data_result, scs_error = self.groupSyncRead.isAvailable(joint_id, SMS_STS_PRESENT_POSITION_L, 4)
                if scs_data_result is True:
                    scs_present_position = self.groupSyncRead.getData(joint_id, SMS_STS_PRESENT_POSITION_L, 4)
                    angle = scs_present_position / 1024 * 90
                    angle = angle % 360
                    result.append(angle)
                else:
                    self.print_error(joint_id, COMM_TX_FAIL, scs_error)
                index += 1
            self.groupSyncRead.clearParam()
            self.joint_angles = result
            return result
        except Exception as e:
            print(f"Error in get_joint_angle_group: {e}")
            return False

    def release_joint_group(self, ids):
        for joint_id in ids:
            scs_comm_result, scs_error = self.packetHandler.torque_disable(joint_id)
            if scs_error != 0 or scs_comm_result != COMM_SUCCESS:
                self.print_error(joint_id, scs_comm_result, scs_error)
                return False
        return True

    def lock_joint_group(self, ids):
        for joint_id in ids:
            scs_comm_result, scs_error = self.packetHandler.torque_enable(joint_id)
            if scs_error != 0 or scs_comm_result != COMM_SUCCESS:
                self.print_error(joint_id, scs_comm_result, scs_error)
                return False
            time.sleep(0.1)
        return True

    def print_error(self, joint_id, scs_comm_result, scs_error):
        if scs_comm_result != COMM_SUCCESS:
            print( f"Message[ID: {joint_id}]: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        if scs_error != 0:
            print( f"Error[ID:{joint_id}]: {self.packetHandler.getRxPacketError(scs_error)}")

    def set_servo_id(self, old_id, new_id):
        self.packetHandler.set_servo_id(old_id, new_id)