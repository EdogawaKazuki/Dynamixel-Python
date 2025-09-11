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

    def connect(self, device_name, baud_rate, ids):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)

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
        return self.portHandler, self.packetHandler

    def disconnect(self):
        self.connected = False
        self.portHandler.closePort()

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
        for index in range(len(ids)):
            scs_addparam_result = self.packetHandler.SyncWritePosEx(ids[index], int(angles[index] / 90 * 1024), 4096*2, 100)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed with a result of %s" % (ids[index], scs_addparam_result))
        scs_comm_result =self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            return False
        self.packetHandler.groupSyncWrite.clearParam()
        return True

    def get_joint_angle_group(self, ids):
        for id in ids:
            scs_addparam_result = self.groupSyncRead.addParam(id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed with a result of %s" % (id, scs_addparam_result))
        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            return False
        result = []
        index = 0
        for id in ids:
            scs_data_result, scs_error = self.groupSyncRead.isAvailable(id, SMS_STS_PRESENT_POSITION_L, 4)
            if scs_data_result == True:
                scs_present_position = self.groupSyncRead.getData(id, SMS_STS_PRESENT_POSITION_L, 4)
                angle = scs_present_position / 1024 * 90
                angle = angle % 360
                result.append(angle)
            else:
                self.print_error(id, COMM_TX_FAIL, scs_error)
            index += 1
        self.groupSyncRead.clearParam()
        self.joint_angles = result
        return result

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
            print("Message[ID: %d]: %s" % (joint_id, self.packetHandler.getTxRxResult(scs_comm_result)))
        if scs_error != 0:
            print("Error[ID:%d]: %s" % (joint_id, self.packetHandler.getRxPacketError(scs_error)))

    def set_servo_id(self, old_id, new_id):
        self.packetHandler.set_servo_id(old_id, new_id)