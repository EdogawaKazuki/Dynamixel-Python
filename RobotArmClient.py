#!/usr/bin/env python
# -*- coding: utf-8 -*-
import ctypes
import os

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *


class RobotArmClient:
    def __init__(self):
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual
        self.LEN_GOAL_POSITION = 4
        self.LEN_PRESENT_POSITION = 4
        self.LEN_TORQUE_ENABLE = 1

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

        self.portHandler = None
        self.packetHandler = None
        self.connected = False

    def connect(self, device_name, protocol_version, baud_rate):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        self.connected = True
        return self.portHandler, self.packetHandler

    def disconnect(self):
        self.connected = False
        self.portHandler.closePort()

    def lock_servo(self, joint_id):

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint_id,
                                                                       self.ADDR_TORQUE_ENABLE,
                                                                       self.TORQUE_ENABLE)

        if dxl_error == 0 and dxl_comm_result == COMM_SUCCESS:
            print(f"Dynamixel Servo {joint_id} init succeed.")
            return True
        else:
            self.print_error(joint_id, dxl_comm_result, dxl_error)
            print(f"Dynamixel Servo {joint_id} init failed.")
            return False

    def release_servo(self, joint_id):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint_id,
                                                                       self.ADDR_TORQUE_ENABLE,
                                                                       self.TORQUE_DISABLE)

        if dxl_error == 0 and dxl_comm_result == COMM_SUCCESS:
            print("Motor " + str(joint_id) + " has been successfully released")
            return True
        else:
            self.print_error(joint_id, dxl_comm_result, dxl_error)
            return False

    def set_joint_angle(self, joint_id, angle):
        target_position = int(angle / 90 * 1024)
        # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, joint_id,
                                                                       self.ADDR_GOAL_POSITION,
                                                                       target_position)
        if dxl_error != 0 or dxl_comm_result != COMM_SUCCESS:
            self.print_error(joint_id, dxl_comm_result, dxl_error)
            return False
        return True
        # else:
        #     while 1:
        #         dxl_present_position, dxl_comm_result, dxl_error = \
        #             self.packetHandler.read4ByteTxRx(self.portHandler,
        #                                              joint_id,
        #                                              self.ADDR_PRESENT_POSITION)
        #
        #         if dxl_error == 0 and dxl_comm_result == COMM_SUCCESS:
        #             if dxl_present_position > 40960:
        #                 dxl_present_position = ctypes.c_int32(dxl_present_position).value
        #             if not abs(target_position - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
        #                 return True
        #         else:
        #             self.print_error(joint_id, dxl_comm_result, dxl_error)
        #             return False

    def get_joint_angle(self, joint_id):
        dxl_present_position, dxl_comm_result, dxl_error = \
            self.packetHandler.read4ByteTxRx(self.portHandler,
                                             joint_id,
                                             self.ADDR_PRESENT_POSITION)
        self.print_error(joint_id, dxl_comm_result, dxl_error)
        if dxl_present_position > 40960:
            dxl_present_position = ctypes.c_int32(dxl_present_position).value
        return dxl_present_position / 1024 * 90, dxl_present_position

    def set_joint_angle_group(self, ids, angles):
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        # groupSyncWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        for index in range(len(ids)):
            target_position = int(angles[index] / 90 * 1024)
            groupSyncWrite.addParam(ids[index], int.to_bytes(target_position, 4, 'little', signed=True))
            # groupSyncWrite.addParam(ids[index], self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, int.to_bytes(target_position, 4, 'little', signed=True))
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("Error writing cmd")

    def get_joint_angle_group(self, ids):
        groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        for index in range(len(ids)):
            groupBulkRead.addParam(ids[index], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        dxl_comm_result = groupBulkRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            return -1
        result = [[0, 0] for i in range(len(ids))]
        for index in range(len(ids)):
            dxl_present_position = groupBulkRead.getData(ids[index], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            if dxl_present_position > 40960:
                dxl_present_position = ctypes.c_int32(dxl_present_position).value
            result[index] = [dxl_present_position / 1024 * 90, dxl_present_position]
        return result

    def release_joint_group(self, ids):
        groupSyncWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        for index in range(len(ids)):
            groupSyncWrite.addParam(ids[index], self.ADDR_TORQUE_ENABLE, self.LEN_TORQUE_ENABLE, b'\x00')
        groupSyncWrite.txPacket()

    def lock_joint_group(self, ids):
        groupSyncWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        for index in range(len(ids)):
            groupSyncWrite.addParam(ids[index], self.ADDR_TORQUE_ENABLE, self.LEN_TORQUE_ENABLE, b'\x01')
        groupSyncWrite.txPacket()

    def print_error(self, joint_id, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("Message[ID: %d]: %s" % (joint_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
        if dxl_error != 0:
            print("Error[ID:%d]: %s" % (joint_id, self.packetHandler.getRxPacketError(dxl_error)))

