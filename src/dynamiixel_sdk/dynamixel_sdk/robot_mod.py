#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

#*******************************************************************************
#***********************     Bulk Read and Bulk Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series). Note that the XL320 does not support Bulk Read and Bulk Write. 
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import time #for testing only

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    # old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from dynamixel_sdk.registerDict import *   # Uses all the register dictionary constants and functions

DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
# BAUDRATE                    = 4000000
# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
# PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
# DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
# DXL2_ID                     = 2                 # Dynamixel#1 ID : 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
# DEVICENAME                  = '/dev/ttyUSB0'

DXL_MOVING_STATUS_THRESHOLD = 20  
COMM_SUCCESS                = 0                             # Communication Success result value
index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]        # Goal position
dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write


class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''

    def __init__( self, servo_ids, DEVICENAME = '/dev/ttyUSB0', BAUDRATE = 4000000, PROTOCOL_VERSION = 2.0):   # CHANGE THIS baudrate VALUE IF YOUR MOTOR HAS DIFERNET BAUD!! 

        # reisterDict imported
        self.rDict = X_Series

        # servo ids
        self.servo_ids = servo_ids
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.servo_ids = servo_ids
        # # Initialize GroupBulkWrite instance
        # self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        # # Initialize GroupBulkRead instace for Present Position
        # self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        
        # Initialize GroupSyncWrite instance
        # self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position, Current, and Velocity
        self.PresentPositionSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.rDict["ADDR_PRESENT_POSITION"], self.rDict["LEN_PRESENT_POSITION"])
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.PresentPositionSyncRead.addParam(servo_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % 1)
                quit()

        self.PresentCurrentSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.rDict["ADDR_PRESENT_CURRENT"], self.rDict["LEN_PRESENT_CURRENT"])
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.PresentCurrentSyncRead.addParam(servo_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % 1)
                quit()

        self.PresentVelocitySyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.rDict["ADDR_PRESENT_VELOCITY"], self.rDict["LEN_PRESENT_VELOCITY"])
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.PresentVelocitySyncRead.addParam(servo_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % 1)
                quit()

        self.PresentVoltageSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.rDict["ADDR_PRESENT_INPUT_VOLTAGE"], self.rDict["LEN_PRESENT_INPUT_VOLTAGE"])
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.PresentVoltageSyncRead.addParam(servo_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % 1)
                quit()

        #Initialize SyncWrite for 
        self.GoalPositionSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.rDict["ADDR_GOAL_POSITION"], self.rDict["LEN_GOAL_POSITION"])
        # for servo_id in self.servo_ids:
        #     dxl_addparam_result = self.GoalPositionSyncWrite.addParam(servo_id)
        #     if dxl_addparam_result != True:
        #         print("[ID:%03d] groupSyncWrite addparam failed" % 1)
        #         quit()

        self.GoalCurrentSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.rDict["ADDR_GOAL_CURRENT"], self.rDict["LEN_GOAL_CURRENT"])
        # for servo_id in self.servo_ids:
        #     dxl_addparam_result = self.GoalCurrentSyncWrite.addParam(servo_id)
        #     if dxl_addparam_result != True:
        #         print("[ID:%03d] groupSyncWrite addparam failed" % 1)
        #         quit()

        self.GoalVelocitySyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.rDict["ADDR_GOAL_VELOCITY"], self.rDict["LEN_GOAL_VELOCITY"])
        # for servo_id in self.servo_ids:
        #     dxl_addparam_result = self.GoalVelocitySyncWrite.addParam(servo_id)
        #     if dxl_addparam_result != True:
        #         print("[ID:%03d] groupSyncWrite addparam failed" % 1)
        #         quit()

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def enable_torque(self,servo_ids):
        # Enable Dynamixel Torque
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_TORQUE_ENABLE"], self.rDict["TORQUE_ENABLE"])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") torque has been successfully enabled")

    def disable_torque(self,servo_ids):
        # Enable Dynamixel Torque
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_TORQUE_ENABLE"], self.rDict["TORQUE_DISABLE"])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel ( ID:",str(servo_id),") torque has been successfully disabled")

    def reboot(self,servo_ids):
        for servo_id in servo_ids:
            self.packetHandler.reboot(self.portHandler, servo_id)
            # if self.packetHandler.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            #     self.packetHandlerself.packetHandler.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
            # elif self.packetHandler.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            # self.packetHandler.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

            print("[ID:%03d] reboot Succeeded" % (servo_id))


    def set_position_P_gain(self, gain, servo_ids):
        # Enable Dynamixel Torque
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_POSITION_P_GAIN"], gain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") Position P Gain has been successfully set to", str(gain))
    
    def set_position_I_gain(self, gain, servo_ids):
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_POSITION_I_GAIN"], gain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") Position I Gain has been successfully set to", str(gain))

    def set_position_D_gain(self, gain, servo_ids):
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_POSITION_D_GAIN"], gain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") Position D Gain has been successfully set to", str(gain))


    def set_profile_velocity(self, velocity, servo_ids):
        # Enable Dynamixel Torque
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_PROFILE_VELOCITY"], velocity)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") Profile Velocity has been successfully set to", str(velocity))

    def set_profile_acc(self, acc, servo_ids):
        # Enable Dynamixel Torque
        for servo_id in servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, self.rDict["ADDR_PROFILE_ACCELERATION"], acc)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel (ID:", str(servo_id),") Profile Acceleration has been successfully set to", str(acc))



    def close_port(self):
        self.portHandler.closePort()
    
    def sync_read(self, type):
        if type == "position":
            dxl_comm_result = self.PresentPositionSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.PresentPositionSyncRead.isAvailable(self.servo_ids[0], self.rDict["ADDR_PRESENT_POSITION"], self.rDict["LEN_PRESENT_POSITION"])
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % self.servo_ids[0])
                quit()
            positions = []
            for servo_id in self.servo_ids:

                dxl_present_position = self.PresentPositionSyncRead.getData(servo_id, self.rDict["ADDR_PRESENT_POSITION"], self.rDict["LEN_PRESENT_POSITION"])
                positions.append(dxl_present_position)
                
                # print("[ID:%03d]  PresPos:%03d\t" % (servo_id, dxl1_present_position))
            return positions
        
        elif type == "current":
            dxl_comm_result = self.PresentCurrentSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            # dxl_getdata_result = self.PresentCurrentSyncRead.isAvailable(self.servo_ids[0], self.rDict["ADDR_PRESENT_CURRENT"], self.rDict["LEN_PRESENT_CURRENT"])
            # if dxl_getdata_result != True:
            #     print("[ID:%03d] groupSyncRead getdata failed" % self.servo_ids[0])
                # quit()
            currents = []

            for servo_id in self.servo_ids:

                dxl_present_current = self.PresentCurrentSyncRead.getData(servo_id, self.rDict["ADDR_PRESENT_CURRENT"], self.rDict["LEN_PRESENT_CURRENT"])
                # print(servo_id)
                # print("[ID:%03d]  PresCurrent:%03d\t" % (servo_id, dxl1_present_current))
                currents.append(dxl_present_current)
            
            return currents

        elif type == "velocity":
            dxl_comm_result = self.PresentVelocitySyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            # dxl_getdata_result = self.PresentVelocitySyncRead.isAvailable(self.servo_ids[0], self.rDict["ADDR_PRESENT_VELOCITY"], self.rDict["LEN_PRESENT_VELOCITY"])
            # if dxl_getdata_result != True:
            #     print("[ID:%03d] groupSyncRead getdata failed" % self.servo_ids[0])
                # quit()
            velocities = []
            for servo_id in self.servo_ids:

                dxl_present_velocity = self.PresentVelocitySyncRead.getData(servo_id, self.rDict["ADDR_PRESENT_VELOCITY"], self.rDict["LEN_PRESENT_VELOCITY"])
                # print(servo_id)
                # print("[ID:%03d]  PresCurrent:%03d\t" % (servo_id, dxl1_present_velocity))
                velocities.append(dxl_present_velocity)
            return velocities

        elif type == "voltage":
            dxl_comm_result = self.PresentVoltageSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            # dxl_getdata_result = self.PresentVoltageSyncRead.isAvailable(self.servo_ids[0], self.rDict["ADDR_PRESENT_INPUT_VOLTAGE"], self.rDict["LEN_PRESENT_INPUT_VOLTAGE"])
            # if dxl_getdata_result != True:
            #     print("[ID:%03d] groupSyncRead getdata failed" % self.servo_ids[0])
                # quit()
            voltages = []
            for servo_id in self.servo_ids:

                dxl_present_voltage = self.PresentVoltageSyncRead.getData(servo_id, self.rDict["ADDR_PRESENT_INPUT_VOLTAGE"], self.rDict["LEN_PRESENT_INPUT_VOLTAGE"])
                # print(servo_id)
                # print("[ID:%03d]  PresCurrent:%03d\t" % (servo_id, dxl1_present_velocity))
                voltages.append(dxl_present_voltage)
            return voltages
            
    def test_read_speed(self):
        ts_old = time.time()
        while 1:
            self.sync_read(type = "velocity",servo_ids=[1,2])
            ts_new = time.time()            
            hz = 1/(ts_new - ts_old)
            print("Tranmission speed is",str(hz), " Hz")
            ts_old = ts_new


    def sync_write(self, type, dataDict):  # this part is only for goal positions, there is no complete current or velocity control

        if type == "position":
            for i in range(len(self.servo_ids)):
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dataDict[i])), DXL_HIBYTE(DXL_LOWORD(dataDict[i])), DXL_LOBYTE(DXL_HIWORD(dataDict[i])), DXL_HIBYTE(DXL_HIWORD(dataDict[i]))]
                dxl_addparam_result = self.GoalPositionSyncWrite.addParam(self.servo_ids[i], param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % 1)
                    quit()

            # Syncwrite goal position
            dxl_comm_result = self.GoalPositionSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            self.GoalPositionSyncWrite.clearParam()    


        if type == "current":
            for i in range(len(self.servo_ids)):
                dxl_addparam_result = self.GoalCurrentSyncWrite.addParam(self.servo_ids[i], dataDict[i])
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % 1)
                    quit()

            # Syncwrite goal position
            dxl_comm_result = self.GoalCurrentSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        if type == "velocity":
            for i in range(len(self.servo_ids)):
                dxl_addparam_result = self.GoalVelocitySyncWrite.addParam(self.servo_ids[i], dataDict[i])
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % 1)
                    quit()

            # Syncwrite goal position
            dxl_comm_result = self.GoalVelocitySyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))