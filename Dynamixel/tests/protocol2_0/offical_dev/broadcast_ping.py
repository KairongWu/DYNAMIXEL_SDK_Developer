#!/usr/bin/env python
# -*- coding: utf-8 -*-     # 编码设定

#*******************************************************************************
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
#*******************************************************************************


#*******************************************************************************
#***********************     Broadcast Ping Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os       # 加载系统库
# os.name  返回当前操作系统的类型  posix/linux , nt/windows, jave/java 虚拟机
# sys.platform 返回当前系统的平台标识  window/win32, linux/linux, Windows Cygwin/cygwin, Mac OS X/darwin
if os.name == 'nt':
    import msvcrt
    print(f"Now os name is {os.name},import msvcrt")
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

from dynamixel_sdk import *                 # Uses Dynamixel SDK library    加载 SDK

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/    通信协议
PROTOCOL_VERSION            = 2.0         # 设定通信协议为 2.0

# Define the proper baudrate to search DYNAMIXELs. Note that XL320's baudrate is 1 M bps.
BAUDRATE                = 2000000   # 设定波特率

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM3'    # 设置串口号

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)   # 实例化串口句柄

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)     # 实例化数据包句柄

# Open port
if portHandler.openPort():      # 句柄成员方法，打开串口
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):   # 句柄成员方法，设置波特率
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Try to broadcast ping the Dynamixel
dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)   # dxl_comm_result 返回连接状态，详见 robotis_def.py 中的定义
print(dxl_data_list)
print(dxl_comm_result)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

print("Detected Dynamixel :")
for dxl_id in dxl_data_list:    # 获取设备 ID
    print("[ID:%03d] model version : %d | firmware version : %d" % (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))

# Close port
portHandler.closePort()
