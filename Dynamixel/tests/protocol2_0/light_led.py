# -*- coding：utf-8 -*-
# 98782
# 2022年04月24日

import os
import time
from controlFunc import *

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
            termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
        return ch

# Set the DXL Model
DXL_MODEL = dxlModelSet('X')
PROTOCOL_VERSION = 2.0
# Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME = 'COM3'
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
BAUDRATE = 2000000
DXL_ID = 11

TORQUE_ENABLE               = 1     # Value for enabling the torque         力矩使能
TORQUE_DISABLE              = 0     # Value for disabling the torque        力矩失能
LED_ENABLE = 1
LED_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Control table address     # 控制表地址
if DXL_MODEL == 'X_SERIES' or DXL_MODEL == 'MX_SERIES':  # 具体地址位与数值可以参考产品 eManual 电子手册
    ADDR_TORQUE_ENABLE = 64  # 电机使能位地址
    ADDR_LED_ENABLE = 65
    ADDR_GOAL_POSITION = 116  # 电机目标位置位地址
    ADDR_PRESENT_POSITION = 132  # 电机当前位置位地址
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual    位置下限
    DXL_MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual    位置上限
    BAUDRATE = 2000000
elif DXL_MODEL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE = 562  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION = 596
    ADDR_PRESENT_POSITION = 611
    DXL_MINIMUM_POSITION_VALUE = -150000  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 150000  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600
elif DXL_MODEL == 'P_SERIES' or DXL_MODEL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE = 512  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION = 564
    ADDR_PRESENT_POSITION = 580
    DXL_MINIMUM_POSITION_VALUE = -150000  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 150000  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600
elif DXL_MODEL == 'XL320':
    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_PRESENT_POSITION = 37
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 1023  # Refer to the CCW Angle Limit of product eManual
    BAUDRATE = 1000000  # Default Baudrate of XL-320 is 1Mbps

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

DXL_ID = getDxlID(portHandler, packetHandler)
enableTorque(portHandler, packetHandler, DXL_ID, ADDR_TORQUE_ENABLE,TORQUE_ENABLE)

while True:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_ENABLE, LED_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  # 连接失败时，返回错误代码
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))  # 双重判断？
    time.sleep(0.5)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_ENABLE, LED_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  # 连接失败时，返回错误代码
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))  # 双重判断？
    time.sleep(0.5)

portHandler.closePort()

