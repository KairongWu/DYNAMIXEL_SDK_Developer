# -*- coding：utf-8 -*-
# 98782
# 2022年04月22日

from dynamixel_sdk import *
import time

ADDR_TORQUE_ENABLE = 0  # 电机使能位地址
ADDR_GOAL_POSITION = 0  # 电机目标位置位地址
ADDR_PRESENT_POSITION = 0  # 电机当前位置位地址
DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual    位置下限
DXL_MAXIMUM_POSITION_VALUE = 0  # Refer to the Maximum Position Limit of product eManual    位置上限
BAUDRATE = 0

def dxlModelSet(Name):
# ********* DYNAMIXEL Model definition *********     DYNAMIXEL 舵机型号
# ***** (Use only one definition at a time) *****
# MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V
    if (Name == 'X')| (Name == 'x'):
        return  'X_SERIES'
    elif (Name == 'MX') | (Name == 'mx'):
        return 'MX_SERIES'
    elif (Name == 'PRO') | (Name == 'pro'):
        return 'PRO_SERIES'
    elif (Name == 'PA') | (Name == 'pa'):
        return 'PRO_A_SERIES'
    elif (Name == 'P') | (Name == 'p'):
        return 'P_SERIES'
    elif (Name == 'XL') | (Name == 'xl'):
        return 'XL320'

def getDxlID(portHandler, packetHandler):
    # Try to broadcast ping the Dynamixel
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(
        portHandler)  # dxl_comm_result 返回连接状态，详见 robotis_def.py 中的定义
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    print("Detected Dynamixel :")
    for dxl_id in dxl_data_list:  # 获取设备 ID
        print("[ID:%03d] model version : %d | firmware version : %d" % (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))
    return dxl_id

def setControlTable(DXL_MODEL):
    # Control table address     # 控制表地址
    if DXL_MODEL == 'X_SERIES' or DXL_MODEL == 'MX_SERIES':  # 具体地址位与数值可以参考产品 eManual 电子手册
        ADDR_TORQUE_ENABLE = 64  # 电机使能位地址
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

def enableTorque(portHandler, packetHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE):
    # Enable Dynamixel Torque   使能舵机力矩
    # 按 1 字节写入，更改舵机状态
    # 输入参数 port, dxl_id, address, data
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  # 连接失败时，返回错误代码
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))  # 双重判断？
    else:
        print("Dynamixel has been successfully connected")  # 输出连接成功信息

def MoveP2P(portHandler, packetHandler, DXL_MODEL, DXL_ID, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION,dxl_goal_position, DXL_MOVING_STATUS_THRESHOLD):
    index = 0
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        # if getch() == chr(0x1b):  # 退出
        #     break

        # Write goal position
        if (DXL_MODEL == 'XL320'):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table     特殊型号
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
        else:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read present position   读取当前位置
            if (DXL_MODEL == 'XL320'):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table     特殊型号
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            else:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID,ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))
            # 目标位置与当前位置差值小于阈值视为抵达
            if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                if index == 0:
                    index = 1
                else:
                    index = 0
                time.sleep(1)
                if (DXL_MODEL == 'XL320'):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table     特殊型号
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,dxl_goal_position[index])
                else:
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,dxl_goal_position[index])
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                # break
