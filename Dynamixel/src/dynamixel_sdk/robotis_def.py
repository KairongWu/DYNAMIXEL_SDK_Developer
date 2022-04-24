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

# Author: Ryu Woon Jung (Leon)

# 舵机相关参数定义

BROADCAST_ID = 0xFE  # 254  广播ID    使所有连接的设备执行指令包
MAX_ID = 0xFC  # 252        最大ID

# Instruction for DXL Protocol  dxl 通信命令指令
# https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-packet
INST_PING = 1           # 检查数据包是否已到达与数据包ID相同的设备的指令
INST_READ = 2           # 从设备读取数据
INST_WRITE = 3          # 往设备写入数据
INST_REG_WRITE = 4      # 将指令包注册到待机状态的指令；数据包稍后通过Action命令执行
INST_ACTION = 5         # 执行预先使用Reg Write注册的数据包的指令
INST_FACTORY_RESET = 6  # 将控制表重置为其初始出厂默认设置的指令
INST_CLEAR = 16         # 重置某些信息的指令
INST_SYNC_WRITE = 131   # 0x83 对于多个设备，一次在同一地址以相同长度写入数据的指令
INST_BULK_READ = 146    # 0x92 对于多个设备，一次从不同地址以不同长度读取数据的指令
# --- Only for 2.0 ---  仅支持 2.0 协议
INST_REBOOT = 8         # 重新启动设备的说明
INST_STATUS = 85  # 0x55    返回指令包的数据包
INST_SYNC_READ = 130  # 0x82    对于多个设备，一次从同一地址以相同长度读取数据的指令
INST_BULK_WRITE = 147  # 0x93   对于多个设备，一次在不同地址以不同长度写入数据的指令

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success   通信成功
COMM_PORT_BUSY = -1000  # Port is busy (in use)             端口繁忙
COMM_TX_FAIL = -1001  # Failed transmit instruction packet  发送指令包失败
COMM_RX_FAIL = -1002  # Failed get status packet            获取状态包失败
COMM_TX_ERROR = -2000  # Incorrect instruction packet       错误的指令包
COMM_RX_WAITING = -3000  # Now recieving status packet      正在接收状态包
COMM_RX_TIMEOUT = -3001  # There is no status packet        没有状态包
COMM_RX_CORRUPT = -3002  # Incorrect status packet          错误的状态包
COMM_NOT_AVAILABLE = -9000  #   连接失败，不可行


# Macro for Control Table Value 控制表值的宏
def DXL_MAKEWORD(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)


def DXL_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16


def DXL_LOWORD(l):
    return l & 0xFFFF


def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF


def DXL_LOBYTE(w):
    return w & 0xFF


def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF
