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

import time     # 加载时间库
import serial   # 加载串口库
import sys      # 加载系统库
import platform     # 加载操作系统库

LATENCY_TIMER = 16  # 延迟计时器
DEFAULT_BAUDRATE = 1000000  # 默认波特率


class PortHandler(object):      # 类
    def __init__(self, port_name):  # 成员变量
        self.is_open = False        # 是否打开标志
        self.baudrate = DEFAULT_BAUDRATE    # 波特率
        self.packet_start_time = 0.0    # 开始时间
        self.packet_timeout = 0.0       # 超时
        self.tx_time_per_byte = 0.0

        self.is_using = False       # 是否正在是否标志
        self.port_name = port_name  # 串口端口号
        self.ser = None

    def openPort(self):     # 打开端口
        return self.setBaudRate(self.baudrate)

    def closePort(self):    # 关闭端口
        self.ser.close()
        self.is_open = False

    def clearPort(self):    # 清空端口
        self.ser.flush()

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):  # 获取端口名字
        return self.port_name

    def setBaudRate(self, baudrate):    # 设置波特率
        baud = self.getCFlagBaud(baudrate)

        if baud <= 0:   # 期望波特率小于0 返回错误
            # self.setupPort(38400)
            # self.baudrate = baudrate
            return False  # TODO: setCustomBaudrate(baudrate)
        else:
            self.baudrate = baudrate        # 设置目标波特率
            return self.setupPort(baud)     # 设置串口

    def getBaudRate(self):          # 获取当前波特率
        return self.baudrate

    def getBytesAvailable(self):
        return self.ser.in_waiting

    def readPort(self, length):
        if (sys.version_info > (3, 0)):
            return self.ser.read(length)
        else:
            return [ord(ch) for ch in self.ser.read(length)]

    def writePort(self, packet):
        return self.ser.write(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (LATENCY_TIMER * 2.0) + 2.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):       # 获取当前时间
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):    # 计算通信时间
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since

    def setupPort(self, cflag_baud):
        if self.is_open:        # 判断端口是否打开
            self.closePort()    # 关闭串口

        # 串口对象实例化，设置串口属性
        self.ser = serial.Serial(
            port=self.port_name,        # 串口端口号
            baudrate=self.baudrate,     # 波特率
            # parity = serial.PARITY_ODD,   # 奇偶校验
            # stopbits = serial.STOPBITS_TWO,   # 停止位
            bytesize=serial.EIGHTBITS,      # 数据位
            timeout=0
        )

        self.is_open = True

        self.ser.reset_input_buffer()   # 重置输入缓冲区

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0     # 计算发送速率 单位时间的字节数

        return True

    def getCFlagBaud(self, baudrate):   # 判断波特率是否支持
        if baudrate in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000,
                        2000000, 2500000, 3000000, 3500000, 4000000]:
            return baudrate
        else:
            return -1            
