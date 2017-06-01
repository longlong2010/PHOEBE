#! /usr/bin/python
# -*- coding:utf-8 -*-
#good sensor
from threading import Thread;
import time;
import math;
import struct;
import serial;
import pigpio;
from datetime import datetime;
from controller import *;
from jy901 import *; 
from actuator import *;

#**********************************执行入口**************************************
#sensor = Jy901Serial('/dev/ttyUSB0', 115200);
sensor = Jy901I2c(0x50);
#姿态相关参数初始化设置
t0 = time.time();
I = 19428e-9;
psi_t = 120.0 / 180.0 * math.pi;
ac = AttitudeController(-0.02, 0.2, I, psi_t);
act = Actuator(); 
try:
    while True:
        #角速度
        #角度
        omega = sensor.getAngularVelocity();
        psi = sensor.getAngle();
        #获得系统当前时间
        t = time.time();
        voltage = ac.getVoltage(t - t0, psi, omega);
        #act.setVoltage(voltage)
        print "%s\t%f\t%f\t%f" % (datetime.fromtimestamp(t), psi * 180 / math.pi, omega * 180 / math.pi, voltage);
except KeyboardInterrupt:
    act.brake();
