#! /usr/bin/python
# -*- coding:utf-8 -*-
import math;
import serial;
import time;
import struct;
import binascii;
class Integrator:
    def __init__(self, u = 0, v = None, t = None):
        self.v = v;
        self.t = t;
        self.u = u;
    def integrate(self, v, t):
        if self.t != None:
            dt = t - self.t;
            self.u += (v + self.v) / 2 * dt;
        self.t = t;
        self.v = v;
        return self.u;
class AttitudeController:
    def __init__(self, u, D, I, psi_t):
        self.u = u;
        self.D = D;
        self.I = I;
        self.psi_t = psi_t;
        self.integrator = Integrator();
    def getControlTorque(self, t, psi, omega):
        u = self.u;
        K = 1 - math.exp(u * t);
        D = self.D;
        I = self.I;
        psi_t = self.psi_t;
        Tc = -D * omega - K * (psi - psi_t);
        return Tc;
    def getVoltage(self, t, psi, omega):
        Tc = self.getControlTorque(t, psi, omega);
        G = self.integrator.integrate(Tc, t);
        return (G / I) * 180 / math.pi / 6 / 650.2;

#sensor=serial. Serial(port='/dev/serial0',baudrate='9600',timeout=1)
sensor=serial. Serial(port='/dev/ttyUSB0',baudrate='115200')
v = {};
az=0.0;
wz=0.0;
t=0.0;
I = 148960e-9;
psi_t = 30 / 180 * math.pi;
ac = AttitudeController(-0.02, 0.05, I, psi_t);
#while True:
#    data = sensor.read(size=1)
#    if data == b'\x55':
#        print '接收到数据！'
#        sensor.read(size=10)
#        break
#    print 'tring',data

try:
    while True:
        #time.sleep()
        data = sensor.read(size=11)    
        k = 0;
        for d in data:
            if d == b'\x55':
				break;
            k = k + 1;
        if k != 0:
            data = sensor.read(k);

        if data[0] != b'\x55':
            continue;

        if not len(data) == 11:
            print '字节错误！'
            break
        if data[1] == b'\x52':
            omega = struct.unpack('h', data[6] + data[7])[0] /32768.0 * 2000 * math.pi / 180;
            v['0'] = omega;
        elif data[1] == b'\x53':
            psi = struct.unpack('h', data[6] + data[7])[0] /32768.0 * 180 * math.pi / 180;
            v['1'] = psi;
        t = time.time();
        if v.has_key('0') and v.has_key('1'):
            print "%f\t%f" % (v['0']*180/math.pi, v['1']*180/math.pi);
            voltage = ac.getVoltage(t, psi, omega);
            print(voltage);
            v = {};
            
except KeyboardInterrupt:
    sensor.close()
    print('关闭传感器 !')
