#! /usr/bin/python
# -*- coding:utf-8 -*-
from threading import Thread;
import time;
import math;
import struct;
import serial;
import pigpio;
from datetime import datetime;
#积分器
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
#姿态控制器
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
        #print "%f\t%f\t%f" % (-D * omega, -K * (psi - psi_t), Tc)
        return Tc;
    def getVoltage(self, t, psi, omega):
        Tc = self.getControlTorque(t, psi, omega);
        G = self.integrator.integrate(Tc, t);
        return (G / I) * 180 / math.pi / 6 / 1353;

def set_dc(channel, dc):
   global old_wid
   if dc < 0:
      dc = 0
   elif dc > _micros:
      dc = _micros
   _dc[channel] = dc
   for c in range(_channels):
      d = _dc[c]
      g = GPIO[c]
      if d == 0:
         pi.wave_add_generic([pigpio.pulse(0, 1<<g, _micros)])
      elif d == _micros:
         pi.wave_add_generic([pigpio.pulse(1<<g, 0, _micros)])
      else:
         pi.wave_add_generic(
            [pigpio.pulse(1<<g, 0, d), pigpio.pulse(0, 1<<g, _micros-d)])
   new_wid = pi.wave_create()
   if old_wid is not None:
      pi.wave_send_using_mode(new_wid, pigpio.WAVE_MODE_REPEAT_SYNC)
      while pi.wave_tx_at() != new_wid:
         pass
      pi.wave_delete(old_wid)
   else:
      pi.wave_send_repeat(new_wid)
   old_wid = new_wid

def SetVoltage(Voltage):
  dur = Voltage/VRef
  dur = dur*200.0#into us
  set_dc(0,dur)

#**********************************执行入口**************************************
#串口初始化
#sensor=serial. Serial(port='/dev/serial0',baudrate='9600',timeout=1)
sensor=serial. Serial(port='/dev/ttyUSB1',baudrate='115200',timeout=1)

#姿态相关参数初始化设置
v = {};
az=0.0;
wz=0.0;
t=0.0;
I = 19428e-9;
psi_t = 30.0 / 180.0 * math.pi;
ac = AttitudeController(-0.02, 0.05, I, psi_t);

#电机相关参数设置
FREQ=5000
VRef = 8.0
PWM1=22
PWM2=23
PWM3=24
PWM4=25
GPIO=[PWM1, PWM2, PWM3, PWM4]
_channels = len(GPIO)
_dc=[0]*_channels
_micros=1000000/FREQ
old_wid = None
pi = pigpio.pi()
if not pi.connected:
    exit(0)
pi.set_mode(PWM1, pigpio.OUTPUT)
#判断读数是否成功
while True:
    data = sensor.read(size=1)
    if data == b'\x55':
        print '接收到数据！'
        sensor.read(size=10)
        break
    print 'tring',data
try:
    while True:
        data = sensor.read(size=11)
        #判断字节数是否正确
        if not len(data) == 11:
            print '字节错误！'
            break
        #角速度
        if data[1] == b'\x52':
            omega = struct.unpack('h', data[6] + data[7])[0] / 32768.0 * 2000 * math.pi / 180;
            v['0'] = omega;
        #角度
        elif data[1] == b'\x53':
            psi = struct.unpack('h', data[6] + data[7])[0] / 32768.0 * 180 * math.pi / 180;
            v['1'] = psi;
        #获得系统当前时间
        t = time.time();
        #转换成电压
        if v.has_key('0') and v.has_key('1'):
            #print "%f\t%f" % (v['0']*180/math.pi, v['1']*180/math.pi);
            voltage = ac.getVoltage(t, psi, omega);
            SetVoltage(voltage)
          #  time.sleep(10.5)
            #print(voltage);
            print "%s\t%f\t%f\t%f" % (datetime.fromtimestamp(t), psi * 180 / math.pi, omega * 180 / math.pi, voltage);
            v = {};
except KeyboardInterrupt:
    pi.wave_tx_stop()
    if old_wid is not None:
        pi.wave_delete(old_wid)
    pi.write(PWM1, 0);    
    pi.stop()
    sensor.close()
    print('关闭传感器 !')
