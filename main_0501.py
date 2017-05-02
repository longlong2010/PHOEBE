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


def Brake():
  pi.write(PIN_MotorEN1,0)
  pi.write(PIN_MotorEN2,0)
  pi.write(PWM1,0)
  pi.write(PWM2,0)


def SetVoltage(Voltage):
#  dur = Voltage/VRef
#  dur2 = dur*255
 # dur = dur*200.0#into us
  global eps

  dur = Voltage/VRef
  dur2 = dur*255.0
  if (dur2>255): 
    dur2=255
  elif(dur2<-255):
    dur2=-255
  else:
    dur2=dur2
    
  if(dur>eps):
   # Start_Motor_ForW()#使能
    #print '+'
   # set_dc(1,dur)
    pi.write(PIN_MotorEN1,1)#允许旋转
    pi.write(PIN_MotorEN2,1)
    pi.set_PWM_dutycycle(PWM1,dur2)
    pi.write(PWM2,0)
  elif (dur<-1*eps):
   # print '-'
   # Start_Motor_BackW()
   # set_dc(2,dur)
    pi.write(PIN_MotorEN1,1)#允许旋转
    pi.write(PIN_MotorEN2,1)
    pi.set_PWM_dutycycle(PWM2,-1*dur2)
    pi.write(PWM1,0)
  else:
    Brake()

#**********************************执行入口**************************************
#串口初始化
#sensor=serial. Serial(port='/dev/serial0',baudrate='9600',timeout=1)
sensor=serial. Serial(port='/dev/ttyUSB0',baudrate='115200',timeout=1)

#姿态相关参数初始化设置
v = {};
az=0.0;
wz=0.0;
t0 = time.time();
I = 19428e-9;
psi_t = 60.0 / 180.0 * math.pi;
ac = AttitudeController(-0.02, 1, I, psi_t);

#电机相关参数设置
FREQ=5000
VRef = 8.0
eps = 0.1
PWM1=20
PWM2=21
PWM3=24
PWM4=25
GPIO=[PWM1, PWM2, PWM3, PWM4]
PIN_MotorEN1 = 17#电机使能
PIN_MotorEN2 = 18
_channels = len(GPIO)
_dc=[0]*_channels
#print _channels
#print _dc
_micros=1000000/FREQ
old_wid = None
pi = pigpio.pi()
if not pi.connected:
    exit(0)
pi.set_mode(PWM1, pigpio.OUTPUT)
pi.set_mode(PWM2, pigpio.OUTPUT)
pi.set_mode(PIN_MotorEN1, pigpio.OUTPUT)
pi.set_mode(PIN_MotorEN2, pigpio.OUTPUT)


try:
    while True:
        data = sensor.read(size=11)
        #数据对齐
        k = 0;
        for d in data:
            if d == b'\x55':
                break;
            k = k + 1;
        if k != 0:
            data = sensor.read(k);

        if data[0] != b'\x55':
            continue;

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

