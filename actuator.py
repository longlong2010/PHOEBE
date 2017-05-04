# -*- coding:utf-8 -*-
import pigpio;

class Actuator:
    FREQ = 5000
    VRef = 8.0
    eps = 0.1
    PWM1 = 20
    PWM2 = 21
    PIN_MotorEN1 = 17#电机使能
    PIN_MotorEN2 = 18
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            exit(0)
        self.pi.set_mode(self.PWM1, pigpio.OUTPUT)
        self.pi.set_mode(self.PWM2, pigpio.OUTPUT)
        self.pi.set_mode(self.PIN_MotorEN1, pigpio.OUTPUT)
        self.pi.set_mode(self.PIN_MotorEN2, pigpio.OUTPUT)
    def setVoltage(self, voltage):
        dur = voltage / self.VRef;
        dur2 = dur*255.0
        if (dur2>255): 
          dur2=255
        elif(dur2<-255):
          dur2=-255
        else:
          dur2=dur2
          
        if(dur>self.eps):
         # Start_Motor_ForW()#使能
          #print '+'
         # set_dc(1,dur)
          self.pi.write(self.PIN_MotorEN1,1)#允许旋转
          self.pi.write(self.PIN_MotorEN2,1)
          self.pi.set_PWM_dutycycle(self.PWM1,dur2)
          self.pi.write(self.PWM2,0)
        elif (dur<-1*self.eps):
         # print '-'
         # Start_Motor_BackW()
         # set_dc(2,dur)
          self.pi.write(self.PIN_MotorEN1,1)#允许旋转
          self.pi.write(self.PIN_MotorEN2,1)
          self.pi.set_PWM_dutycycle(self.PWM2,-1*dur2)
          self.pi.write(self.PWM1,0)
        else:
          self.brake()
    def brake(self):
        self.pi.write(self.PIN_MotorEN1,0)
        self.pi.write(self.PIN_MotorEN2,0)
        self.pi.write(self.PWM1,0)
        self.pi.write(self.PWM2,0)
