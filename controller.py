# -*- coding:utf-8 -*-  
from numpy import *;
#import scipy.integrate;
import math;
import time;

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
    #AttitudeController 构造方法
    #u, D 控制参数
    #I 飞轮转动惯量
    #psi_t 目标角度
    def __init__(self, u, D, I, psi_t):
        self.u = u;
        self.D = D;
        self.I = I;
        self.psi_t = psi_t;
        self.integrator = Integrator();
    #PD算法计算控制力距
    #t 经过的时间
    #psi 当前的角度
    #omega 当前的角速度 
    #返回需要的控制力矩
    def getControlTorque(self, t, psi, omega):
        u = self.u;
        K = 1 - math.exp(u * t);
        D = self.D;
        I = self.I;
        psi_t = self.psi_t;
        Tc = -D * omega - K * (psi - psi_t);
        return Tc;
    #PD算法计算电机对应的电压
    #t 经过的时间
    #psi 当前的角度
    #omega 当前的角速度
    #返回电机对应的电压
    def getVoltage(self, t, psi, omega):
        Tc = self.getControlTorque(t, psi, omega);
        #G = self.integrator.integrate(Tc, t);
        G = Tc * 2;
        return (G / self.I) * 180 / math.pi / 6 / 1353;


if __name__ == '__main__':
    I = 19428.0e-9;

    psi_t = 30.0 / 180.0 * math.pi;
    ac = AttitudeController(-0.02, 0.05, I, psi_t);
    y0 = zeros(2);

    def f(y, t):
        ydot = zeros(2);
        psi = y[0];
        omega = y[1];
        Tc = ac.getControlTorque(t, psi, omega);
        ydot[0] = omega;
        ydot[1] = Tc / 9952843.1e-9;
        return ydot;
    tspan = linspace(10000000, 10000020, 501);
    y = scipy.integrate.odeint(f, y0, tspan);
    k = 0;
    for t in tspan:
        v = y[k];
        k = k + 1;
        print("%f\t%f" % (t, v[0] * 180 / math.pi));
        #print("%f\t%f" % (t, ac.getVoltage(t, v[0], v[1])));
