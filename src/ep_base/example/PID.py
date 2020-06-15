from __future__ import absolute_import
from __future__ import division

import time


class PID(object):
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.kp_gain = 10 * self.Kp
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.kp_callbackflag = 0
        self.clear()

    def clear(self):
        self.error = 0
        self.read = 0
        self.target = 0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0

    def kp_callback(self):
        if abs(self.error) < 10:
            temp_kp = self.Kp
        else:
            temp_kp = self.kp_gain / abs(self.error) * 3
            print
            u'kp', temp_kp, u'ke', self.Kp, u'g', self.kp_gain, abs(self.error)
        return temp_kp

    def update(self, target, read):
        self.read = read
        self.target = target
        self.error = self.read - self.target
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        print
        u'time', delta_time
        delta_error = self.error - self.last_error
        if (delta_time >= self.sample_time):
            if self.kp_callbackflag == 1:
                temp_kp = self.kp_callback()
            else:
                temp_kp = self.Kp
            self.PTerm = temp_kp * self.error  # 比例
            if abs(self.PTerm) > 70:
                self.PTerm = 70 * self.PTerm / abs(self.PTerm)
            print
            self.PTerm
            self.ITerm += self.error * delta_time  # 积分
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
                # print('d',self.DTerm,'e',self.error)
            self.last_time = self.current_time
            self.last_error = self.error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setkp_callback(self, flag):
        self.kp_callbackflag = flag

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
