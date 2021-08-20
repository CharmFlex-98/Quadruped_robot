from __future__ import division
import time
import Adafruit_PCA9685
import pygame
import numpy as np
from inverse_kinematics import *
from math import *

class robot:
    def __init__(self, width=640, height=480):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

        self.direction={15:-1, 14:1, 13:-1, 12:1, 11:1, 10:-1, 9:1, 8:-1}
        self.origin=[84, 49, 74, 63, 99, 117, 114, 124]  # from 8 to 15, real servo degree!
        self.stand_up=[58, 104, 50, 112, 123, 67, 136, 74]
        self.thigh_calibration=[49, 63, 117, 124]  # 9, 11, 13, 15
        self.servo_arm_calibration=[43, 33, 140, 155] # 8, 10, 12, 14
        self.calibration={8:43, 9:49, 10:33, 11:63, 12:140, 13:117, 14:155, 15:124} # from 8 to 15

        self.servos={}

        self.initial=[]
        self.delta=[]

        self.reset()
        self.rad=0
        self.y_offset=0
        self.x_offset=0
        self.vertical=0
        self.vertical_direction=1
        self.counter=0

        self.status='sleep'  #'normal', 'sleep', 'sit', 'forward', 'backward, reset'

    def degree2pwm(self, degree):
        pwm=round((degree/180)*(600-150)+150)
        return pwm

    def pwm2degree(self, pwm):
        degree=round((pwm-150)/(600-150)*180)
        return degree

    def rotate_to(self, servo_index, target_degree, time_step, pov=False):
        if pov:
            target_degree=self.pov2servo_view(servo_index, target_degree)

        for index, i in enumerate(servo_index):
            self.initial.append(self.servos[i])
            target_pwm=self.degree2pwm(target_degree[index])
            self.delta.append((target_pwm-self.servos[i]))

        for step in range(time_step):
            for index, p in enumerate(servo_index):
                self.servos[p]=self.initial[index]+self.delta[index]*((step+1)/time_step)
                self.servos[p]=self.post_process_pwm(self.servos[p])
                self.pwm.set_pwm(p, 0, round(self.servos[p]))

        self.initial=[]
        self.delta=[]


        #print('rotate all done')
        #self.check_pwm()

    def rotate(self, servo_index, velocity):
        for index, i in enumerate(servo_index):
            self.servos[i]=self.servos[i]+velocity[index]*self.direction[i]
            self.servos[i]=self.post_process_pwm(self.servos[i])
            self.pwm.set_pwm(i, 0, round(self.servos[i]))
            print('servo {} pwm is {}'.format(i, self.servos[i]))

    def reset(self):
        for index, i in enumerate(range(8, 16)):
            self.servos[i]=self.degree2pwm(self.origin[index])
            self.pwm.set_pwm(i, 0, self.degree2pwm(self.origin[index]))
            time.sleep(0.01)

    def check_pwm(self):
        for i in self.servos:
            print('servo {} in degree:{} pwm:{}'.format(i, self.pwm2degree(self.servos[i]), self.servos[i]))

        time.sleep(1)

    def post_process_pwm(self, pwm, min_pwm=150, max_pwm=600):
        value=max(min(pwm, max_pwm), min_pwm)
        return value

    def pov2servo_view(self, servo_index, servo_pov_degree):
        real_degree=servo_pov_degree
        for index, i in enumerate(servo_index):
            if self.direction[i]==-1:
                real_degree[index]=180-servo_pov_degree[index]

        print(real_degree)
        return real_degree

    def walk(self, radius1, radius2, velocity):
        r1=radius1
        r2=radius2

        x1=r1*cos(self.rad)+self.x_offset
        y1=r2*sin(self.rad)+self.y_offset
        x2=r1*cos(self.rad+pi)+self.x_offset
        y2=r2*sin(self.rad+pi)+self.y_offset

        self.gait([15, 14, 9, 8], x1, y1, 1)
        self.gait([13, 12, 11, 10], x2, y2, 1)

        self.rad-=velocity

    def walk_turn(self, radius1, radius2, velocity, turn_rate=2, direction='left'):
        r1=radius1
        r2=radius2
        print(self.rad)

        x1=r1*cos(self.rad)+self.x_offset
        y1=r2*sin(self.rad)+self.y_offset
        x2=r1*cos(self.rad+pi)+self.x_offset
        y2=r2*sin(self.rad+pi)+self.y_offset

        if direction=='right':
            self.gait([15, 14], x1, y1, 1)
            self.gait([9, 8], x1/turn_rate, y1, 1)
            self.gait([13, 12], x2, y2, 1)
            self.gait([11, 10], x2/turn_rate, y2, 1)
        elif direction=='left':
            self.gait([15, 14], x1/turn_rate, y1, 1)
            self.gait([9, 8], x1, y1, 1)
            self.gait([13, 12], x2/turn_rate, y2, 1)
            self.gait([11, 10], x2/2, y2, 1)

        else:
            print('insert direction please!')
            return

        self.rad-=velocity

    def up_down(self, velocity):
        self.y_offset-=velocity
        self.gait(range(8, 16), self.x_offset, self.y_offset, 1)


    def front_back(self, velocity):
        self.x_offset-=velocity
        self.gait(range(8, 16), self.x_offset, self.y_offset, 1)

    def turning_body(self, radius, velocity, radius_multiplier=2.5, x_multiplier=4, y_multiplier=4):
        dif = radius * radius_multiplier * sin(self.rad)
        x_offset = radius * x_multiplier * cos(self.rad / 2) + self.x_offset-radius*x_multiplier
        y_offset = radius * y_multiplier * sin(self.rad/2) + self.y_offset
        rad = math.asin(dif / 100)

        x1 = (160 - dif - self.y_offset) * sin(rad) + x_offset + radius * cos(rad + pi / 2)
        y1 = (160 - dif - self.y_offset) * (1 - sin(rad + pi / 2)) + y_offset + dif
        x2 = (160 + dif - self.y_offset) * sin(rad) + x_offset + radius * cos(rad + pi / 2)
        y2 = (160 + dif - self.y_offset) * (1 - sin(rad + pi / 2)) + y_offset - dif

        self.gait([15, 14, 11, 10], x1, y1, 1)
        self.gait([13, 12, 9, 8], x2, y2, 1)
        self.rad += velocity

    def jump(self, radius, velocity):
        x=self.x_offset
        self.vertical=self.vertical+velocity*self.vertical_direction

        if self.vertical<=0:
            self.counter+=1
            self.vertical_direction*=-1
            self.vertical=0
        if self.vertical>=radius:
            self.vertical_direction*=-1
            self.vertical=radius
        print(self.vertical)

        y1=self.vertical*(self.counter%2)+self.y_offset
        y2=self.vertical*((self.counter+1)%2)+self.y_offset

        self.gait([15, 14, 9, 8], x, y1, 1)
        self.gait([13, 12, 11, 10], x, y2, 1)

    def body_slant(self, velocity):
        self.rad+=velocity
        dif=200*sin(self.rad)
        dif=dif/2

        front_legs_x=(160-dif-self.y_offset)*cos((pi/2)-self.rad)+self.x_offset
        front_legs_y=(160-dif-self.y_offset)*(1-sin((pi/2)+self.rad))+self.y_offset+dif
        hind_legs_x=(160+dif-self.y_offset)*cos((pi/2)-self.rad)+self.x_offset
        hind_legs_y=(160+dif-self.y_offset)*(1-sin((pi/2)+self.rad))+self.y_offset-dif

        self.gait([15, 14, 11, 10], front_legs_x, front_legs_y, 1)
        self.gait([13, 12, 9, 8], hind_legs_x, hind_legs_y, 1)

    def stand_reset(self, time_step, x=-22, y=-16):
        self.y_offset=y     #-13
        self.x_offset=x    #-6
        self.rad=0
        self.gait(range(8, 16), self.x_offset, self.y_offset, time_step)

    def gait(self, servos, dx, dy, time_step):
        servo_index=[]
        servo_angle=[]
        thigh_angle, arm_angle=ik_solver(dx, dy)
        for x in servos:
            servo_index.append(x)
            if x in [9, 11, 13, 15]:
                servo_angle.append(thigh_angle*self.direction[x]+self.calibration[x])
            else:
                servo_angle.append(arm_angle*self.direction[x]+self.calibration[x])

        self.rotate_to(servo_index, servo_angle, time_step, pov=False)


#my_robot.rotate_to([15, 13, 11, 9], [110, 110, 110, 110], 50)
#my_robot.check_pwm()