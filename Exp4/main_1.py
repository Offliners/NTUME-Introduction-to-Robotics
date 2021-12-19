#!/usr/bin/env python3

from time import sleep
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.sound import Sound
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor, MediumMotor
from threading import Thread
import os
import numpy as np

# Large Motor Build Class 
class MotorControl(Thread):
    def __init__(self, port, rot_dir):
        Thread.__init__(self)
        self.m = LargeMotor(port)
        self.m.polarity = rot_dir

    def run(self, speed, theta):
        self.m.on_for_degrees(speed, theta, brake=False)
    
    def motorInfo(self):
        print(self.m.state)
    
    def reset(self):
        self.m.reset()
    
    def stop(self):
        self.m.stop()

# Two Link Manipulator Class
class TwoLinkManipulator:
    def __init__(self, l1, l2, decelerate_ratio):
        self.L1 = l1
        self.L2 = l2
        self.dr = decelerate_ratio
        self.J0_loc = np.array([0, 0])
    
    def FK(self, theta1, theta2):
        J1_loc = self.J0_loc + [self.L1 * np.cos(theta1), self.L1 * np.sin(theta1)]
        J2_loc = J1_loc + [self.L2 * np.cos(theta1 + theta2), self.L2 * np.sin(theta1 + theta2)]
        return J1_loc, J2_loc

    def IK(self, x, y):
        theta2 = np.arccos((x ** 2 + y ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2))
        theta1 = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(theta2), self.L2 * np.cos(theta2) + self.L1)
        return self.dr * np.degrees(theta1), self.dr * np.degrees(theta2)

    def move(self, theta1, theta2):
        t12 = abs(theta1) + abs(theta2)
        maxspeed = 50
        if theta1 != 0 or theta2 != 0:
            p1 = maxspeed * abs(theta1) / t12
            p2 = maxspeed * abs(theta2) / t12
            t1 = Thread(target=m1.run, args=(p1, theta1,))
            t2 = Thread(target=m2.run, args=(p2, theta2,))
            t1.start()
            t2.start()
            t1.join()
            t2.join()
    
# Initialize
sound = Sound()
m1 = MotorControl(OUTPUT_B, 'normal')
m2 = MotorControl(OUTPUT_C, 'normal')
m3 = MediumMotor(OUTPUT_D)
decelerate_ratio = 24
arm = TwoLinkManipulator(10, 8.5, decelerate_ratio)

# SUN
# sun_dx = np.array([-4, 5.657, 0.7071, -0.7071 + -5.657, 0.7071, -4.243, 5.657, -0.471, 0.471 + 1.4142, 0.7071, -0.7071, -4.243, -2.357, 2.357 + 7.071, -2.8284, 2.8284, -4.243])
# sun_dy = np.array([-1, 5.657, 0.7071, -0.7071 + -5.657, -0.7071, 4.243, 5.657, 2.357, -2.357 + -7.071, -0.7071, 0.7071, 4.243, 0.471, -0.471 + -1.414, 2.8284, -2.8284, 4.243])
# sun_z = np.array([0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
sun_dx = np.array([-4, 5.65685, -5.65685, -4.24264, 5.65685, 1.41421, -4.24264, 7.07107, -4.24264])
sun_dy = np.array([-1, 5.65685, -5.65685, 4.24264, 5.65685, -7.07107, 4.24264, -1.41421, 4.24264])
sun_z = np.array([0, 1, 0, 1, 1, 0, 1, 0, 1])

# Spring
spring_dx = np.array([-4, 0, 1, 0, 1, 0, -3, 4, -1, 1, 0, 4, -4, 0, 4, -2, 0, 2, 0])
spring_dy = np.array([-1, 3, -3.5, 3, -4, 5, -2.5, -1.5, 4, 1, -4, 0, 0, 2, 0, -2, 2, -2, 2])
spring_z = np.array([0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1])

start_point = np.array([10, 5.4])
def TrajectoryPlanner(dx, dy, z, start):
    t_x = []
    t_y = []
    t_z = []
    t1 = start[0]
    t2 = start[1]
    interval = 15
    for i in range(dx.shape[0]):
        for j in range(interval):
            t1 += dx[i] * 1 / interval
            t2 += dy[i] * 1 / interval
            temp1, temp2 = arm.IK(t1, t2)
            t_x.append(temp1)
            t_y.append(temp2)
            t_z.append(z[i])

    return t_x, t_y, t_z

def Menu():
    history_angle = np.array([0, 0])
    while True:
        os.system('cls || clear')
        print('====== Two Link Manipulator ======')
        print('1. Move to origin')
        print('2. Parameeter setting')
        print('3. Hand control mode')
        print('4. Write SUN')
        print('5. Write Spring')
        print('6. Motor Info')
        print('7. Stop all motors')
        print('0. Exit')
        print('==================================')
        
        select = int(input('Please input your select : '))
        global arm
        if select == 0:
            break
        elif select == 1:
            arm.move(-history_angle[0], -history_angle[1])
        elif select == 2:
            l1 = float(input('Please input length of Link 1 : '))
            l2 = float(input('Please input length of Link 2 : '))
            arm = TwoLinkManipulator(l1, l2, None)
        elif select == 3:
            while True:
                t1, t2, t3 = map(float, input('Please input theta1, theta2 and theta3 (split by space, input \'0 0 0\' to leave this mode) : ').split(' '))
                if t1 == 0 and t2 == 0 and t3 == 0:
                    break
                else:
                    m3.on_for_degrees(30, t3)
                    arm.move(decelerate_ratio * t1, decelerate_ratio * t2)
                    endpoint = start_point + np.array([-3, 6])
        elif select == 4:
            t_x, t_y, t_z = TrajectoryPlanner(sun_dx, sun_dy, sun_z, start_point)
            for i in range(1, len(t_x)):
                m3.on_for_degrees(30, 90 * (t_z[i] - t_z[i - 1]))
                arm.move((t_x[i] - t_x[i - 1]), (t_y[i] - t_y[i - 1]))
            m3.on_for_degrees(30, -90)
            
            history_angle = np.array([t_x[-1] - t_x[0], t_y[-1] - t_y[0]])
        elif select == 5:
            t_x, t_y, t_z = TrajectoryPlanner(spring_dx, spring_dy, spring_z, start_point)
            for i in range(1, len(t_x)):
                m3.on_for_degrees(30, 90 * (t_z[i] - t_z[i - 1]))
                arm.move((t_x[i] - t_x[i - 1]), (t_y[i] - t_y[i - 1]))
            m3.on_for_degrees(30, -90)
            
            history_angle = np.array([t_x[-1] - t_x[0], t_y[-1] - t_y[0]])
        elif select == 6:
            print('Motor 1 info : ')
            m1.motorInfo()
            print('Motor 2 info : ')
            m2.motorInfo()
            print('Motor 3 info : ')
            print(m3.state)
            input('Press any key to exit...')
        elif select == 7:
            m1.stop()
            m2.stop()
            m3.stop()
    return 0

# Start Alert
sound.beep()

if Menu() == 0:
    print('End')
else:
    print('Error')

m1.reset()
m2.reset()
m3.reset()

# End Alert
sound.beep()
