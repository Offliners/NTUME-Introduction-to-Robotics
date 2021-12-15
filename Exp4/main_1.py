#!/usr/bin/env python3

from time import sleep
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.sound import Sound
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, LargeMotor, MoveTank, SpeedDPS
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
        self.m.on_for_degrees(speed, theta, False)
    
    def motorInfo(self):
        print(self.m.state)
    
    def reset(self):
        self.m.reset()

# Two Link Manipulator Class
class TwoLinkManipulator:
    def __init__(self, l1, l2, path):
        self.L1 = l1 # 10.5 cm
        self.L2 = l2 # 8 cm
        self.Path = path
        self.move(0, 0) # Move to origin (0, 0) we assume
        self.J0_loc = np.array([0, 0])
    
    def FK(self, theta1, theta2):
        J1_loc = self.J0_loc + [self.L1 * np.cos(theta1), self.L1 * np.sin(theta1)]
        J2_loc = J1_loc + [self.L2 * np.cos(theta1 + theta2), self.L2 * np.sin(theta1 + theta2)]
        return J1_loc, J2_loc

    def IK(self, x, y):
        theta2 = np.arccos((x ** 2 + y ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2))
        theta1 = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(theta2), self.L2 * np.cos(theta2) + self.L1)
        return np.degrees(theta1), np.degrees(theta2)

    def move(self, theta1, theta2):
        maxspeed = 10
        if theta1 != 0 or theta2 != 0:
            p1 = maxspeed * abs(theta1) / (abs(theta1) + abs(theta2))
            p2 = maxspeed * abs(theta2) / (abs(theta1) + abs(theta2))
            t1 = Thread(target=m1.run, args=(p1, theta1,))
            t2 = Thread(target=m2.run, args=(p2, theta2,))
            t1.start()
            t2.start()
            t1.join()
            t2.join()

def TrajectoryPlanner(x, y, z):
    pass
    
# Initialize
sound = Sound()
m1 = MotorControl(OUTPUT_A, 'normal')
m2 = MotorControl(OUTPUT_B, 'normal')
arm = TwoLinkManipulator(10.5, 8, None)

def Menu():
    while True:
        os.system('cls || clear')
        print('====== Two Link Manipulator ======')
        print('1. Move to origin')
        print('2. Parameeter setting')
        print('3. Hand control mode')
        print('4. Write SUN')
        print('5. Write Spring')
        print('6. Motor Info')
        print('0. Exit')
        print('==================================')
        
        select = int(input('Please input your select : '))
        global arm
        if select == 0:
            break
        elif select == 1:
            pass
        elif select == 2:
            l1 = float(input('Please input length of Link 1 : '))
            l2 = float(input('Please input length of Link 2 : '))
            arm = TwoLinkManipulator(l1, l2, None)
        elif select == 3:
            while True:
                t1, t2 = map(float, input('Please input theta1 and theta2 (split by space, input \'0 0\' to leave this mode) : ').split(' '))
                if t1 == 0 and t2 == 0:
                    break
                else:
                    arm.move(t1, t2)
        elif select == 6:
            print('Motor 1 info : ')
            m1.motorInfo()
            print('Motor 2 info : ')
            m2.motorInfo()
            input('Press any key to exit...')
    
    return 0


# SUN
arm.move(90, 90)
x = np.array([10.5, 4.5, 10.5, 10.5, 4.5, 7.5, 7.5, 7.5, 4.5, 4.5])
y = np.array([8, 8, 8, 12, 12, 8, 12, 8, 8, 12])

for i in range(1, x.shape[0]):
    

t_x = []
t_y = []
for i in range(5):
    temp1, temp2 = arm.IK(x[i], y[i])
    t_x.append(temp1)
    t_y.append(temp2)

print(t_x)
print(t_y)

for i in range(1, 5):
    arm.move(t_x[i] - t_x[i - 1], t_y[i] - t_y[i - 1])

# Start Alert
sound.beep()

# if Menu() == 0:
#     print('End')
# else:
#     print('Error')

m1.reset()
m2.reset()

# End Alert
sound.beep()
