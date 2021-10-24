#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class P_controller:
    def __init__(self, P):
        self.P = P

    def control(self, error):
        return self.P * error

def Check(color, dis, blackExist, redExist, blueExist, index):
    Turn(-90)
    wait(100)
    forward = 130 #- index * 10

    mobile_car.straight(forward)

    flag = 0
    print(cSensor.color())
    if (cSensor.color() == Color.BLACK or cSensor.color() == None) and blackExist == 1:
        gripperMotor.run_angle(-50, 30)
        blackExist = 0
        flag = 1
    elif cSensor.color() == Color.RED and blackExist == 0 and redExist == 1:
        gripperMotor.run_angle(-50, 30)
        redExist = 0
        flag = 2
    elif (cSensor.color() == Color.BLUE or cSensor.color() == Color.GREEN) and blackExist == 0 and redExist == 0:
        gripperMotor.run_angle(-50, 30)
        flag = 3

    mobile_car.straight(-forward)
    Turn(0)
    wait(100)
    
    if flag == 1:
        print('Throw black')
        GoStraight(660 - dis, 0)
        gripperMotor.run_angle(50, 30)
        Turn(0)
        wait(100)
        blackExist = 0
    elif flag == 2:
        print('Correct Red')
        GoStraight(240 - dis, 0)
        Turn(-90)
        mobile_car.straight(forward)
        gripperMotor.run_angle(50, 30)
        wait(100)
        mobile_car.straight(-forward)
        Turn(0)
        redExist = 0
    elif flag == 3:
        print('Correct Blue')
        GoStraight(660 - dis, 0)
        Turn(-90)
        mobile_car.straight(forward)
        gripperMotor.run_angle(50, 30)
        wait(100)
        mobile_car.straight(-forward)
        Turn(0)
        blueExist = 0

    return blackExist, redExist, blueExist

def GoStraight(distance, degree):
    mobile_car.reset()
    if distance > 0:
        while mobile_car.distance() < distance:
            mobile_car.drive(100, 0)

            if mobile_car.distance() != 0 and mobile_car.distance() % 100 == 0:
                Turn(degree)
                wait(5)
    else:
        while mobile_car.distance() > distance:
            mobile_car.drive(-100, 0)

            if mobile_car.distance() != 0 and (-1 * mobile_car.distance()) % 100 == 0:
                Turn(degree)
                wait(5)

def Turn(degree):
    if degree >= 0:
        sign = 1
    else:
        sign = -1

    while True:
        if sign * gSensor.angle() <=  degree:
            ctrl = P_controller(0.1)
            gain = 10
            while sign * gSensor.angle() !=  degree:
                gain += ctrl.control(gSensor.angle() - sign * degree)
                mobile_car.drive(0, gain)
                wait(5)
            mobile_car.stop()
            break

        mobile_car.drive(0, 50 * sign)
        wait(5)

# Initialize
ev3 = EV3Brick()
err = 0
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
Lmotor.reset_angle(0)
Rmotor.reset_angle(0)
gripperMotor = Motor(Port.D)
gripperMotor.reset_angle(0)
gSensor = GyroSensor(Port.S1)
gSensor.reset_angle(0)
cSensor = ColorSensor(Port.S2)
Goals = [[Color.RED, 240], [Color.BLACK, 210], [Color.BLUE, 210]]
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=110)
mobile_car.reset()

# Start alert
ev3.speaker.beep()

dis = 0
blackExist = 1
redExist = 1
blueExist = 1

print('Round 1')
for c in Goals:
    dis += c[1]
    GoStraight(c[1], 0)
    blackExist, redExist, blueExist = Check(c[0], dis, blackExist, redExist, blueExist, 0)

    if blackExist == 0:
        break
        
GoStraight(-700, 0)
Turn(0)

Lmotor.reset_angle(0)
Rmotor.reset_angle(0)
dis = -10
print('Round 2')
for c in Goals:
    dis += c[1]
    GoStraight(c[1], 0)
    blackExist, redExist, blueExist = Check(c[0], dis, blackExist, redExist, blueExist, 1)

    if redExist == 0:
        break

GoStraight(-240, 0)
Turn(0)

Lmotor.reset_angle(0)
Rmotor.reset_angle(0)
dis = 0
print('Round 3')
for c in Goals:
    dis += c[1]
    GoStraight(c[1], 0)
    blackExist, redExist, blueExist = Check(c[0], dis, blackExist, redExist, blueExist, 1)

    if blueExist == 0:
        break

GoStraight(-700, 0)

ev3.speaker.beep()
