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

def Check(index, color, dis, ballExist):
    Turn(-90)
    wait(100)
    forward = 120
    gripperAngle = 30
    gripperPower = 50
    mobile_car.straight(forward)

    flag = 0
    print(cSensor.color())
    
    if cSensor.color() == Color.BLACK or cSensor.color() == None:
        ballExist[0][2] = index
        if ballExist[0][1]:
            gripperMotor.run_angle(-gripperPower, gripperAngle)
            flag = 1
    elif cSensor.color() == Color.RED: 
        ballExist[1][2] = index
        if (not ballExist[0][1]) and ballExist[1][1] and (ballExist[1][2] < ballExist[2][2]):
            gripperMotor.run_angle(-gripperPower, gripperAngle)
            flag = 2
    elif (cSensor.color() == Color.BLUE or cSensor.color() == Color.GREEN):
        ballExist[2][2] = index
        if (not ballExist[1][1]) and (ballExist[2][2] < ballExist[1][2]):
            gripperMotor.run_angle(-gripperPower, gripperAngle)
            flag = 3

    mobile_car.straight(-forward)
    Turn(0)
    wait(100)
    
    if flag == 1:
        print('Throw black')
        GoStraight(650 - dis, 0)
        gripperMotor.run_angle(gripperPower, gripperAngle)
        Turn(0)
        wait(100)
        ballExist[0][1] = False
        dis = 700
    elif flag == 2:
        print('Correct Red')
        GoStraight(250 - dis, 0)    
        Turn(-90)
        mobile_car.straight(forward)
        gripperMotor.run_angle(gripperPower, gripperAngle)
        wait(100)
        mobile_car.straight(-forward)
        Turn(0)
        ballExist[1][1] = False
    elif flag == 3:
        print('Correct Blue')
        GoStraight(650 - dis, 0)
        Turn(-90)
        mobile_car.straight(forward)
        gripperMotor.run_angle(gripperPower, gripperAngle)
        wait(100)
        mobile_car.straight(-forward)
        Turn(0)
        ballExist[2][1] = False

    return ballExist, dis

def GoStraight(distance, degree):
    mobile_car.reset()
    if distance > 0:
        while mobile_car.distance() < distance:
            mobile_car.drive(130, 0)

            if mobile_car.distance() != 0 and mobile_car.distance() % 100 == 0:
                Turn(degree)
    else:
        while mobile_car.distance() > distance:
            mobile_car.drive(-130, 0)

            if mobile_car.distance() != 0 and (-1 * mobile_car.distance()) % 100 == 0:
                Turn(degree)

    print(mobile_car.distance())

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
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
gripperMotor = Motor(Port.D)
gripperMotor.reset_angle(0)
gSensor = GyroSensor(Port.S1)
gSensor.reset_angle(0)
cSensor = ColorSensor(Port.S2)
 
# [Color, distance]
Goals = [[Color.RED, 230], [Color.BLACK, 200], [Color.BLUE, 200]]

# [Color, isExist, found order]
ballExist = [[Color.BLACK, True, 3], [Color.RED, True, 3], [Color.BLUE, True, 3]]
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=120)
mobile_car.reset()

# Start alert
ev3.speaker.beep()


for i in range(3):
    print('Round ' + str(i + 1))

    dis = 0
    Lmotor.reset_angle(0)
    Rmotor.reset_angle(0)
    count = 0
    for c in Goals:
        if i != 0 and c[0] == Color.RED:
            pass
        else:
            dis += c[1]
            GoStraight(c[1], 0)
        
        ballExist, dis = Check(count, c[0], dis, ballExist)
        count += 1

        if not ballExist[i][1]:
            break
        
    GoStraight(-dis + 220, 0)
    Turn(0)

GoStraight(-220, 0)

ev3.speaker.beep()
