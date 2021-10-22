#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def Check(color, err):
    Forward(color)
    wait(1000)
    Back(color)
    wait(1000)
    
    return err

def Forward(color):
    dis = 210 + Goals.index(color) * 200
    mobile_car.straight(dis)
    Turn(-87)
    mobile_car.straight(100)

def Back(color):
    mobile_car.straight(-100)
    Turn(87)
    dis = 210 + Goals.index(color) * 200
    mobile_car.straight(-dis)

def Turn(degree):
    curDegree = gSensor.angle()
    if degree > 0:
        sign = 1
    elif degree < 0:
        sign = -1
    else:
        sign = 0

    while True: 
        print(gSensor.angle())
        if abs(gSensor.angle() - curDegree) >= abs(degree):
            mobile_car.stop()
            break

        mobile_car.drive(0, 50 * sign)
        wait(5)

# Initialize
ev3 = EV3Brick()
err = 0
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
gSensor = GyroSensor(Port.S1)
gSensor.reset_angle(0)
# cSensor = ColorSensor(Port.S2)
Goals = [Color.RED, Color.BLACK, Color.BLUE]
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=120)

# Start alert
ev3.speaker.beep()

# Check the colors in order
for c in Goals:
    err = Check(c, err)
    # print(Goals.index(c))

# while err != 0:
#     # Todo : find ball
    
#     err = Sort(color, err)

ev3.speaker.beep()
