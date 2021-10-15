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

    if cSensor.color() == color:
        Back(color)
    else:
        err += 1
        Gripper.run_target(100, 90)
        Back(color)
        Gripper.run_target(-100, 90)
    
    return err

def Forward(color):
    mobile_car.turn(100, 180)
    dis = 150 + Goal[color] * 100
    mobile_car.straight(dis)
    mobile_car.turn(100, 90)
    mobile_car.straight(100)

def Back(color):
    mobile_car.turn(100, 180)
    mobile_car.straight(100)
    mobile_car.turn(-100, 90)
    dis = 150 + Goal[color] * 100
    mobile_car.straight(dis)

def Sort(color, err):
    Gripper.run_target(100, 90)
    Forward(color)
    Gripper.run_target(-100, 90)
    Back(color)
    err -= 1

    return err


# Initialize
ev3 = EV3Brick()
err = 0
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
Gripper = Motor(Port.D)
cSensor = ColorSensor(Port.S2)
Goals = {Color.RED:0, Color.BLACK:1, Color.BLUE:2}
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=104)

# Start alert
ev3.speaker.beep()

# Check the colors in order
for c in Goals.items():
    err = Check(c[0], err)

while err != 0:
    # Todo : find ball
    
    err = Sort(color, err)



# Write your program here.
ev3.speaker.beep()
