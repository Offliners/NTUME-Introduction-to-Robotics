#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

PI = 3.141592653589793238462643383279
X_diameter = 43.2 # mm
y_diameter = 56 # mm

def liftPen(select):
    if select:
        pen_motor.run_angle(50, 30)
    else:
        pen_motor.run_angle(50, -30)

def horizonMove(dx):
    rotation = dx / (PI * X_diameter / 10)
    horizontal_motor.run_angle(-100, 360 * rotation)
    horizontal_motor.stop()

def verticalMove(dy):
    rotation = dy / (PI * y_diameter / 10)
    vertical_motor.run_angle(100, 360 * rotation)
    vertical_motor.stop()

# Draw Sun
def drawSun(size=2):
    liftPen(False)
    horizonMove(2*size)
    verticalMove(3*size)
    liftPen(True)
    verticalMove(-3*size)
    horizonMove(-2*size)

    liftPen(False)
    verticalMove(3*size)

    liftPen(True)
    verticalMove(-1.5*size)

    liftPen(False)
    horizonMove(2*size)

    liftPen(True)
    horizonMove(-2*size)
    verticalMove(1.5*size)

    liftPen(False)
    horizonMove(2*size)

    liftPen(True)

# Draw Spring
def drawSpring():
    horizonMove(1)
    liftPen(False)
    horizonMove(3)

    liftPen(True)
    horizonMove(-2.5)
    verticalMove(0.75)

    liftPen(False)
    horizonMove(2)

    liftPen(True)
    horizonMove(-3.5)
    verticalMove(0.75)

    liftPen(False)
    horizonMove(5)

    liftPen(True)
    horizonMove(-2.5)
    verticalMove(-1.5)

    liftPen(False)
    
    horizontal_motor.run_angle(-75, 360 * (-3) / (PI * X_diameter / 10), Stop.COAST, False)
    vertical_motor.run_angle(100, 360 * 4 / (PI * y_diameter / 10), Stop.COAST, False)
    wait(1400)
    
    vertical_motor.stop()
    horizontal_motor.stop()
    
    liftPen(True)
    horizonMove(1)
    verticalMove(-3.5)

    while touch_sensor.pressed() != True:
        horizontal_motor.run(150)
    
    horizonMove(2)
    liftPen(False)
    horizontal_motor.run_angle(-75, 360 * 2 / (PI * X_diameter / 10), Stop.COAST, False)
    vertical_motor.run_angle(100, 360 * 3 / (PI * y_diameter / 10), Stop.COAST, False)
    wait(600)
    
    liftPen(True)
    horizonMove(-3)

    drawSun(1.2)




# Initialize
ev3 = EV3Brick()
pen_motor = Motor(Port.A)
horizontal_motor = Motor(Port.B)
vertical_motor = Motor(Port.C)
touch_sensor = TouchSensor(Port.S1)

pen_motor.reset_angle(0)
horizontal_motor.reset_angle(0)
vertical_motor.reset_angle(0)

# Set origin
while touch_sensor.pressed() != True:
    horizontal_motor.run(150)

horizontal_motor.stop()
wait(100)

# drawSun()
drawSpring()

# End 
ev3.speaker.beep()
