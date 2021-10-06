#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from controller import P_controller

# Initialize
ev3 = EV3Brick()
cSensor_reflect = ColorSensor(Port.S2)
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=104)
p_ctrl = P_controller(proportion)
threshold = 15
proportion = 2
motor_power = 10

# Start alert
ev3.speaker.beep()

while cSensor_reflect.color() != Color.RED:
    error = threshold - cSensor_reflect.reflection()
    proportional_gain = p_ctrl.control(error)
    robot.drive(motor_power, proportional_gain)
    wait(10)

# End alert
ev3.speaker.beep(1000, 500)