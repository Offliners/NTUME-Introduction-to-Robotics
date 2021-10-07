#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

class P_controller:
    def __init__(self, P):
        self.P = P

    def control(self, error):
        return self.P * error

class PD_controller:
    def __init__(self, P, D, timestep):
        self.P = P
        self.D = D
        self.dt = timestep

    def control(self, error, previous_error):
        derivative = (error - previous_error) / self.dt
        return self.k * error + self.D * derivative

class PI_controller:
    def __init__(self, P, I, timestep):
        self.P = P
        self.I = I
        self.dt = timestep

    def control(self, error, sum_error):
        integral = (sum_error + error) * self.dt
        return self.k * error + self.I * integral

class PID_controller:
    def __init__(self, P, I, D, timestep):
        self.P = P
        self.I = I
        self.D = D
        self.dt = timestep

    def control(self, error, sum_error, previous_error):
        integral = (sum_error + error) * self.dt
        derivative = (error - previous_error) / self.dt
        return self.k * error + self.I * integral + self.D * derivative

# Initialize
ev3 = EV3Brick()
cSensor_reflect = ColorSensor(Port.S2)
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=104)


# Hyperparameter
threshold = 15
proportion = 2
integral = 0.000025
derivative = 100
dt = 10
motor_power = 100
err = 0
previous_err = 0
sum_err = 0


# Choose controller
ctrl = P_controller(proportion)
# ctrl = PD_controller(proportion, derivative, dt)
# ctrl = PI_controller(proportion, integral, dt)
# ctrl = PID_controller(proportion, integral, derivative, dt)


# Start alert
ev3.speaker.beep()

while cSensor_reflect.color() != Color.RED:
    err = threshold - cSensor_reflect.reflection()
    gain = ctrl.control(err)
    # gain = ctrl.control(err, previous_err)
    # gain = ctrl.control(err, sum_err)
    # gain = ctrl.control(err, sum_err, previous_err)
    previous_err = err
    sum_err += err
    mobile_car.drive(motor_power, gain)
    wait(dt)


# End alert
ev3.speaker.beep(1000, 500)
