#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class PID_controller:
    def __init__(self, P, I, D, timestep):
        self.P = P
        self.I = I
        self.D = D
        self.dt = timestep

    def control(self, error, sum_error, previous_error):
        integral = (sum_error + error) * self.dt
        derivative = (error - previous_error) / self.dt
        return self.P * error + self.I * integral + self.D * derivative

def colorCorrect(color):
    if color == Color.BLACK or color == None:
        return Color.BLACK
    elif color == Color.BLUE or color == Color.GREEN:
        return Color.BLUE
    else:
        return Color.RED

def Check(ballInfo, index):
    Turn(-90)
    wait(100)
    forward = 110

    GoStraight(forward, -90)
    wait(100)
    Turn(-90)
    wait(200)

    c = colorCorrect(cSensor.color())
    ballInfo[index][2] = c
    ballInfo[index][3] = index
    if ballInfo[index][0] == c:
        ballInfo[index][4] = True
    
    GoStraight(-forward, -90)
    wait(100)
    Turn(0)
    wait(100)
    
    return ballInfo

def GoStraight(distance, degree):
    mobile_car.reset()
    
    gain = 1.1
    err = 0
    sum_err = 0
    previous_err = 0
    dt = 5
    ctrl = PID_controller(2, 0.01, 10, dt)
    if distance > 0:
        while mobile_car.distance() < distance:
            print(gSensor.angle(), -1 * degree)
            err = -1 * degree - gSensor.angle()
            gain = ctrl.control(err, sum_err, previous_err)
            previous_err = err
            sum_err += err
            mobile_car.drive(100, -gain)
            wait(dt)
    else:
        while mobile_car.distance() > distance:
            print(gSensor.angle(), -1 * degree)
            err = -1 * degree - gSensor.angle()
            gain = ctrl.control(err, sum_err, previous_err)
            previous_err = err
            sum_err += err
            mobile_car.drive(-100, -gain)
            wait(dt)


def Turn(degree):
    if degree >= 0:
        sign = 1
    else:
        sign = -1

    while True:
        if sign * gSensor.angle() <=  degree:
            gain = 1.1
            err = 0
            sum_err = 0
            previous_err = 0
            dt = 10
            ctrl = PID_controller(2, 0.01, 25, dt)
            while True:
                if sign * gSensor.angle() == degree:
                    mobile_car.stop()
                    break
                err = gSensor.angle() - sign * degree
                gain += ctrl.control(err, sum_err, previous_err)
                previous_err = err
                sum_err += err
                mobile_car.drive(0, gain)
                wait(dt)
            
            break

        mobile_car.drive(0, 30 * sign)

# Initialize
ev3 = EV3Brick()
Lmotor = Motor(Port.C)
Rmotor = Motor(Port.B)
gripperMotor = Motor(Port.D)
gripperMotor.reset_angle(0)
gSensor = GyroSensor(Port.S1)
gSensor.reset_angle(0)
cSensor = ColorSensor(Port.S2)

# [correct ball color, distance, current ball color, found order, is the color correct]
ballInfo = [[Color.RED, 230, None, 3, False], [Color.BLACK, 210, None, 3, False], [Color.BLUE, 210, None, 3, False]]
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=110)
mobile_car.reset()

# Start alert
ev3.speaker.beep()

for i in range(3):
    Lmotor.reset_angle(0)
    Rmotor.reset_angle(0)
    GoStraight(ballInfo[i][1], 0)
    ballInfo = Check(ballInfo, i)
    print(ballInfo)

dis = 0
for c in reversed(ballInfo):
    if c[2] != Color.BLACK:
        dis += c[1]
    else:
        GoStraight(-dis, 0)
        Turn(-90)
        wait(100)
        forward = 100
        GoStraight(forward, -90)
        Turn(-90)
        wait(100)
        gripperMotor.run_angle(-50, 30)
        GoStraight(-forward, -90)
        Turn(0)
        wait(100)
        GoStraight(dis, 0)
        gripperMotor.run_angle(50, 30)
        break

Turn(0)
GoStraight(-660, 0)

if ballInfo[1][4]:
    if ballInfo[0][4] and ballInfo[2][4]:
        print('Done')
    else:
        pass
        # Todo
else:
    pass
    # Todo

ev3.speaker.beep()
