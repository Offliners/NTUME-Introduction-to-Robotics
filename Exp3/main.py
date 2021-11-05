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
    forward = 75

    GoStraight(forward, -90)
    Turn(-90)

    c = colorCorrect(cSensor.color())
    ballInfo[index][2] = c
    ballInfo[index][3] = index
    if ballInfo[index][0] == c:
        ballInfo[index][4] = True
    
    GoStraight(-forward, -90)
    Turn(0)
    
    return ballInfo

def GoStraight(distance, degree):
    mobile_car.reset()
    
    gain = 1.5
    err = 0
    sum_err = 0
    previous_err = 0
    dt = 5
    ctrl = PID_controller(2, 0.01, 5, dt)
    if distance > 0:
        while mobile_car.distance() < distance:
            err = -1 * degree - gSensor.angle()
            gain = ctrl.control(err, sum_err, previous_err)
            previous_err = err
            sum_err += err
            mobile_car.drive(100, -gain)
            wait(dt)
    else:
        while mobile_car.distance() > distance:
            err = -1 * degree - gSensor.angle()
            gain = ctrl.control(err, sum_err, previous_err)
            previous_err = err
            sum_err += err
            mobile_car.drive(-100, -gain)
            wait(dt)
    
    mobile_car.stop()



def Turn(degree):
    if degree >= 0:
        sign = 1
    else:
        sign = -1

    while True:
        if sign * gSensor.angle() <=  degree:
            gain = 1.5
            err = 0
            sum_err = 0
            previous_err = 0
            dt = 5
            ctrl = PID_controller(2, 0.01, 5, dt)
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
        wait(10)

def GriporThrowBall(select):
    Turn(-90)
    forward = 65
    GoStraight(forward, -90)
    Turn(-90)
    if select == True:
        gripperMotor.run_angle(50, 70)
    else:
        gripperMotor.run_angle(-50, 70)

    GoStraight(-forward, -90)
    Turn(0)

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
ballInfo = [[Color.RED, 225, None, 3, False], [Color.BLACK, 210, None, 3, False], [Color.BLUE, 210, None, 3, False]]
mobile_car = DriveBase(Lmotor, Rmotor, wheel_diameter=55.5, axle_track=104)
mobile_car.reset()

# Start alert
ev3.speaker.beep()

for i in range(3):
    Lmotor.reset_angle(0)
    Rmotor.reset_angle(0)
    GoStraight(ballInfo[i][1], 0)
    ballInfo = Check(ballInfo, i)
    print(ballInfo)

Lmotor.reset_angle(0)
Rmotor.reset_angle(0)
dis = 0
blackLoc = None
for c in reversed(ballInfo):
    if c[2] != Color.BLACK:
        dis += c[1]
    else:
        blackLoc = c[0]
        if dis != 0:
            GoStraight(-dis, 0)
        GriporThrowBall(True)
        GoStraight(dis + 10, 0)
        gripperMotor.run_angle(-50, 70)
        break

Turn(0)
GoStraight(-15, 0)
wait(10)

Lmotor.reset_angle(0)
Rmotor.reset_angle(0)
if ballInfo[1][4]:
    if ballInfo[0][4] and ballInfo[2][4]: # Red - Black - Blue
        print('Done')
    else: # Blue - Black - Red
        GriporThrowBall(True)
        GoStraight(-ballInfo[1][1], 0)
        GriporThrowBall(False)
        GoStraight(-ballInfo[0][1] + 30, 0)
        GriporThrowBall(True)
        GoStraight(ballInfo[0][1] + ballInfo[1][1], 0)
        GriporThrowBall(False)
        GoStraight(-ballInfo[1][1], 0)
        GriporThrowBall(True)
        GoStraight(-ballInfo[0][1] + 30, 0)
        GriporThrowBall(False)
else: 
    if blackLoc == Color.RED:
        dis = 0
        for c in reversed(ballInfo):
            if c[2] != blackLoc:
                dis += c[1]
            else:
                GoStraight(-dis, 0)
                GriporThrowBall(True)
                break

        if ballInfo[2][4]: # Black - Red - Blue
            GoStraight(-ballInfo[1][1], 0)
            GriporThrowBall(False)
        else: # Black - Blue - Red
            GoStraight(-ballInfo[1][1] - ballInfo[2][1], 0)
            GriporThrowBall(False)
            GoStraight(ballInfo[1][1], 0)
            GriporThrowBall(True)
            GoStraight(ballInfo[2][1], 0)
            GriporThrowBall(False)
    else:
        dis = -10
        for c in reversed(ballInfo):
            if c[2] != blackLoc:
                dis += c[1]
            else:
                GoStraight(-dis, 0)
                GriporThrowBall(True)
                break
        
        if ballInfo[0][4]: # Red - Blue - Black
            GoStraight(ballInfo[2][1], 0)
            GriporThrowBall(False)
        else: # Blue - Red - Black
            GoStraight(ballInfo[2][1] + ballInfo[1][1], 0)
            GriporThrowBall(False)
            GoStraight(-ballInfo[2][1] + 10, 0)
            GriporThrowBall(True)
            GoStraight(-ballInfo[1][1], 0)
            GriporThrowBall(False)

ev3.speaker.beep()
