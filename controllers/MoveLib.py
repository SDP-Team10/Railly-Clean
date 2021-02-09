#!/usr/bin/env python3

#"""default_controller controller."""
#Standard python libraries import
from numpy import inf
from controller import Robot
from controller import Keyboard
import math

MAX_SPEED = 0

BASE_SPEED = 6.5 # maximum velocity of the turtlebot motors
TURN_MULT = 0.25 # constant to slow down turn speed
MOVE_MULT = 0.6 # constant to slow down move speed

gRobot = None
gLeftMotor = None
gRightMotor = None

def setTurnMult(tm=0.15):
    TURN_MULT=tm
    return True
def setMoveMult(mm=0.6):
    MOVE_MULT=mm
    return True

### Library setup for provided robot
def Setup(robot):
    global gRobot
    global gLeftMotor
    global gRightMotor
    global MAX_SPEED
    timestep = int(robot.getBasicTimeStep())
    gRobot = robot
    gLeftMotor = gRobot.getDevice('wheel_left_joint')
    gRightMotor = gRobot.getDevice('wheel_right_joint')
    if (gLeftMotor==None)or(gRightMotor==None):
        print('Cant find left or right motors: %s and %s'%('wheel_left_joint','wheel_right_joint'))
        return False
    MAX_SPEED = gLeftMotor.getMaxVelocity()
    enableSpeedControl(gLeftMotor)
    enableSpeedControl(gRightMotor)
    return True

def getLeftSpeed():
    if gRobot==None:
        print('No robot object given to MoveLib, call Setup(robot)')
        raise KeyError
    return gLeftMotor.getVelocity()
def getRightSpeed():
    if gRobot==None:
        print('No robot object given to MoveLib, call Setup(robot)')
        raise KeyError
    return gRightMotor.getVelocity()

def TurnRight(leftVel=-1,rightVel=-1):
    if leftVel==-1:
        leftVel=MAX_SPEED
    if rightVel==-1:
        rightVel=-MAX_SPEED
    limitVel(leftVel*TURN_MULT,rightVel*TURN_MULT)
def TurnLeft(leftVel=-1,rightVel=-1):
    if rightVel==-1:
        rightVel=MAX_SPEED
    if leftVel==-1:
        leftVel=-MAX_SPEED
    limitVel(leftVel*TURN_MULT,rightVel*TURN_MULT)
def MoveForward(speed=-1):
    if speed==-1:
        speed=MAX_SPEED
    limitVel(speed*MOVE_MULT,speed*MOVE_MULT)
def MoveBack(speed=-1):
    if speed==-1:
        speed=MAX_SPEED
    limitVel(-speed*MOVE_MULT,-speed*MOVE_MULT)
def Stop():
    enableSpeedControl(gLeftMotor)
    enableSpeedControl(gRightMotor)

# static parameters
wheel_radius = 0.0985
distance_between_wheels = 0.404
degToRad = math.pi/180
wheelCirc = 2*math.pi*wheel_radius

rotTimer=0

def CheckRotation():
    global rotTimer
    if rotTimer>0:
        rotTimer-=1/gRobot.getBasicTimeStep()
        if rotTimer<=0:
            print('STOPPING')
            Stop()
            rotTimer=0

def TurnAngle(angle=90):
    global rotTimer
    if rotTimer==0:
        print('TURNING')
        rads = abs(angle)*degToRad*2 ###
        gLeftMotor.setPosition(0)
        gRightMotor.setPosition(0)
        sectorLen = rads*distance_between_wheels/2
        revs = sectorLen/(2*math.pi*wheel_radius)
        pos = revs*2*math.pi
        turnSign = angle/abs(angle)
        if turnSign==-1:
            gLeftMotor.setPosition(0)
            gRightMotor.setPosition(pos)
            limitVel(0,MAX_SPEED*TURN_MULT)
        else:
            gLeftMotor.setPosition(pos)
            gRightMotor.setPosition(0)
            limitVel(MAX_SPEED*TURN_MULT,0)
        rotTimer=5 ### TEMP
        return True
    else:
        return False



def enableSpeedControl(motor, initVelocity = 0.):
    """Disable position control and set velocity of the motor to initVelocity (in rad / s)"""
    motor.setPosition(float('inf'))
    motor.setVelocity(initVelocity)

def limitVel(newLeftVel, newRightVel):
    if newLeftVel > gLeftMotor.getMaxVelocity(): newLeftVel=gLeftMotor.getMaxVelocity()
    if newRightVel > gRightMotor.getMaxVelocity(): newRightVel=gRightMotor.getMaxVelocity()
    if newLeftVel < -gLeftMotor.getMaxVelocity(): newLeftVel=-gLeftMotor.getMaxVelocity()
    if newRightVel < -gRightMotor.getMaxVelocity(): newRightVel=-gRightMotor.getMaxVelocity()
    gLeftMotor.setVelocity(newLeftVel)
    gRightMotor.setVelocity(newRightVel)
