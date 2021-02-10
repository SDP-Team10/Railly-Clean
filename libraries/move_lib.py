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
MOVE_MULT = 0.25 # constant to slow down move speed

grobot = None
gleft_motor = None
gright_motor = None

def set_turn_mult(tm=0.15):
    TURN_MULT=tm
    return True
def set_move_mult(mm=0.6):
    MOVE_MULT=mm
    return True

### Library setup for provided robot
def setup(robot):
    global grobot
    global gleft_motor
    global gright_motor
    global MAX_SPEED
    timestep = int(robot.getBasicTimeStep())
    grobot = robot
    gleft_motor = grobot.getDevice('wheel_left_joint')
    gright_motor = grobot.getDevice('wheel_right_joint')
    if (gleft_motor==None)or(gright_motor==None):
        print('Cant find left or right motors: %s and %s'%('wheel_left_joint','wheel_right_joint'))
        return False
    MAX_SPEED = gleft_motor.getMaxVelocity()
    enable_speed_control(gleft_motor)
    enable_speed_control(gright_motor)
    return True

def get_left_speed(robot):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    return gleft_motor.getVelocity()
def get_right_speed(robot):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    return gright_motor.getVelocity()

def turn_right(robot,leftVel=-1,rightVel=-1):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    if leftVel==-1:
        leftVel=MAX_SPEED
    if rightVel==-1:
        rightVel=-MAX_SPEED
    limit_vel(leftVel*TURN_MULT,rightVel*TURN_MULT)
def turn_left(robot,leftVel=-1,rightVel=-1):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    if rightVel==-1:
        rightVel=MAX_SPEED
    if leftVel==-1:
        leftVel=-MAX_SPEED
    limit_vel(leftVel*TURN_MULT,rightVel*TURN_MULT)
def move_forward(robot,speed=-1):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    if speed==-1:
        speed=MAX_SPEED
    limit_vel(speed*MOVE_MULT,speed*MOVE_MULT)
def move_back(robot,speed=-1):
    if robot!=grobot or grobot==None:
        if not Setup(robot):
            print('Error in setting up robot')
            return False
    if speed==-1:
        speed=MAX_SPEED
    limit_vel(-speed*MOVE_MULT,-speed*MOVE_MULT)
def stop(robot):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    enable_speed_control(gleft_motor)
    enable_speed_control(gright_motor)

# static parameters
wheel_radius = 0.0985
distance_between_wheels = 0.404
degToRad = math.pi/180
wheelCirc = 2*math.pi*wheel_radius

rotTimer=0

def check_move(robot):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    global rotTimer
    if rotTimer>0:
        rotTimer-=1/grobot.getBasicTimeStep()
        if rotTimer<=0:
            print('STOPPING')
            stop(robot)
            rotTimer=0

def turn_angle(robot,angle=90):
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    global rotTimer
    if rotTimer==0:
        print('TURNING')
        rads = abs(angle)*degToRad*2 ###
        gleft_motor.setPosition(0)
        gright_motor.setPosition(0)
        sectorLen = rads*distance_between_wheels/2
        revs = sectorLen/(2*math.pi*wheel_radius)
        pos = revs*2*math.pi
        turnSign = angle/abs(angle)
        if turnSign==-1:
            gleft_motor.setPosition(0)
            gright_motor.setPosition(pos)
            limit_vel(0,MAX_SPEED*TURN_MULT)
        else:
            gleft_motor.setPosition(pos)
            gright_motor.setPosition(0)
            limit_vel(MAX_SPEED*TURN_MULT,0)
        rotTimer=5 ### TEMP
        return True
    else:
        return False

def move_distance(robot,dist=1):
    if dist <= 0:
        return False
    if robot!=grobot or grobot==None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    global rotTimer
    if rotTimer==0:
        print('MOVING')
        gleft_motor.setPosition(0)
        gright_motor.setPosition(0)
        pos = dist/wheel_radius
        gleft_motor.setPosition(pos)
        gright_motor.setPosition(pos)
        limit_vel(MAX_SPEED*MOVE_MULT,MAX_SPEED*MOVE_MULT)
        rotTimer=5 ### TEMP
        return True
    else:
        return False

def enable_speed_control(motor, initVelocity = 0.):
    """Disable position control and set velocity of the motor to initVelocity (in rad / s)"""
    motor.setPosition(float('inf'))
    motor.setVelocity(initVelocity)

def limit_vel(newLeftVel, newRightVel):
    if newLeftVel > gleft_motor.getMaxVelocity(): newLeftVel=gleft_motor.getMaxVelocity()
    if newRightVel > gright_motor.getMaxVelocity(): newRightVel=gright_motor.getMaxVelocity()
    if newLeftVel < -gleft_motor.getMaxVelocity(): newLeftVel=-gleft_motor.getMaxVelocity()
    if newRightVel < -gright_motor.getMaxVelocity(): newRightVel=-gright_motor.getMaxVelocity()
    gleft_motor.setVelocity(newLeftVel)
    gright_motor.setVelocity(newRightVel)
