#!/usr/bin/env python3

"""default_controller controller."""
import cv2
import numpy as np

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Keyboard, Robot

# Standard python libraries import
from numpy import inf

import visionController as vc

TIME_STEP = 64  # default time step in ms
BASE_SPEED = 6.5  # maximum velocity of the turtlebot motors
TURN_MULT = 0.15  # constant to slow down turn speed
MOVE_MULT = 0.6  # constant to slow down move speed
TURN_LIMIT = 90  # constant to vary when to fix movement


### Helper functions to initialise the turtlebot motor devices ###
def initRobot():
    """Create a robot instance, retrieve the  """
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    camera = robot.getDevice("front_camera")
    camera.enable(timestep)
    dsNames = [
        "front distance sensor",
        "back distance sensor",
        "right distance sensor",
        "left distance sensor",
    ]
    distanceSensors = initDist(robot, 4, dsNames, timestep)
    leftMotor = robot.getDevice("wheel_left_joint")
    rightMotor = robot.getDevice("wheel_right_joint")
    return robot, leftMotor, rightMotor, camera, distanceSensors


# Distance sensor initilazation


def initDist(robot, dsNum, dsNames, timestep):
    """ function initDist
    :param robot: webots robot object
    :param dsNum: number of sensors on the robot
    :param dsName: names of the 'physical' distance sensor on the webots robot
    :param timestep: Time step in the webots controller file
    :return: list of enabled distance sensor objects
    """
    distanceSensors = []
    for i in range(dsNum):
        distanceSensors.append(robot.getDevice(dsNames[i]))
        distanceSensors[i].enable(timestep)
    return distanceSensors


# Checking for obstacles


def shouldStop(distanceSensor, threshold=0.6):
    """function shouldStop
    :param distanceSensor: distancesensor that needs to be checked for objects in the way
    :return: boolean if distance is <40cm, return true, otherwise false
    """
    if (
        distanceSensor.getValue() < threshold
    ):  # 0.4 is placeholder, need to set up "lookupTable field
        return True
    else:
        return False


def enableSpeedControl(motor, initVelocity=0.0):
    """Disable position control and set velocity of the motor to initVelocity (in rad / s)"""
    motor.setPosition(float("inf"))
    motor.setVelocity(initVelocity)


def limitVel(newLeftVel, newRightVel):
    if newLeftVel > leftMotor.getMaxVelocity():
        newLeftVel = leftMotor.getMaxVelocity()
    if newRightVel > rightMotor.getMaxVelocity():
        newRightVel = rightMotor.getMaxVelocity()
    if newLeftVel < -leftMotor.getMaxVelocity():
        newLeftVel = -leftMotor.getMaxVelocity()
    if newRightVel < -rightMotor.getMaxVelocity():
        newRightVel = -rightMotor.getMaxVelocity()
    leftMotor.setVelocity(newLeftVel)
    rightMotor.setVelocity(newRightVel)


def CheckDistanceSensors():
    return (distanceSensors[3].getValue(), distanceSensors[2].getValue())


def AdjustMovement(threshold=4):
    leftD, rightD = CheckDistanceSensors()
    newLeftVel = 0
    newRightVel = 0
    sign = (leftD - rightD) / abs(leftD - rightD)
    if (abs(leftD - rightD)) > 0.1:
        if sign == 1 and leftD < threshold:
            newLeftVel = -sign * (abs(leftD - rightD)) * BASE_SPEED * TURN_MULT
            print("Turning Left")
        if sign == -1 and rightD < threshold:
            newRightVel = sign * (abs(leftD - rightD)) * BASE_SPEED * TURN_MULT
            print("Turning Right")
    return (newLeftVel, newRightVel)


### Main ###

robot, leftMotor, rightMotor, camera, distanceSensors = initRobot()

enableSpeedControl(leftMotor)
enableSpeedControl(rightMotor)
inThreshold = False
detectedSticker = False
# visualizeLidarRotation()
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    newLeftVel = 0
    newRightVel = 0
    time = robot.getTime()
    for ds in distanceSensors:
        print(ds.getName(), ds.getValue())
    if shouldStop(distanceSensors[0]):
        inThreshold = True
    else:
        inThreshold = False
    print("inThreshold ", inThreshold)
    print("----")
    if not inThreshold and not detectedSticker:
        newLeftVel, newRightVel = AdjustMovement()
        newLeftVel += BASE_SPEED * MOVE_MULT
        newRightVel += BASE_SPEED * MOVE_MULT
        limitVel(newLeftVel, newRightVel)
    elif inThreshold and not detectedSticker:
        if vc.is_carriage_end(camera):
            detectedSticker = True
            newLeftVel = -BASE_SPEED * MOVE_MULT
            newRightVel = -BASE_SPEED * MOVE_MULT
            limitVel(newLeftVel, newRightVel)
    elif not inThreshold and detectedSticker:
        newLeftVel = 0
        newRightVel = 0
        limitVel(newLeftVel, newRightVel)
