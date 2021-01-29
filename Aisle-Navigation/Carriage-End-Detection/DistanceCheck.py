# Distance sensor helper functions for railly Clean
from controller import Robot, DistanceSensor

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

#Checking for obstacles
def shouldStop(distanceSensor):
    """function shouldStop
    :param distanceSensor: distancesensor that needs to be checked for objects in the way
    :return: boolean if distance is <40cm, return true, otherwise false
    """
    if(distanceSensor.getValue<0.1): #Threshold value, based on lookup table in Distance Sensor node
        return True
    else:
        return False
