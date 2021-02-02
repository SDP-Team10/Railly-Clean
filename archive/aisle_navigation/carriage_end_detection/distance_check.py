# Distance sensor helper functions for railly Clean
from controller import Robot, DistanceSensor

# Distance sensor initilazation
def init_dist(robot, ds_names, timestep):
    """ function init_dist
    :param robot: webots robot object
    :param ds_name: names of the 'physical' distance sensor on the webots robot
    :param timestep: Time step in the webots controller file
    :return: list of enabled distance sensor objects
    """
    dist_sensors = []
    for name in ds_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        dist_sensors.append(sensor)
    return dist_sensors


# Checking for obstacles
def should_stop(distance_sensor, threshold=0.4):
    """function should_stop
    :param distance_sensor: distance sensor that needs to be checked for objects in the way
    :return: boolean if distance is < threshold return true, otherwise false
    """
    # Threshold value, based on lookup table in Distance Sensor node
    return distance_sensor.getValue < threshold
