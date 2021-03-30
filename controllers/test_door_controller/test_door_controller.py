"""test_door_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
door_interval = 5

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

slide_motor = robot.getDevice("slide_motor")
button_sensor1 = robot.getDevice("button_sensor")
button_sensor2 = robot.getDevice("button_sensor2")

button_sensor1.enable(1) #sets the sampling period
button_sensor2.enable(1) #sets the sampling period

while robot.step(timestep) != -1:
    button_value1 = button_sensor1.getValue()
    button_value2 = button_sensor2.getValue()
    if (button_value1 == 1) or (button_value2 == 1):
        slide_motor.setPosition(1.1)
    pass

