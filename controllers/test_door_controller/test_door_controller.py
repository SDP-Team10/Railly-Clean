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
button_sensor = robot.getDevice("button_sensor")

button_sensor.enable(1) #sets the sampling period

while robot.step(timestep) != -1:
    button_value = button_sensor.getValue()
    if (button_value == 1):
        slide_motor.setPosition(1.1)
        pre = robot.getTime()
        
        post = robot.getTime()
        while (post - pre < door_interval):
            print("post - pre", post -pre)
            post = robot.getTime()
            slide_motor.setPosition(1.1)
            
        slide_motor.setPosition(0)
       

    pass

