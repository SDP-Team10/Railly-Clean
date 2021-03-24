"""button_image_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import os
import sys
sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import button_detection as bd
from libraries import arm_controller_trash as arm_controller
from libraries import move_lib_new_base as mc
from controller import Robot
import cv2


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1  # param
HEAD_WIDTH = 0.3  # param
BIN_LENGTH = 0.35  # param
CLEAN_ATTEMPTS = int(TABLE_WIDTH // HEAD_WIDTH)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
cam = robot.getDevice("front_camera")
cam.enable(timestep)
ac = arm_controller.ArmController(robot)
front_sensor = robot.getDevice("front distance sensor")
front_sensor.enable(timestep)
cam_res = (128,128)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

counter = 0
found = False
while robot.step(timestep) != -1:
    dist = front_sensor.getValue()
    if counter == 20:
        cam.saveImage("bttn_img.png", 100)
        image = cv2.imread("bttn_img.png")
        hfov = cam.getFov()
        x, y, z = bd.button_match(image, hfov, dist, cam_res)
        print(x, y, z)
        ac.set_button_click(0.42, -x , -0.12-y)
        print("after set_button_click")
        mc.move_distance(robot, 'forward', z-0.14)
    if counter == 21:
        ac.tuck_in_action()

    counter+=1
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
