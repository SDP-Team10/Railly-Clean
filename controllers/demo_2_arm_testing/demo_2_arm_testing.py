"""demo_2_arm_testing controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
import math



sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import move_lib_step as mc
from libraries import arm_controller as ac
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1  # parameter
HEAD_WIDTH = 0.3  # parameter
CLEAN_ATTEMPTS = int(TABLE_WIDTH // HEAD_WIDTH)

class CleaningController(object):
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("front_camera")
        self.camera.enable(self.time_step)
        self.ds_names = [
            "front distance sensor",
            "back distance sensor",
            "left distance sensor",
            "right distance sensor"
        ]
        self.distance_sensors = self.init_dist()
        self.am_names = [
            "base_motor",
            "sec_1_motor",
            "sec_1_motor",
            "head_motor"
        ]
        self.arm_motors = self.init_arm()
        # Arm motors' position sensors?
        self.left_motor = self.robot.getDevice("wheel_left_joint")
        self.right_motor = self.robot.getDevice("wheel_right_joint")
        self.arm_controller = ac.ArmController(self.robot)

    # Arm's motors
    def init_arm(self):
        arm_motors = []
        for name in self.am_names:
            motor = self.robot.getDevice(name)
            arm_motors.append(motor)
        return arm_motors

    # Distance sensor initialization
    def init_dist(self):
        """ function init_dist
        :return: list of enabled distance sensor objects
        """
        dist_sensors = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            dist_sensors.append(sensor)
        return dist_sensors

    def clean_table(self, distance_to_wall):
        mc.stop(self.robot)
        self.arm_controller.sweep(distance_to_wall)
    
    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False

# Testing here
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check, table_length, table_detected, left_side = sc.SideCheck(robot), None, False, True
    attempts = CLEAN_ATTEMPTS

    #Test statement
    times = 5*[0]    
    for i in range(1):
        pre = robot.getTime()
        #controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
        controller.clean_table(1.6)
        post = robot.getTime()
        elapsed = post - pre
        times[i] = elapsed
    print(times)
      
         
    

