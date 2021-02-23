import sys
import os
import math
import traceback
import numpy as np


sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import side_check as sc
from libraries import move_lib_step as mc
from libraries import classify_trash as ct
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1  # parameter
HEAD_WIDTH = 0.3  # parameter
CLEAN_ATTEMPTS = math.floor(TABLE_WIDTH // HEAD_WIDTH)


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

    def clean_table(self):
        mc.stop(self.robot)
        self.arm_controller.sweep()
        # ac.wipe(self.robot)
    
    @staticmethod
    def next_row(table_check_l, table_check_r):
        table_check_l.done_cleaning()
        table_check_r.done_cleaning()
        return False, 'f'


# Controller assumes library functions handle their own time steps
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check_l, table_check_r = sc.SideCheck(robot), sc.SideCheck(robot)
    table_length_l, table_length_r = None, None
    table_detected, side = False, 'f'  # control flags

    # Assume robot is already centered
    while controller.robot.step(controller.time_step) != -1:
        try:
            img = controller.camera.getImage()
            print(type(img))
            image = np.frombuffer(img, np.uint8).reshape((controller.camera.getHeight(), controller.camera.getWidth(), 4))
            res = ct.classify_trash(image)
            print(res)
        except Exception as e:
            print(traceback.format_exc())
# Enter here exit cleanup code.
