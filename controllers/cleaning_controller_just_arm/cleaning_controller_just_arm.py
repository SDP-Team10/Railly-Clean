#!/usr/bin/env python3

import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import move_lib_step as mc
from libraries import arm_controller_trash as ac
from controller import Robot
from libraries import bin_controller as bc

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
        self.side_camera = self.robot.getDevice("side_camera")
        self.pressure_sensor = self.robot.getDevice("head_touch_sensor")
        if self.side_camera.hasRecognition():
            self.side_camera.enable(self.time_step)
            self.side_camera.recognitionEnable(self.time_step)
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
            "sec_2_motor",
            "sec_3_motor",
            "head_motor"
        ]
        self.arm_motors = self.init_arm()
        # Arm motors' position sensors?
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

    def clean_table(self, distance_to_wall, height):
        self.arm_controller.sweep(distance_to_wall,height)

    def press_button(self, height, length, distance_to_door):
        self.arm_controller.set_button_click(height, length, distance_to_door)

    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    attempts = CLEAN_ATTEMPTS
    bin_controller = bc.BinController(robot)
    bin_controller.close_bin()
    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:
        controller.clean_table(1.45, -0.2)