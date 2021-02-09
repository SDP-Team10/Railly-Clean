#!/usr/bin/env python3

"""arm_control controller."""

import sys
import os

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
# to change accordingly
from libraries import robot_movement as mc
from libraries import arm_control as ac
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
CLEAN_ATTEMPTS = 3  # parameter for how many times it takes to cover a whole table
TABLE_WIDTH = 1


class CleaningController(object):
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("front_camera")
        self.camera.enable(self.time_step)
        self.ds_names = [
            "front distance sensor",
            "back distance sensor",
            "right distance sensor",
            "left distance sensor",
        ]
        self.distance_sensors = self.init_dist()
        self.am_names = [
            "base_motor",
            "sec_1_motor",
            "sec_1_motor",
            "rotational motor"
        ]
        self.arm_motors = self.init_arm()
        # arm motors' position sensors?
        self.left_motor = self.robot.getDevice("wheel_left_joint")
        self.right_motor = self.robot.getDevice("wheel_right_joint")

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

    def clean_table(self, swept, clean_step):
        # pass Robot instance or Motor instances?
        if not swept:
            mc.stop(self.robot)
            ac.sweep(self.robot)
            return True, clean_step
        else:
            mc.stop(self.robot)
            ac.wipe(self.robot)
            return False, (clean_step+1) % (CLEAN_ATTEMPTS*2)


# Main
if __name__ == "__main__":
    controller = CleaningController()
    robot = controller.robot()
    table_check = sc.SideCheck()

    table_detected = False
    swept = False
    clean_step = 0
    side = 'l'

    while controller.robot.step(controller.time_step) != -1:
        # assume already centered
        # better to work with an instance of side_check
        # global variables -> instance variables to be updated each time
        if not table_detected:
            if table_check.side_check(robot, controller.distance_sensors):
                # assume stop at pole, move to dge
                mc.move_backward(TABLE_WIDTH / 2)
                table_detected = True
            if vc.is_carriage_end(controller.camera):
                mc.stop(robot)
            else:
                mc.move_forward(robot)
        else:
            # left side first
            if clean_step < CLEAN_ATTEMPTS:
                swept, clean_step = controller.clean_table(swept, clean_step)
            elif clean_step == CLEAN_ATTEMPTS and side == 'l':
                ac.switch_side(robot)
                side = 'r'
            # right side after
            elif clean_step < CLEAN_ATTEMPTS*2 and side == 'r':
                swept, clean_step = controller.clean_table(swept, clean_step)
            else:
                table_detected = False
                side = 'l'


# Enter here exit cleanup code.
