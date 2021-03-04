#!/usr/bin/env python3

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
        ds = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            ds.append(sensor)
        return ds

    def clean_table(self, distance_to_wall):
        mc.stop(self.robot)
        self.arm_controller.sweep(distance_to_wall)
    
    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check, table_length, table_detected, left_side = sc.SideCheck(robot), None, False, True
    attempts = CLEAN_ATTEMPTS

    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:
        if not table_detected:
            print(dist_sensors[0].getValue())
            table_length, pole_length = table_check.side_check(dist_sensors[2])
            
            if table_length:  # if not None or 0 -> table detected
                print("Table detected")
                table_detected = True
                table_check.stop_scanning()
                table_length = table_length if table_length < 1 else 1  # TODO: remove
                print("Table Length:", table_length)
                attempts = math.ceil(table_length / HEAD_WIDTH)
                print("TOTAL ATTEMPTS:", attempts)
                distance = (table_length / 2) + pole_length
                mc.move_distance(robot, -distance)  # to back edge of table
                # TODO
                #  turn_angle(robot, -90)
                #  move_distance() given bin's length and distance to wall
                #  turn_angle(robot, 90)
                #  open_bin()
            
            elif dist_sensors[0].getValue() < STOP_THRESHOLD:  # check front distance sensor
                print("Detected wall in front")
                mc.stop(robot)
                # if vc.is_carriage_end(controller.camera):
                #     print("End of carriage detected")
                if left_side:
                    mc.turn_angle(robot, 180)
                    left_side = False
            
            else:  # business as usual
                print("Moving forward")
                mc.move_forward(robot)

        else:
            print("Attempts:", attempts)
            for i in range(attempts-1):
                print("Attempt #", i)
                controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
                mc.move_distance(robot, HEAD_WIDTH)
            controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
            table_check.done_cleaning()
            # TODO
            #  close_bin()
            #  if bin's full (pressure plate) -> set flag to stop cleaning (action TBD)
            #  else:
            #      turn_angle(90)
            #      move_distance() given bin's length and distance to wall
            #      turn_angle(-90)
            #      move_while_centering() until next row of chairs
            table_detected = False


# Enter here exit cleanup code.
