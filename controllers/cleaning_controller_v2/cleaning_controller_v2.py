#!/usr/bin/env python3

import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import arm_controller_trash as ac
from libraries import bin_controller as bc
from libraries import move_lib_new_base as mc
from libraries import side_check as sc
from libraries import trash_classifier_wb as tc
from libraries import sticker_detection as vc
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1  # param
HEAD_WIDTH = 0.3  # param
BIN_LENGTH = 0.35  # param
CLEAN_ATTEMPTS = int(TABLE_WIDTH // HEAD_WIDTH)


class CleaningController(object):
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.front_camera = self.robot.getDevice("front_camera")
        self.side_camera = self.robot.getDevice("side_camera")
        self.front_camera.enable(self.time_step)
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

        self.arm_controller = ac.ArmController(self.robot)
        self.bin_controller = bc.BinController(self.robot)

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
    
    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False

    def check_valuable(self):  # NOT USED
        return

    # TODO next week: sweep and check again for reamining trash, max. 3 sweeps
    def clean_table(self, distance_to_wall, desired_x):
        mc.stop(self.robot)
        if tc.has_valuable(self.side_camera):
            return
        self.arm_controller.sweep(distance_to_wall, desired_x)
    
    def closer_to_table(self, to_wall, to_table):
        # move a bit out
        if abs(to_wall - to_table) < BIN_LENGTH + 0.03:
            move_dist = BIN_LENGTH + 0.03 - to_table
        # move closer
        else:
            move_dist = -(to_table - BIN_LENGTH + 0.05)
        mc.move_distance(self.robot, 'side', move_dist)

    # TODO use `sticker_detection`
    # IMPORTANT: centre before next row
    def centre(self):
        return

    def wall_in_front(self, turn_around):
        if self.distance_sensors[0].getValue() < STOP_THRESHOLD:  # check front sensor
            print("Detected wall in front")
            mc.stop(self.robot)
            if vc.is_carriage_end(self.front_camera):
                print("End of carriage detected")
                if turn_around:
                    mc.turn_angle(self.robot, 180)
                return True
        return False


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, bin_controller, dist_sensors = controller.robot, controller.bin_controller, controller.distance_sensors
    table_check, table_length = sc.SideCheck(robot), None
    table_detected, done_cleaning, left_side = False, False, True  # flags
    attempts = CLEAN_ATTEMPTS
    desired_x = -0.20

    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:
        # TODO next week
        # if not centred:
        #    centre()
        #    move_back()

        if done_cleaning:  # either completed cleaning both sides or bin is full
            if controller.wall_in_front(not left_side):  # turn around when on right side
                if left_side:
                    break
                    # TODO next week
                    # detect -> clean -> push button
                    # pass through door -> dock on the side -> wait to clear rubbish
                else:
                    left_side = True
            mc.move_forward(robot)

        elif not table_detected:  # still cleaning
            table_length, pole_length, distance_to_table = table_check.side_check(dist_sensors[2])
            
            if table_length:  # if not None or 0 -> table detected
                table_detected = True
                table_check.stop_scanning()
                table_length = table_length if table_length < 1 else 1  # TODO remove
                # print("Table Length:", table_length)
                print("To table:", distance_to_table)
                attempts = math.ceil(table_length / HEAD_WIDTH)
                distance = (table_length / 2) + pole_length
                mc.move_distance(robot, 'forward', -distance)  # to back edge of table
                # closer_to_table(dist_sensors[2].getValue(), distance_to_table)
                move_dist_to_table = distance_to_table - BIN_LENGTH
                mc.move_distance(robot, 'side', -move_dist_to_table)  # move bin closer to table
            
            elif controller.wall_in_front(True):  # turn around
                if left_side:
                    left_side = False  # start on right side after turning
                else:  # right side
                    done_cleaning = True  # completed both sides
                    left_side = True  # on left side after turning -> straight to carriage's end
            
            else:  # business as usual
                mc.move_forward(robot)
        
        else:
            bin_controller.open_bin()
            print("Attempts:", attempts)
            for i in range(attempts):
                print("Attempt #", i)
                controller.clean_table(table_check.params['DISTANCE_TO_WALL'], desired_x)
                if bin_controller.is_full():
                    done_cleaning = True
                    break
                if i < attempts-1:
                    mc.move_distance(robot, 'forward', HEAD_WIDTH)
            bin_controller.close_bin()
            mc.move_distance(robot, 'side', move_dist_to_table)
            table_check.done_cleaning()
            table_detected = False
            # TODO centre()  # beware speed: until next row of chairs

# Enter here exit cleanup code.
