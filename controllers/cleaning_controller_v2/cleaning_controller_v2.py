#!/usr/bin/env python3

import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import arm_controller_trash as ac
from libraries import bin_controller as bc
from libraries import button_detection as bd
from libraries import move_lib_new_base as mc
from libraries import side_check as sc
from libraries import trash_classifier_wb as tc
from libraries import sticker_detection as vc
from controller import Robot
import cv2


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
        self.camera_resolution = (128,128)
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

    # TODO next week: sweep and check again for remaining trash, max. 3 sweeps
    def clean_table(self, dist_to_wall):
        desired_x = -0.20
        mc.stop(self.robot)
        if tc.has_valuable(self.side_camera):
            return
        self.bin_controller.open_bin()
        self.arm_controller.sweep(dist_to_wall, desired_x)
    
    def closer_to_table(self, to_wall, to_table):
        # move a bit out
        if abs(to_wall - to_table) < BIN_LENGTH + 0.02:
            move_dist = BIN_LENGTH + 0.02 - to_table
        # move closer
        else:
            move_dist = -(to_table - BIN_LENGTH + 0.2)
        mc.move_distance(self.robot, 'side', move_dist)

    def work_on_button(self):
        counter = 0
        while self.robot.step(self.timestep) != -1:
            dist = self.distance_sensors[0].getValue()
            self.front_camera.saveImage("bttn_img.png", 100)
            image = cv2.imread("bttn_img.png")
            hfov = self.front_camera.getFov()
            x, y, z = bd.button_match(image, hfov, dist, self.camera_resolution)
            print(x, y, z)
            self.arm_controller.set_button_click(0.42, -x , -0.12-y)
            print("after set_button_click")
            mc.move_distance(self.robot, 'forward', z-0.14)
            if counter == 21:
                self.arm_controller.tuck_in_action()
            counter+=1

    def wall_in_front(self, turn_around, stop_dist=STOP_THRESHOLD):
        if self.distance_sensors[0].getValue() < stop_dist:  # check front sensor
            print("Detected wall in front")
            mc.stop(self.robot)
            if vc.is_carriage_end(self.front_camera):
                print("End of carriage detected")
                if turn_around:
                    mc.turn_angle(self.robot, 180)
                return True
        return False


if __name__ == "__main__":
    controller = CleaningController()
    robot, bin_controller, dist_sensors = controller.robot, controller.bin_controller, controller.distance_sensors
    table_check, table_length, distance_to_wall = sc.SideCheck(robot), None, None
    table_detected, done_cleaning, left_side = False, False, True  # flags
    attempts = CLEAN_ATTEMPTS

    while robot.step(controller.time_step) != -1:
        if done_cleaning:  # either completed cleaning both sides or bin is full
            stop_dist = STOP_THRESHOLD if not left_side else 1.5
            if controller.wall_in_front(not left_side, stop_dist):  # turn around when on right side
                if left_side:
                    controller.work_on_button() # detect -> clean -> push button -> pass through door
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

                # closer_to_table(table_check.params['DISTANCE_TO_WALL'], distance_to_table)
                move_dist_to_table = distance_to_table - (BIN_LENGTH - 0.05)
                mc.move_distance(robot, 'side', -move_dist_to_table)  # move bin closer to table
                distance_to_wall = dist_sensors[2].getValue()
            
            elif controller.wall_in_front(True):  # turn around
                if left_side:
                    left_side = False  # start on right side after turning
                else:  # right side
                    done_cleaning = True  # completed both sides
                    left_side = True  # on left side after turning -> straight to carriage's end
            
            else:  # business as usual
                mc.move_forward(robot)
        
        else:
            print("Attempts:", attempts)
            for i in range(attempts):
                print("Attempt #", i)
                # controller.clean_table(table_check.params['DISTANCE_TO_WALL'], desired_x)
                controller.clean_table(distance_to_wall)
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
