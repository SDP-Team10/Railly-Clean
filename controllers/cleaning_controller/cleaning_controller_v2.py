#!/usr/bin/env python3

import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
# from libraries import classify_trash
from libraries import move_lib_step as mc
from libraries import arm_controller as ac  # arm_controller_trash
from libraries import bin_controller as bc
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1  # parameter
HEAD_WIDTH = 0.3  # parameter
CLEAN_ATTEMPTS = int(TABLE_WIDTH // HEAD_WIDTH)
BIN_LENGTH = 0.34


class CleaningController(object):
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.front_camera = self.robot.getDevice("front_camera")
        self.front_camera.enable(self.time_step)
        # TODO get and enable side camera
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
        self.bin_controller = bc.BinController(self.robot)

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
    
    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False

    def check_valuable(self):
        # take image with side camera
        return  # classifier.classify_valuable(img)

    def clean_table(self, distance_to_wall):
        # TODO
        #  if self.check_valuable():
        #      return
        #  self.bin_controller.open_bin()
        mc.stop(self.robot)
        self.arm_controller.sweep(distance_to_wall)
        #  self.bin_controller.close_bin()
    
    # TODO
    def wall_in_front(self, turn_around):
        if self.distance_sensors[0].getValue() < STOP_THRESHOLD:  # check front sensor
            print("Detected wall in front")
            mc.stop(self.robot)
            # if vc.is_carriage_end(controller.front_camera):
            #     print("End of carriage detected")
            if turn_around:
                mc.turn_angle(self.robot, 180)
                return True
            else:
                # 
        return False


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, bin_controller, dist_sensors = controller.robot, controller.bin_controller, controller.distance_sensors
    table_check, table_length = sc.SideCheck(robot), None
    table_detected, left_side, done_cleaning = False, True, False  # flags
    attempts = CLEAN_ATTEMPTS

    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:
        if done_cleaning:
            mc.move_forward(robot)
            if controller.wall_in_front(False):
                # detect, clean, push button
                # move to the side and dock
                # wait for rubbish to be cleared

        elif not table_detected:
            print(dist_sensors[0].getValue())
            table_length, pole_length, distance_to_table = table_check.side_check(dist_sensors[2])
            
            if table_length:  # if not None or 0 -> table detected
                print("Table detected")
                table_detected = True
                table_check.stop_scanning()
                table_length = table_length if table_length < 1 else 1  # TODO remove
                print("Table Length:", table_length)
                attempts = math.ceil(table_length / HEAD_WIDTH)
                print("TOTAL ATTEMPTS:", attempts)
                distance = (table_length / 2) + pole_length
                mc.move_distance(robot, -distance)  # to back edge of table
                move_dist_to_table = distance_to_table - BIN_LENGTH
                # TODO
                #  mc.turn_angle(robot, -90)
                mc.move_left_distance(robot, move_dist_to_table)  # move bin closer to table
                #  mc.turn_angle(robot, 90)
            
            controller.wall_in_front(left_side)
            
            else:  # business as usual
                print("Moving forward")
                mc.move_forward(robot)
        
        else:
            print("Attempts:", attempts)
            for i in range(attempts):
                print("Attempt #", i)
                controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
                    if bin_controller.is_full():
                        done_cleaning = True
                        break
                if i < attempts-1:
                    mc.move_distance(robot, HEAD_WIDTH)
            bin_controller.close_bin()
            mc.move_right_distance(robot, move_dist_to_table)
            table_check.done_cleaning()
            table_detected = False
            move_while_centering()  # beware speed: until next row of chairs

# Enter here exit cleanup code.
