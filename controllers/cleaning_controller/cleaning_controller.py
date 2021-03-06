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
        self.sticker_offset = 0
        self.sticker_image_offset = 0
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

    def check_camera(self):
        print(type(self.camera))

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


    def off_centre_value(self):
        try:
            field_of_view = self.camera.getFov()
            image_width = self.camera.getWidth()
            sticker_centroid = vc.centroid_detection(self.camera)
            cpa = vc.camera_point_angle(field_of_view, image_width,sticker_centroid)
            print(cpa)
            print(sticker_centroid[0]/image_width)
            self.sticker_image_offset = sticker_centroid[0]/image_width
            self.sticker_offset = cpa
        except:
            print(self.sticker_offset)


    def vision_centering(self):
        field_of_view = self.camera.getFov()
        image_width = self.camera.getWidth()
        print(field_of_view)
        print(image_width)

    @staticmethod
    def next_row(table_check):  # NOT USED
        table_check.done_cleaning()
        return False


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    controller.vision_centering()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check, table_length, table_detected, left_side = sc.SideCheck(robot), None, False, True
    attempts = CLEAN_ATTEMPTS

    centering_eps = 0.035 ### threshold value for centering
    steps_until_sticker_check = 50 ### webots goes veery slow if it checks every frame

    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:
        print('-----------------------')
        if not table_detected:
            if robot.step(controller.time_step)%steps_until_sticker_check == 0:
                print(dist_sensors[0].getValue())
                print("****")
                controller.off_centre_value()
                if abs(controller.sticker_image_offset-0.5) > centering_eps:
                    print('Adjusting Centering')
                    mc.fix_centering(robot, controller.sticker_image_offset)
                print("****")

            table_length, pole_length = table_check.side_check(dist_sensors[2])

            if table_length:  # if not None -> table detected
                print("Table detected")
                table_detected = True
                table_check.stop_scanning()
                table_length = table_length if table_length < 1 else 1
                print("Table Length:", table_length)
                attempts = math.ceil(table_length / HEAD_WIDTH)
                print("TOTAL ATTEMPTS:", attempts)
                distance = (table_length / 2) + pole_length
                mc.move_distance(robot, -distance)  # to back edge of table

            elif dist_sensors[0].getValue() < STOP_THRESHOLD:  # check front distance sensor
                print("Detected wall in front")
                mc.stop(robot)
                # if vc.is_carriage_end(controller.camera):
                #     print("End of carriage detected")
                if left_side:
                    mc.turn_angle(robot, 180)
                    left_side = False

            else:  # business as usual
                #print("Moving forward")
                mc.move_forward(robot)

        else:
            print("Attempts:", attempts)
            for i in range(attempts-1):
                print("Attempt #", i)
                controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
                mc.move_distance(robot, HEAD_WIDTH)
            controller.clean_table(table_check.params['DISTANCE_TO_WALL'])
            table_check.done_cleaning()
            table_detected = False
        print('-----------------------\n')


# Enter here exit cleanup code.
