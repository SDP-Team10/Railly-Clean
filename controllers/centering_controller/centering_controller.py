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
        # self.camera = self.robot.getDevice("front_camera")
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.front_camera = self.robot.getDevice("front_camera")
        self.side_camera = self.robot.getDevice("side_camera")
        self.front_camera.enable(self.time_step)
        if self.side_camera.hasRecognition():
            self.side_camera.enable(self.time_step)
            self.side_camera.recognitionEnable(self.time_step)
        self.sticker_offset = 0
        self.sticker_image_offset = 0
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

    def clean_table(self, dist_to_wall):
        desired_x = -0.20
        mc.stop(self.robot)
        if tc.has_valuable(self.side_camera):
            return
        self.bin_controller.open_bin()
        self.arm_controller.sweep(dist_to_wall, desired_x)

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

    def check_camera(self):
        print(type(self.camera))

    def off_centre_value(self):
        try:
            field_of_view = self.front_camera.getFov()
            image_width = self.front_camera.getWidth()
            sticker_centroid = vc.centroid_detection(self.front_camera)
            cpa = vc.camera_point_angle(field_of_view, image_width, sticker_centroid)
            self.sticker_offset = cpa
            self.sticker_image_offset = sticker_centroid[0]/image_width
        except:
            print(self.sticker_offset)

    def vision_centering(self):
        field_of_view = self.front_camera.getFov()
        image_width = self.front_camera.getWidth()
        print(field_of_view)
        print(image_width)

    def centre(self, l_dist, r_dist):
        centred = False
        print("****")
        temp_l_dist = self.distance_sensors[2].getValue()
        temp_r_dist = self.distance_sensors[3].getValue()
        # useful for chairs and poles but may cause future problems
        left_dist = temp_l_dist if abs(temp_l_dist-l_dist) < centering_dist_eps*2 else l_dist
        right_dist = temp_r_dist if abs(temp_r_dist-r_dist) < centering_dist_eps*2 else r_dist
        dist_diff = right_dist - left_dist
        if abs(dist_diff) > centering_dist_eps:
            print('Moving to Center')
            mc.move_distance(self.robot, 'side', dist_diff*centering_move_mult)
        else:
            print('No side movement needed')
        print(left_dist, right_dist, dist_diff, dist_diff*centering_move_mult)
        print("****")
        print("****")
        controller.off_centre_value()
        if abs(self.sticker_image_offset-0.5) > centering_eps:
            print('Adjusting Centering')
            print('sticker_image_offset:', abs(self.sticker_image_offset-0.5))
            mc.fix_centering(self.robot, self.sticker_image_offset)
        else:
            print('No rotation needed')
            centred = True
        print("****")
        return left_dist, right_dist, centred


# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, bin_controller, dist_sensors = controller.robot, controller.bin_controller, controller.distance_sensors
    table_check, table_length, distance_to_wall = sc.SideCheck(robot), None, None
    table_detected, done_cleaning, left_side, centred = False, False, True, False  # flags
    attempts = CLEAN_ATTEMPTS

    steps_until_sticker_check = 280
    centering_eps = 0.035  # for rotations to center
    centering_dist_eps = 0.2  # for sideways movement to center
    centering_move_mult = 1/3  # for sideways movement to center (how much of side diff to move)
    l_dist = dist_sensors[2].getValue()
    r_dist = dist_sensors[3].getValue()

    while robot.step(controller.time_step) != -1:
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

        elif not table_detected:
            if robot.step(controller.time_step) % steps_until_sticker_check == 0:
                l_dist, r_dist, centred = controller.centre(l_dist, r_dist)
                
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
            centred = False
