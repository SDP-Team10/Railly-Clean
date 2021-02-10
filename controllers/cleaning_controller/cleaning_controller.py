"""arm_control controller."""

import sys
import os

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import move_lib as mc  # to change
from libraries import arm_controller as ac  # to change
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
STOP_THRESHOLD = 0.6
TABLE_WIDTH = 1
HEAD_WIDTH = 0.3
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

    def clean_table(self):
        mc.stop(self.robot)
        ac.sweep(self.robot)
        ac.wipe(self.robot)
        ac.next_iteration(self.robot)
    
    @staticmethod
    def next_row(table_check_l, table_check_r):
        table_check_l.done_cleaning()
        table_check_r.done_cleaning()
        return False, 'f'


if __name__ == "__main__":  # assumes functions take care of their own timesteps
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check_l, table_check_r = sc.SideCheck(), sc.SideCheck()
    table_length_l, table_length_r = None, None
    table_detected, side = False, 'f'

    while controller.robot.step(controller.time_step) != -1:
        if not table_detected:  # assume already centered
            table_length_l = table_check_l.side_check(robot, dist_sensors[2])
            table_length_r = table_check_r.side_check(robot, dist_sensors[3])
            
            if table_length_l or table_length_r:  # if not None -> table detected
                table_detected = True
                table_check_l.stop_scanning()
                table_check_r.stop_scanning()
                if table_length_l:
                    side = 'l'
                    mc.move_distance(robot, table_length_l / 2, -1)  # to back edge of table
                else:
                    side = 'r'
                    mc.move_distance(robot, table_length_r / 2, 1)  # to front edge of table
                    mc.turn_angle(robot, 180)
            
            elif controller.distance_sensors[0] < STOP_THRESHOLD:  # check front distance sensor
                mc.stop(robot)
                if vc.is_carriage_end(controller.camera): pass
            
            else:
                mc.move_forward(robot)  # business as usual

        else:
            for i in range(CLEAN_ATTEMPTS-1):
                controller.clean_table()
                table_length = table_length_l if table_length_l else table_length_r
                mc.move_distance(robot, table_length / CLEAN_ATTEMPTS, 1)
            if side == 'l':  # left side first
                if table_length_r:  # more to clean
                    mc.turn_angle(robot, 180)
                    side = 'r'
                else:
                    table_detected, side = CleaningController.next_row(table_check_l, table_check_r)
            elif side == 'r':
                mc.turn_angle(robot, 180)
                table_detected, side = CleaningController.next_row(table_check_l, table_check_r)


# Enter here exit cleanup code.
