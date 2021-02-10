"""arm_control controller."""

import sys
import os

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import movement as mc  # to change
from libraries import arm_controller as ac  # to change
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
CLEAN_ATTEMPTS = 3  # parameter for how many times it takes to cover a whole table
ACTION_STEPS = 4
STOP_THRESHOLD = 0.6
# TABLE_WIDTH = 1


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

    def clean_table(self, action, clean_step):
        mc.stop(self.robot)
        if action == 0:   ac.sweep(self.robot)
        elif action == 1: ac.wipe(self.robot)
        elif action == 2: ac.next_iteration(self.robot)
    
    @staticmethod
    def next_row(table_check_l, table_check_r):
        table_check_l.done_cleaning()
        table_check_r.done_cleaning()
        return 'f', False


# Main
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check_l, table_check_r = sc.SideCheck(), sc.SideCheck()
    table_detected, action, clean_step, side = False, 0, 0, 'f'

    while controller.robot.step(controller.time_step) != -1:
        if not table_detected:  # assume already centered
            table_length_l = table_check_l.side_check(robot, dist_sensors[2])
            table_length_r = table_check_r.side_check(robot, dist_sensors[3])
            if table_length_r or table_length_r:  # if not None -> table detected
                table_detected = True
                table_check_l.stop_scanning()
                table_check_r.stop_scanning()
                mc.move_back(robot, table_length_l / 2)  # to back edge of table
                if table_length_l: side = 'l'
                else:              side, action = 'r', -2  # extra actions needed
            elif controller.distance_sensors[0] < STOP_THRESHOLD:  # check front distance sensor
                mc.stop(robot)
                if vc.is_carriage_end(controller.camera): pass
            else:
                mc.move_forward()  # business as usual

        else:
            if side == 'l':  # left side first
                if action < ACTION_STEPS-1:
                    controller.clean_table(action, clean_step)
                else:  # last action concerns moving
                    if clean_step < CLEAN_ATTEMPTS-1:
                        mc.move_forward(table_length_l / CLEAN_ATTEMPTS)
                    else:  # last action of last cleaning step
                        if table_length_r:  # still cleaning...
                            mc.turn_angle(180)
                            side = 'r'
                        else:  # calling controller.escape_hell()
                            side, table_detected = controller.next_row(table_check_l, table_check_r)
                action = (action+1) % ACTION_STEPS
                clean_step = (clean_step+1) % CLEAN_ATTEMPTS
            
            if side == 'r':
                if action == -2:
                    mc.move_forward(table_length_l)
                    action += 1
                elif action == -1:
                    mc.turn_angle(180)
                    action += 1
                elif action < ACTION_STEPS-1:
                    controller.clean_table(action, clean_step)
                    action = (action+1) % ACTION_STEPS
                    clean_step = (clean_step+1) % CLEAN_ATTEMPTS
                else:
                    if clean_step < CLEAN_ATTEMPTS-1:
                        mc.move_forward(table_length_l / CLEAN_ATTEMPTS)
                    else:
                        mc.turn_angle(180)
                        side, table_detected = controller.next_row(table_check_l, table_check_r)
                    action = (action+1) % ACTION_STEPS
                    clean_step = (clean_step+1) % CLEAN_ATTEMPTS


# Enter here exit cleanup code.
