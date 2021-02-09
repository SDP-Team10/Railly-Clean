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
        next_step = clean_step
        if action == 0:
            mc.stop(self.robot)
            ac.sweep(self.robot)
        elif action == 1:
            mc.stop(self.robot)
            ac.wipe(self.robot)
        elif action == 2:
            mc.stop(self.robot)
            ac.next_iteration(self.robot)
        return (action+1) % ACTION_STEPS, next_step


# Main
if __name__ == "__main__":
    controller = CleaningController()
    robot = controller.robot()
    dist_sensors = controller.distance_sensors
    table_check_l = sc.SideCheck()
    table_check_r = sc.SideCheck()

    table_detected = False
    action = 0
    clean_step = 0
    side = 'f'

    while controller.robot.step(controller.time_step) != -1:
        table_length_l = table_check_l.side_check(robot, dist_sensors[2])
        table_length_r = table_check_r.side_check(robot, dist_sensors[3])

        if not table_detected:  # assume already centered
            if table_length_r or table_length_r:  # if not none -> table detected
                table_check_l.stop_scanning()
                table_check_r.stop_scanning()
                mc.move_back(robot, table_length_l / 2)  # to back edge of table
                table_detected = True
                if table_length_l:
                    side = 'l'
                else:
                    side = 'r'
            elif controller.distance_sensors[0] < STOP_THRESHOLD:  # check front distance sensor
                mc.stop(robot)
                if vc.is_carriage_end(controller.camera):
                    pass
            else:
                mc.move_forward()

        else:
            if side == 'l':
                if clean_step < 2:
                    if action < 3:
                        action, clean_step = controller.clean_table(action, clean_step)
                    else:
                        mc.move_forward(table_length_l / CLEAN_ATTEMPTS)
                        action = 0
                        clean_step += 1
                else:
                    if action < 3:
                        action, clean_step = controller.clean_table(action, clean_step)
                    else:
                        clean_step = 0
                        if table_length_r:
                            mc.turn_angle(180)
                            side = 'r'
                        else:
                            side = 'f'
            if side == 'r':
                if clean_step < CLEAN_ATTEMPTS:
                    if action < 3:
                        action, clean_step = controller.clean_table(action, clean_step)
                    else:
                        mc.move_forward(table_length_l / CLEAN_ATTEMPTS)
                        action = 0
                        clean_step += 1
                else:
                    if action < 3:
                        action, clean_step = controller.clean_table(action, clean_step)
                    else:
                        clean_step = 0
                        side = 'f'
                        if table_length_l:
                            mc.turn_angle(90)
            else:
                table_detected = False
                table_check_l.done_cleaning()
                table_check_r.done_cleaning()


# Enter here exit cleanup code.
