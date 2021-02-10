"""arm_control controller."""

import sys
import os

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import move_lib as mc
from libraries import arm_controller as ac  # to change
from controller import Robot


TIME_STEP = 32  # this or robot.getBasicTimeStep()
TABLE_WIDTH = 1  # parameter
HEAD_WIDTH = 0.3  # parameter
CLEAN_ATTEMPTS = int(TABLE_WIDTH // HEAD_WIDTH)
ACTION_STEPS = 4
STOP_THRESHOLD = 0.6


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
        if   action == 0: ac.sweep(self.robot)
        elif action == 1: ac.wipe(self.robot)
        elif action == 2: ac.next_iteration(self.robot)
    
    @staticmethod
    def next_row(table_check_l, table_check_r):
        table_check_l.done_cleaning()
        table_check_r.done_cleaning()
        return 'f', False


# Each library function has at least a timestep to execute
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check_l, table_check_r = sc.SideCheck(), sc.SideCheck()
    table_length_l, table_length_r = None, None
    table_detected, action, clean_step, side = False, 0, 0, 0, 'f'


    # Assume robot is already centered
    while controller.robot.step(controller.time_step) != -1:
        if not table_detected:
            table_length_l = table_check_l.side_check(robot, dist_sensors[2])
            table_length_r = table_check_r.side_check(robot, dist_sensors[3])
            
            if table_length_r or table_length_r:  # if not None -> table detected
                table_detected = True
                table_check_l.stop_scanning()
                table_check_r.stop_scanning()
                if table_length_l:
                    mc.move_distance(robot, table_length_l / 2, -1)  # to back edge of table
                    side = 'l'
                else:
                    mc.move_distance(robot, table_length_r / 2, 1)  # to front edge of table
                    side = 'r'
                    action = -1  # extra action needed
                
            elif controller.dist_sensors[0] < STOP_THRESHOLD:  # check front distance sensor
                mc.stop(robot)
                if vc.is_carriage_end(controller.camera):
                    print("End of carriage detected")
                
            else:  # business as usual
                mc.move_forward(robot)

        else:
            # Left side first
            if side == 'l':
                if action < ACTION_STEPS-1:
                    controller.clean_table(action, clean_step)
                else:  # last action concerns moving to next step
                    if clean_step < CLEAN_ATTEMPTS-1:
                        mc.move_distance(robot, table_length / CLEAN_ATTEMPTS, 1)
                    else:  # last action of last cleaning step
                        if table_length_r:  # still cleaning...
                            mc.turn_angle(robot, 180)
                            side = 'r'
                        else:  # call escape_hell()
                            side, table_detected = controller.next_row(table_check_l, table_check_r)
                    clean_step = (clean_step+1) % CLEAN_ATTEMPTS
                action = (action+1) % ACTION_STEPS
            
            elif side == 'r':
                if action == -1:
                    mc.turn_angle(robot, 180)
                else:
                    if action < ACTION_STEPS-1:
                        controller.clean_table(action, clean_step)
                    else:
                        if clean_step < CLEAN_ATTEMPTS-1:
                            mc.move_distance(robot, table_length / CLEAN_ATTEMPTS, 1)
                        else:  # escape
                            mc.turn_angle(robot, 180)
                            side, table_detected = controller.next_row(table_check_l, table_check_r)
                        clean_step = (clean_step+1) % CLEAN_ATTEMPTS 
                action = (action+1) % ACTION_STEPS

# Enter here exit cleanup code.
