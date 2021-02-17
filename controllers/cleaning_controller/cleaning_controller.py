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
CLEAN_ATTEMPTS = math.floor(TABLE_WIDTH // HEAD_WIDTH)


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
        dist_sensors = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            dist_sensors.append(sensor)
        return dist_sensors

    def clean_table(self):
        mc.stop(self.robot)
        self.arm_controller.sweep()
        # ac.wipe(self.robot)
    
    @staticmethod
    def next_row(table_check_l, table_check_r):
        table_check_l.done_cleaning()
        table_check_r.done_cleaning()
        return False, 'f'


# Controller assumes library functions handle their own time steps
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check_l, table_check_r = sc.SideCheck(robot), sc.SideCheck(robot)
    table_length_l, table_length_r = None, None
    table_detected, side = False, 'f'  # control flags

    # Assume robot is already centered
    while controller.robot.step(controller.time_step) != -1:
        if not table_detected:
            table_length_l, pole_length = table_check_l.side_check(dist_sensors[2])
            table_length_r, pole_length = table_check_r.side_check(dist_sensors[3])
            
            if table_length_l or table_length_r:  # if not None -> table detected
                table_detected = True
                table_check_l.stop_scanning()
                table_check_r.stop_scanning()
                if table_length_l:  # prioritize left side since arm & camera are on the left
                    side = 'l'
                    # mc.move_distance(robot, table_length_l / 2, -1)  # to back edge of table
                else:
                    side = 'r'
                    distance = (table_length_r / 2) - (2 * pole_length)
                    mc.move_distance(robot, distance)  # to front edge of table
                    mc.turn_angle(robot, 180)
            
            elif dist_sensors[0].getValue() < STOP_THRESHOLD:  # check front distance sensor
                mc.stop(robot)
                if vc.is_carriage_end(controller.camera):
                    print("End of carriage detected")
            
            else:  # business as usual
                mc.move_forward(robot)

        else:
            attempts = CLEAN_ATTEMPTS
            if side == 'l':
                # Since moving backwards hasn't been implemented, robot only cleans from the pole
                attempts = math.ceil(CLEAN_ATTEMPTS / 2)
            for i in range(attempts - 1):
                controller.clean_table()
                table_length = table_length_l if side == 'l' else table_length_r
                mc.move_distance(robot, table_length / CLEAN_ATTEMPTS)
            # Last step concerns how to move onto next table
            if side == 'l':
                if table_length_r:  # this row still has more to clean
                    mc.turn_angle(robot, 180)
                    side = 'r'
                else:  # can move onto next row, update control flags
                    table_detected, side = CleaningController.next_row(table_check_l, table_check_r)
            elif side == 'r':
                mc.turn_angle(robot, 180)
                table_detected, side = CleaningController.next_row(table_check_l, table_check_r)


# Enter here exit cleanup code.
