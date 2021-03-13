import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join("..", "..")))
from libraries import sticker_detection as vc
from libraries import side_check as sc
from libraries import move_lib_new_base as mc
from libraries import arm_controller_trash as ac
from controller import Robot
from libraries import bin_controller as bc


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
        #self.left_motor = self.robot.getDevice("wheel_left_joint")
        #self.right_motor = self.robot.getDevice("wheel_right_joint")
        #self.arm_controller = ac.ArmController(self.robot)

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
            self.sticker_offset = cpa
            self.sticker_image_offset = sticker_centroid[0]/image_width
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

steps_until_sticker_check = 300
centering_eps = 0.035 ### for rotations to center
centering_dist_eps = 0.2 ### for sideways movement to center
centering_move_mult = 1/3 ### for sideways movement to center (how much of side diff to move)

# Controller assumes library functions handle their own timesteps
if __name__ == "__main__":
    controller = CleaningController()
    robot, dist_sensors = controller.robot, controller.distance_sensors
    table_check, table_length, table_detected, left_side = sc.SideCheck(robot), None, False, True
    attempts = CLEAN_ATTEMPTS
    bin_controller = bc.BinController(robot)
    bin_controller.close_bin()
    desired_x = -0.20
    l_dist = dist_sensors[2].getValue()
    r_dist = dist_sensors[3].getValue()
    # Assume robot is already centered
    while robot.step(controller.time_step) != -1:

        print('-----------------------')
        if not table_detected:
            if robot.step(controller.time_step)%steps_until_sticker_check == 0:
                print("****")
                temp_l_dist = dist_sensors[2].getValue()
                temp_r_dist = dist_sensors[3].getValue()
                l_dist = temp_l_dist if abs(temp_l_dist-l_dist) < centering_dist_eps*2 else l_dist ### useful for chairs and poles but may cause future problems
                r_dist = temp_r_dist if abs(temp_r_dist-r_dist) < centering_dist_eps*2 else r_dist
                dist_diff = r_dist - l_dist
                if abs(dist_diff) > centering_dist_eps:
                    print('Moving to Center')
                    mc.move_distance(robot, 'side', dist_diff*centering_move_mult)
                else:
                    print('No side movement needed')
                print(l_dist, r_dist, dist_diff, dist_diff*centering_move_mult)
                print("****")
                print("****")
                controller.off_centre_value()
                if abs(controller.sticker_image_offset -0.5) > centering_eps:
                    print('Adjusting Centering')
                    print('sticker_image_offset: ',abs(controller.sticker_image_offset -0.5))
                    mc.fix_centering(robot, controller.sticker_image_offset)
                else:
                    print('No rotation needed')
                print("****")

            print(dist_sensors[0].getValue())
            table_length, pole_length, chair_dist = table_check.side_check(dist_sensors[2])

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
                print("Moving forward")
                mc.move_forward(robot)

        else:
            bin_controller.open_bin()
            print("Attempts:", attempts)
            for i in range(attempts-1):
                print("Attempt #", i)
                controller.clean_table(table_check.params['DISTANCE_TO_WALL'], desired_x)
                mc.move_distance(robot, HEAD_WIDTH)
            controller.clean_table(table_check.params['DISTANCE_TO_WALL'], desired_x)
            table_check.done_cleaning()
            table_detected = False
    print('-----------------------\n')


# Enter here exit cleanup code.
