#!/usr/bin/env python3

"""default_controller controller."""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, distance_sensor
import sys
import os
sys.path.append(os.path.abspath(os.path.join('..', '..')))
from libraries import sticker_detection as vc
from controller import Robot


TIME_STEP = 64  # default time step in ms
BASE_SPEED = 6.5  # maximum velocity of the turtlebot motors
TURN_MULT = 0.3  # constant to slow down turn speed
MOVE_MULT = 0.75  # constant to slow down move speed
TURN_LIMIT = 90  # constant to vary when to fix movement


class TiagoController(object):
    def __init__(self) -> None:
        """Create a robot instance, retrieve the  """
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("front_camera")
        self.camera.enable(self.timestep)
        self.ds_names = [
            "front distance sensor",
            "back distance sensor",
            "right distance sensor",
            "left distance sensor",
        ]
        self.distance_sensors = self.init_dist()
        self.left_motor = self.robot.getDevice("wheel_left_joint")
        self.right_motor = self.robot.getDevice("wheel_right_joint")

    # Distance sensor initilazation
    def init_dist(self):
        """ function init_dist
        :return: list of enabled distance sensor objects
        """
        dist_sensors = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.timestep)
            dist_sensors.append(sensor)
        return dist_sensors

    def should_stop(self, distance_sensor, threshold=0.6) -> bool:
        """function should_stop
        :param distance_sensor: distance_sensor that needs to be checked for objects in the way
        :return: boolean if distance is less than threshold return true, otherwise false
        """
        return distance_sensor.getValue() < threshold

    def enable_speed_control(self, motor, init_velocity=0.0) -> None:
        """Disable position control and set velocity of the motor to initVelocity (in rad / s)"""
        motor.setPosition(float("inf"))
        motor.setVelocity(init_velocity)

    def limit_vel(self, left_vel, right_vel) -> None:
        if left_vel > self.left_motor.getMaxVelocity():
            left_vel = self.left_motor.getMaxVelocity()
        if right_vel > self.right_motor.getMaxVelocity():
            right_vel = self.right_motor.getMaxVelocity()
        if left_vel < -self.left_motor.getMaxVelocity():
            left_vel = -self.left_motor.getMaxVelocity()
        if right_vel < -self.right_motor.getMaxVelocity():
            right_vel = -self.right_motor.getMaxVelocity()
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)

    def check_distance_sensors(self):
        return self.distance_sensors[3].getValue(), self.distance_sensors[2].getValue()

    def adjust_movement(self, threshold=4):
        left_dist, right_dist = self.check_distance_sensors()
        left_vel = 0
        right_vel = 0
        if abs(left_dist - right_dist) > 0.1:
            if right_dist < left_dist < threshold:
                left_vel = -BASE_SPEED * TURN_MULT
                print('Turning left')
            elif left_dist < right_dist < threshold:
                right_vel = -BASE_SPEED * TURN_MULT
                print('Turning right')
        return left_vel, right_vel


# Main #
if __name__ == "__main__":
    tiago_controller = TiagoController()
    tiago_controller.enable_speed_control(tiago_controller.left_motor)
    tiago_controller.enable_speed_control(tiago_controller.right_motor)

    # Flags
    in_threshold = False
    detected_sticker = False
    # Main loop: perform simulation steps until Webots is stopping the controller
    while tiago_controller.robot.step(TIME_STEP) != -1:
        new_left_vel = 0
        new_right_vel = 0
        time = tiago_controller.robot.getTime()
        for ds in tiago_controller.distance_sensors:
            print(ds.getName(), ds.getValue())

        in_threshold = tiago_controller.should_stop(
            tiago_controller.distance_sensors[0]
        )
        if not in_threshold and not detected_sticker:
            new_left_vel, new_right_vel = tiago_controller.adjust_movement()
            new_left_vel += BASE_SPEED * MOVE_MULT
            new_right_vel += BASE_SPEED * MOVE_MULT
            tiago_controller.limit_vel(new_left_vel, new_right_vel)
        elif in_threshold and not detected_sticker:
            if vc.is_carriage_end(tiago_controller.camera):
                detected_sticker = True
                news_left_vel = -BASE_SPEED * MOVE_MULT
                new_right_vel = -BASE_SPEED * MOVE_MULT
                tiago_controller.limit_vel(new_left_vel, new_right_vel)
        elif not in_threshold and detected_sticker:
            new_left_vel = 0
            new_right_vel = 0
            tiago_controller.limit_vel(new_left_vel, new_right_vel)
