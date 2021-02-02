# """aisle_navigation controller."""
import time
from typing import Tuple

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor

TIME_STEP = 64
MAX_SPEED = 6.28


class AisleNavigation(object):
    def __init__(self) -> None:
        # create the Robot instance.
        self.robot = Robot()
        # static parameters
        self.wheel_radius = 0.0985
        self.angle_of_rotation = 6.28 / 4  # rotate 90 degree
        self.linear_velocity = self.wheel_radius * MAX_SPEED
        self.distance_between_wheels = 0.404
        self.rate_of_rotation = (
            2 * self.linear_velocity
        ) / self.distance_between_wheels  # circular motion
        self.duration_turn = self.angle_of_rotation / self.rate_of_rotation
        self.duration_time = 0
        ## Initial speed
        self.speed = 0.5 * MAX_SPEED
        self.left_speed = 0
        self.right_speed = 0
        # flags
        self.stop = False
        self.turn = False
        self.detected = False
        self.origin = False
        self.calculate = False
        # setup devices required for the robot to work
        self.setup_devices()

    def setup_devices(self) -> None:
        self.left_motor = self.robot.getDevice("wheel_right_joint")
        self.right_motor = self.robot.getDevice("wheel_left_joint")
        self.left_ps = self.robot.getDevice("wheel_left_joint_sensor")
        self.right_ps = self.robot.getDevice("wheel_right_joint_sensor")
        self.front_ds = self.robot.getDevice("front distance sensor")
        self.back_ds = self.robot.getDevice("back distance sensor")
        # enable the robot devices
        self.left_ps.enable(TIME_STEP)
        self.right_ps.enable(TIME_STEP)
        self.front_ds.enable(TIME_STEP)
        self.back_ds.enable(TIME_STEP)
        # set the target position of the motors
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

    def robot_go_forward(self) -> Tuple[float, float]:
        self.left_speed = self.speed
        self.right_speed = self.speed
        return self.left_speed, self.right_speed

    def robot_go_back(self) -> Tuple[float, float]:
        print("Going back")
        self.left_speed = -self.speed
        self.right_speed = -self.speed
        return self.left_speed, self.right_speed

    def robot_stop(self) -> Tuple[float, float]:
        print("stop")
        time.sleep(0.5)
        self.left_speed = 0
        self.right_speed = 0
        return self.left_speed, self.right_speed

    def calculate_turn_duration(self, curr_time: float) -> float:
        return curr_time + self.duration_turn

    def robot_turn(self) -> Tuple[float, float]:
        self.left_speed = -MAX_SPEED
        self.right_speed = MAX_SPEED
        return self.left_speed, self.right_speed

    def detect_front_object(self) -> bool:
        return self.front_ds.getValue() < 5e4


if __name__ == "__main__":

    navigator = AisleNavigation()
    # Main loop: perform simulation steps until Webots is stopping the controller
    while navigator.robot.step(TIME_STEP) != -1:

        current_time = navigator.robot.getTime()

        print(
            f"Wheel L {navigator.left_ps.getValue()}   R {navigator.right_ps.getValue()}\n"
        )
        print(f"detection: {navigator.front_ds.getValue}")
        # print('look up, ', frontDs.getLookupTable())
        # front_distance = frontDs.getValue()
        # print('fd ', front_distance)
        # # Use Case 1
        # if not detected and not stop:
        #     if int(leftPs.getValue()) == 5:
        #         detected = True
        #         stop = True
        #     else:
        #         left_speed, right_speed = robot_go_forward()
        # if stop and not origin:
        #     left_speed, right_speed = robot_stop()
        #     stop = False
        # if leftPs.getValue() > 0:
        #     left_speed, right_speed = robot_go_back()
        #     leftMotor.setVelocity(left_speed)
        #     rightMotor.setVelocity(right_speed)
        # elif leftPs.getValue() < 0:
        #     left_speed, right_speed = robot_stop()

        # # Use Case 2
        # if not detected:
        #     if int(leftPs.getValue()) == 5:
        #         detected = True
        #         turn = True
        #         stop = True
        #         print('in')
        #     else:
        #         left_speed, right_speed = robot_go_forward()

        # if detected and turn:
        #     if stop:
        #         left_speed, right_speed = robot_stop()
        #         stop = False

        # if not calculate:
        #     calculate = True
        #     duration_time = calculate_turn_duration(current_time)

        # else:
        #     if current_time < duration_time:
        #         print('current time: {} Duration time: {}'.format(current_time, duration_time))
        #         print('turning with MAX_SPEED: ', MAX_SPEED)
        #         left_speed, right_speed = robot_turn()
        #     else:
        #         stop = True
        #         detected = False
        #         turn = False

        # print("speed: ", navigator.left_speed)
        # navigator.left_motor.setVelocity(navigator.left_speed)
        # navigator.right_motor.setVelocity(navigator.right_speed)

        # current_time = robot.getTime()

        # print(stop)
        # print(turn)
        # if not stop or not turn:
        #     print('Wheel L {}   R {}\n'.format(leftPs.getValue(),rightPs.getValue()))
        #     print('Wheel R {}\n'.format(rightPs.getValue()))
        #     robot_go_straight()

        # if int(leftPs.getValue()) == 5 and int(rightPs.getValue()) == 5:
        #     stop = True
        #     robot_stop()

        # elif int(leftPs.getValue()) == 15 and int(rightPs.getValue()) == 15:
        #     print('now im in')
        #     stop = True
        #     turn = True
        #     rot_start_time = current_time
        #     duration_time = rot_start_time + duration_turn
        #     time.sleep(1)
        # elif turn == True:
        #     print('in1?')
        #     print('start: ', rot_start_time)
        #     print('current ', current_time)
        #     print('duration ',duration_time)

        # if rot_start_time <= current_time < duration_time:
        #     print('turning')
        #     right_speed = -MAX_SPEED
        #     left_speed = MAX_SPEED
        # else:
        #     print(stop)
        #     print(turn)
        #     print(left_speed)
        #     print(right_speed)
        #     robot_stop()
        #     print('breaked')
        #     print(speed)
        #     left_speed = 0 * MAX_SPEED
        #     right_speed = 0 * MAX_SPEED

        # leftMotor.setVelocity(left_speed)
        # rightMotor.setVelocity(right_speed)

        # if int(leftPs.getValue()) == 270 and int(rightPs.getValue()) == 270 and not stop:
        #     print('in\n')
        #     stop = True
        #     leftMotor.setVelocity(0)
        #     rightMotor.setVelocity(0)
        # else :
        #     pass
