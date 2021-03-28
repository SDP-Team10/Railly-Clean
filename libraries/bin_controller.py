from controller import Robot, Motor, PositionSensor
from collections import deque


class BinController(object):
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.bin_motor = robot.getDevice("door_motor")
        self.bin_sensor = robot.getDevice("door_sensor")
        self.bin_sensor.enable(self.time_step)

        self.closed_pos = 0
        self.opened_pos = 1

        self.last_4_positions = deque([], maxlen=8)

    def open_bin(self):  # 1 rad/s
        self.bin_motor.setPosition(self.opened_pos)
        self.bin_motor.setVelocity(1)
        sensor_val = self.bin_sensor.getValue()
        while round(sensor_val, 3) != self.opened_pos:
            self.last_4_positions.append(sensor_val)
            self.robot.step(self.time_step)
            sensor_val = self.bin_sensor.getValue()
            if self.is_stationary():
                return

    def close_bin(self):
        self.bin_motor.setVelocity(1)  # 1 rad/s
        self.bin_motor.setPosition(self.closed_pos)
        while round(self.bin_sensor.getValue(), 1) != self.closed_pos:
            self.robot.step(self.time_step)

    # use touch sensor?
    def is_full(self):
        return
    
    def is_stationary(self):
        temp = self.last_4_positions.copy()
        if len(temp) == 8:
            last_joint = temp.pop()
            while len(temp) > 0:
                next_joint = temp.pop()
                if round(last_joint, 3) != round(next_joint, 3):
                    return False
                last_joint = next_joint
            return True
        return False
