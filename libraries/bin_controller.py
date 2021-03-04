from controller import Robot, Motor, PositionSensor

class BinController(object):
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.bin_motor = robot.getDevice("door_motor")
        self.bin_sensor = robot.getDevice("door_sensor")
        self.bin_sensor.enable(self.time_step)

        self.closed_pos = 0
        self.opened_pos = -1

    def open_bin(self): # 1 rad/s
        self.bin_motor.setPosition(self.opened_pos)
        self.bin_motor.setVelocity(1)
        while round(self.bin_sensor.getValue(),1) != self.opened_pos:
            self.robot.step(self.time_step)

    def close_bin(self):
        self.bin_motor.setVelocity(1)  # 1 rad/s
        self.bin_motor.setPosition(self.closed_pos)
        while round(self.bin_sensor.getValue(),1) != self.closed_pos:
            self.robot.step(self.time_step)
    
    # use touch sensor?
    def is_full(self):
        return
