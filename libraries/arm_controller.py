"""arm_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor


class ArmController(object):
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.ps_names = [
            "base_sensor",
            "sec_1_sensor",
            "sec_2_sensor",
            "head_sensor"
        ]
        self.rm_names = [
            "base_motor",
            "sec_1_motor",
            "sec_2_motor",
            "head_motor"
        ]
        self.position_sensors = self.init_ps()
        self.rotational_motors = self.init_rm()

        self.table_top_sec_1 = 0
        self.table_bottom_sec_1 = 0.84  # The edge of the table
        self.table_top_sec_2 = 0
        self.table_bottom_sec_2 = 2.3  # The edge of the table

    def init_ps(self):
        position_sensors = []
        for name in self.ps_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            position_sensors.append(sensor)
        return position_sensors

    def init_rm(self):
        rotational_motors = []
        for name in self.rm_names:
            motor = self.robot.getDevice(name)
            rotational_motors.append(motor)
        return rotational_motors

    def sweep_action(self):
        # Sweep the table from top edge to bottom edge
        self.rotational_motors[1].setPosition(self.table_bottom_sec_1)
        self.rotational_motors[2].setPosition(self.table_bottom_sec_2)
        self.rotational_motors[1].setVelocity(0.3)
        self.rotational_motors[2].setVelocity(0.5)
        while self.robot.step(self.time_step) != -1:
            if (round(self.position_sensors[1].getValue(), 2) == self.table_bottom_sec_1 and
                    round(self.position_sensors[2].getValue(), 2) == self.table_bottom_sec_2):
                return

    def tuck_in_action(self):
        # Set the arm to tuck in position
        self.rotational_motors[1].setPosition(0.5)
        self.rotational_motors[2].setPosition(2.7)
        self.rotational_motors[1].setVelocity(0.3)
        self.rotational_motors[2].setVelocity(0.5)
        while self.robot.step(self.time_step) != 1:
            if (round(self.position_sensors[1].getValue(), 2) == 0.5 and
                    round(self.position_sensors[2].getValue(), 2) == 2.7):
                print('Finished tuck in')
                return

    def set_sweeping_action(self):
        # Set the arm from tuck in position to the top edge of the table for upcoming sweep action
        self.rotational_motors[1].setPosition(1.5)
        self.rotational_motors[2].setPosition(0.5)
        self.rotational_motors[1].setVelocity(0.05)
        self.rotational_motors[2].setVelocity(0.6)
        while self.robot.step(self.time_step) != -1:
            if (round(self.position_sensors[1].getValue(), 2) == 1.52 and
                    round(self.position_sensors[2].getValue(), 1) == 0.5):
                return

    def sweep(self):
        # It assumes the arm is tucked in and try to do the sweep action from reaching the 
        # top end of the table. After it finishes, it returns to tuck in positions
        self.set_sweeping_action()
        self.sweep_action()
        self.tuck_in_action()
