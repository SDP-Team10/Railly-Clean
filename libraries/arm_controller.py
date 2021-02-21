"""arm_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor
from libraries import kinematics

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
        dqs = []
        d_y = 1.4
        d_x = -0.28
        y_step = 0.1*d_y
        x_step = 0.05*d_x
        while d_y > 0.2:
            dq= kinematics.brute_force(d_x, d_y, 0.78, 0.7,round(self.position_sensors[1].getValue(), 2), round(self.position_sensors[2].getValue(), 2))
            d_y -= y_step
            print(d_y)
            d_x -= x_step     
            self.rotational_motors[1].setPosition(dq[0])
            self.rotational_motors[2].setPosition(dq[1])
            self.rotational_motors[1].setVelocity(0.2)
            self.rotational_motors[2].setVelocity(0.6)
            print(dq[0])
            print(dq[1])
            while self.robot.step(self.time_step) != -1:
                print("1=", self.position_sensors[1].getValue())
                print("2=",self.position_sensors[2].getValue())
                if (round(self.position_sensors[1].getValue(), 2) == round(dq[0],2) and
                        round(self.position_sensors[2].getValue(), 1) == round(dq[1],1)):
                    break
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
        d_y = 1.4
        d_x = -0.28
        k = kinematics.brute_force(d_x, d_y, 0.78, 0.7)
        #pos1 = 1.44
        pos1 = k[0]
        #pos2 = 0.64
        pos2 = k[1]
        vel1 = 0.4
        vel2 = 1
        self.rotational_motors[1].setPosition(pos1)
        self.rotational_motors[2].setPosition(pos2)
        self.rotational_motors[1].setVelocity(vel1)
        self.rotational_motors[2].setVelocity(vel2)
        print("TORQUE", self.rotational_motors[1].getAvailableTorque())
        print("HIGHEST_TORQUE", self.rotational_motors[1].getMaxTorque())
        print("FORCE", self.rotational_motors[1].getAvailableForce())
        print("MAX FORCE", self.rotational_motors[1].getAvailableTorque())
        while self.robot.step(self.time_step) != -1:
            print("1",self.position_sensors[1].getValue())
            print(self.position_sensors[2].getValue())
            if (round(self.position_sensors[1].getValue(), 2) == round(pos1,2) and
                    round(self.position_sensors[2].getValue(), 2) == round(pos2,2)):
                print("bingo")
                print(self.rotational_motors[1].getVelocity())
                print(self.rotational_motors[2].getVelocity())
                self.rotational_motors[1].setVelocity(0.0)
                self.rotational_motors[2].setVelocity(0.0)
                return

    def sweep(self):
        # It assumes the arm is tucked in and try to do the sweep action from reaching the 
        # top end of the table. After it finishes, it returns to tuck in positions
        self.set_sweeping_action()
        self.sweep_action()
        self.tuck_in_action()
