"""arm_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from collections import deque

from controller import Robot, Motor, PositionSensor
from libraries import kinematics_v100 as kinematics


class ArmController(object):
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.ps_names = [
            "base_sensor",
            "sec_1_sensor",
            "sec_2_sensor",
            "sec_3_sensor",
            "head_sensor"
        ]
        self.rm_names = [
            "base_motor",
            "sec_1_motor",
            "sec_2_motor",
            "sec_3_motor",
            "head_motor"
        ]

        self.prs_names = [
            "head_touch_sensor"
        ]

        self.position_sensors = self.init_ps()
        self.rotational_motors = self.init_rm()
        self.pressure_sensors = self.init_prs()

        self.table_top_sec_1 = 0
        self.table_bottom_sec_1 = 0.84  # The edge of the table
        self.table_top_sec_2 = 0
        self.table_bottom_sec_2 = 2.3  # The edge of the table
        self.last_4_positions = deque([], maxlen=4)
        self.sec_1_length = 0.5
        self.sec_2_length = 0.5
        self.sec_3_length = 0.5
        self.head_length = 0.07

        self.target_table_height = -0.15

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

    def init_prs(self):
        pressure_sensors = []
        for name in self.prs_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            pressure_sensors.append(sensor)
        return pressure_sensors

    def sweep_action(self, distance_to_wall, height = -0.07):
        print("start sweep action")
        # Sweep the table from top edge to bottom edge
        dqs = []
        # Robot upper body has width = 0.3
        d_y = distance_to_wall
        d_x = height
        tuck_in = False
        print("d_x is", d_x)
        y_step = 0.1 * d_y
        x_step = 0.05 * d_x
        while d_y > 0.4:

            dq = kinematics.all_joints(d_x, d_y, self.sec_1_length,self.sec_2_length,self.sec_3_length, self.head_length, round(self.position_sensors[1].getValue(), 2),
                                        round(self.position_sensors[2].getValue(), 2), round(self.position_sensors[3].getValue(),2),round(self.position_sensors[4].getValue(),2))

            d_y -= 0.01
            print(d_y)
            print(d_x)
            self.rotational_motors[1].setPosition(dq[0])
            self.rotational_motors[2].setPosition(dq[1])
            self.rotational_motors[3].setPosition(dq[2])
            self.rotational_motors[4].setPosition(dq[3])
            self.rotational_motors[1].setVelocity(0.6)
            self.rotational_motors[2].setVelocity(0.5)
            self.rotational_motors[3].setVelocity(0.5)
            self.rotational_motors[4].setVelocity(0.8)
            while self.robot.step(self.time_step) != -1:
                print("UNDER PRESSURE: ",self.pressure_sensors[0].getValue())
                self.last_4_positions.append([self.position_sensors[1].getValue(), self.position_sensors[2].getValue(),
                                              self.position_sensors[3].getValue(), self.position_sensors[4].getValue()])
                if self.is_stationary(self.last_4_positions):
                    d_x = d_x + 0.005
                    print("stationary")
                    break

                if not tuck_in:
                    if self.pressure_sensors[0].getValue() > 10.0:
                        d_x = d_x + 0.001
                        dq = kinematics.all_joints(d_x, d_y, self.sec_1_length, self.sec_2_length, self.sec_3_length,
                                                   self.head_length, round(self.position_sensors[1].getValue(), 2),
                                                   round(self.position_sensors[2].getValue(), 2),
                                                   round(self.position_sensors[3].getValue(), 2),
                                                   round(self.position_sensors[4].getValue(), 2))
                        print(d_y)
                        print(d_x)
                        self.rotational_motors[1].setPosition(dq[0])
                        self.rotational_motors[2].setPosition(dq[1])
                        self.rotational_motors[3].setPosition(dq[2])
                        self.rotational_motors[4].setPosition(dq[3])
                        continue

                    if self.pressure_sensors[0].getValue() < 0.2:
                        d_x = d_x - 0.001
                        if d_x < height - 0.03:
                            print("dx: ", d_x, "height: ", height)
                            tuck_in = True
                            d_x = height+0.3
                        dq = kinematics.all_joints(d_x, d_y, self.sec_1_length, self.sec_2_length, self.sec_3_length,
                                                   self.head_length, round(self.position_sensors[1].getValue(), 2),
                                                   round(self.position_sensors[2].getValue(), 2),
                                                   round(self.position_sensors[3].getValue(), 2),
                                                   round(self.position_sensors[4].getValue(), 2))
                        print(d_y)
                        print(d_x)
                        self.rotational_motors[1].setPosition(dq[0])
                        self.rotational_motors[2].setPosition(dq[1])
                        self.rotational_motors[3].setPosition(dq[2])
                        self.rotational_motors[4].setPosition(dq[3])
                        continue
                if (round(self.position_sensors[1].getValue(), 2) == round(dq[0], 2) and
                        round(self.position_sensors[2].getValue(), 1) == round(dq[1], 1) and
                        round(self.position_sensors[3].getValue(),1) == round(dq[2],1) and
                        round(self.position_sensors[4].getValue(), 1) == round(dq[3], 1)):
                    if tuck_in:
                        print("tuck in activated")
                        return
                    print('Curr1: ', round(self.position_sensors[1].getValue(), 2))
                    print('Curr2: ', round(self.position_sensors[2].getValue(), 1))
                    print('Curr3: ', round(self.position_sensors[3].getValue(), 1))
                    print('Curr4: ', round(self.position_sensors[4].getValue(), 1))
                    break
            height = d_x
        print("for some reason I decided to give up on life, dy is probs issue: ", d_y)
        return

    def tuck_in_action(self):
        # Set the arm to tuck in position
        pos1 = 0.0
        pos2 = 3.0
        pos3 = -2.9
        last_joint_set = False
        self.rotational_motors[3].setPosition(pos3)
        self.rotational_motors[3].setVelocity(1)
        while self.robot.step(self.time_step) != 1:
            if last_joint_set:
                if (round(self.position_sensors[1].getValue(), 2) == round(pos1, 2) and
                        round(self.position_sensors[2].getValue(), 2) == round(pos2, 2) and round(
                            self.position_sensors[3].getValue(), 2) == round(pos3, 2)):
                    print('Finished tuck in')
                    return
                else:
                    continue

            if round(self.position_sensors[3].getValue(), 2) > -0.1 and round(self.position_sensors[3].getValue(), 2) < 0.1:
                self.rotational_motors[1].setPosition(pos1)
                self.rotational_motors[2].setPosition(pos2)
                self.rotational_motors[1].setVelocity(1)
                self.rotational_motors[2].setVelocity(0.8)
                last_joint_set = True


    def is_stationary(self, last_4_joint_positions):
        temp = last_4_joint_positions.copy()
        if len(temp) == 4:
            last_joints = temp.pop()
            while len(temp) > 0:
                next_joints = temp.pop()
                if round(last_joints[0], 3) != round(next_joints[0], 3) or round(last_joints[1],3) != round(next_joints[1],3) or round(last_joints[2],3) != round(next_joints[2],3):
                    return False
                last_joints = next_joints
            return True
        return False

    def set_sweeping_action_sensor(self,distance_to_wall, height=-0.07):
        # Set the arm from tuck in position to the top edge of the table for upcoming sweep action
        print('Detected distance to wall is {} and setting it to be d_y'.format(distance_to_wall))
        # Robot upper body has width = 0.3
        d_y = distance_to_wall + 0.15
        print('real d_y after adding half body width', d_y)
        if (d_y > 1.45):
            print('The distance is longer than the arm')
            print('d_y now set to 1.4')
            d_y = 1.45
        d_x = height
        print("d_x is", d_x)
        print(self.pressure_sensors[0].getValue())
        while self.pressure_sensors[0].getValue() < 1.0:
            print("Pressure is ", self.pressure_sensors[0].getValue())
            k = kinematics.all_joints(d_x, d_y, self.sec_1_length, self.sec_2_length,self.sec_3_length,self.head_length)
            print('k: ', k)
            print("type is: ", k.dtype)
            # pos1 = 1.44
            pos1 = k[0]
            # pos2 = 0.64
            pos2 = k[1]
            print("head rotation at start is = ", self.position_sensors[4].getValue())
            pos3 = k[2]
            pos4 = k[3]
            # pos2 = 0.5
            vel1 = 0.25
            vel2 = 0.8
            vel3 = 1
            vel4 = 1
            print(type(pos1))
            self.rotational_motors[1].setPosition(pos1)
            self.rotational_motors[2].setPosition(pos2)
            self.rotational_motors[3].setPosition(pos3)
            self.rotational_motors[4].setPosition(pos4)
            self.rotational_motors[1].setVelocity(vel1)
            self.rotational_motors[2].setVelocity(vel2)
            self.rotational_motors[3].setVelocity(vel3)
            self.rotational_motors[4].setVelocity(vel4)
            # # print("TORQUE", self.rotational_motors[1].getAvailableTorque())
            # # print("HIGHEST_TORQUE", self.rotational_motors[1].getMaxTorque())
            # # print("FORCE", self.rotational_motors[1].getAvailableForce())
            # # print("MAX FORCE", self.rotational_motors[1].getAvailableTorque())
            while self.robot.step(self.time_step) != -1:
                print("pressure is", self.pressure_sensors[0].getValue())
                self.last_4_positions.append([self.position_sensors[1].getValue(),self.position_sensors[2].getValue(), self.position_sensors[3].getValue()])
                if self.pressure_sensors[0].getValue() > 1.0:
                    kin = kinematics.kinematics4joint(self.position_sensors[1].getValue(),
                                                      self.position_sensors[2].getValue(),
                                                      self.position_sensors[3].getValue(),
                                                      self.position_sensors[4].getValue(), self.sec_1_length,
                                                      self.sec_2_length, self.sec_3_length, self.head_length)
                    print("pressure sensor value above 1.0")
                    return kin[7], kin[3]
                if (round(self.position_sensors[1].getValue(), 2) == round(pos1, 2) and
                        round(self.position_sensors[2].getValue(), 2) == round(pos2, 2) and
                        round(self.position_sensors[3].getValue(), 2) == round(pos3,2) and
                        round(self.position_sensors[4].getValue(), 2) == round(pos4, 2)):
                    print("found it! Should probs keep going down tho")
                    break
                elif self.is_stationary(self.last_4_positions):
                    print("stationary for a bit")
                    print(self.rotational_motors[1].getVelocity())
                    print(self.rotational_motors[2].getVelocity())
                    print("head rotation is = ", self.position_sensors[4].getValue())

                    kin = kinematics.kinematics4joint(self.position_sensors[1].getValue(),
                                                      self.position_sensors[2].getValue(),
                                                      self.position_sensors[3].getValue(),
                                                      self.position_sensors[4].getValue(), self.sec_1_length,
                                                      self.sec_2_length, self.sec_3_length, self.head_length)
                    print("returning coz stationary")
                    return kin[7], kin[3]
            d_x -= 0.03
        kin = kinematics.kinematics4joint(self.position_sensors[1].getValue(), self.position_sensors[2].getValue(),
                                          self.position_sensors[3].getValue(), self.position_sensors[4].getValue(),
                                          self.sec_1_length, self.sec_2_length, self.sec_3_length, self.head_length)
        print("Happens when pressure is greater than 1.0")
        return kin[7], kin[3]


    def set_sweeping_action(self, distance_to_wall, height=-0.20):
        # Set the arm from tuck in position to the top edge of the table for upcoming sweep action
        print('Detected distance to wall is {} and setting it to be d_y'.format(distance_to_wall))
        # Robot upper body has width = 0.3
        d_y = distance_to_wall + 0.15
        print('real d_y after adding half body width', d_y)
        if (d_y > 1.45):
            print('The distance is longer than the arm')
            print('d_y now set to 1.4')
            d_y = 1.45
        d_x = height
        k = kinematics.all_joints(d_x, d_y, self.sec_1_length, self.sec_2_length,self.sec_3_length,self.head_length)
        print('k: ', k)
        print("type is: ", k.dtype)
        # pos1 = 1.44
        pos1 = k[0]
        # pos2 = 0.64
        pos2 = k[1]
        print("head rotation at start is = ", self.position_sensors[4].getValue())
        pos3 = k[2]
        pos4 = k[3]
        # pos2 = 0.5
        vel1 = 0.25
        vel2 = 0.8
        vel3 = 1
        vel4 = 1
        print(type(pos1))
        self.rotational_motors[1].setPosition(pos1)
        self.rotational_motors[2].setPosition(pos2)
        self.rotational_motors[3].setPosition(pos3)
        self.rotational_motors[4].setPosition(pos4)
        self.rotational_motors[1].setVelocity(vel1)
        self.rotational_motors[2].setVelocity(vel2)
        self.rotational_motors[3].setVelocity(vel3)
        self.rotational_motors[4].setVelocity(vel4)
        # # print("TORQUE", self.rotational_motors[1].getAvailableTorque())
        # # print("HIGHEST_TORQUE", self.rotational_motors[1].getMaxTorque())
        # # print("FORCE", self.rotational_motors[1].getAvailableForce())
        # # print("MAX FORCE", self.rotational_motors[1].getAvailableTorque())
        while self.robot.step(self.time_step) != -1:
            self.last_4_positions.append([self.position_sensors[1].getValue(),self.position_sensors[2].getValue(), self.position_sensors[3].getValue()])
            if (round(self.position_sensors[1].getValue(), 2) == round(pos1, 2) and
                    round(self.position_sensors[2].getValue(), 2) == round(pos2, 2) and
                    round(self.position_sensors[3].getValue(), 2) == round(pos3,2) and
                    round(self.position_sensors[4].getValue(), 2) == round(pos4, 2)) or \
                    self.is_stationary(self.last_4_positions):
                print("bingo, Setting done")
                print(self.rotational_motors[1].getVelocity())
                print(self.rotational_motors[2].getVelocity())
                print("head rotation is = ", self.position_sensors[4].getValue())
                kin = kinematics.kinematics4joint(self.position_sensors[1].getValue(),self.position_sensors[2].getValue(), self.position_sensors[3].getValue(),self.position_sensors[4].getValue(),self.sec_1_length,self.sec_2_length,self.sec_3_length, self.head_length)
                return kin[7], kin[3]

    def set_button_click(self, d_height, d_length,d_z):
        # Set the arm from tuck in position to the top edge of the table for upcoming sweep action
        print('Detected distance to wall is {} and setting it to be d_y'.format(d_length))
        # Robot upper body has width = 0.3
        d_y = d_length
        print('real d_y after adding half body width', d_y)
        if (d_y > 1.45):
            print('The distance is longer than the arm')
            print('d_y now set to 1.4')
            d_y = 1.45
        d_x = d_height
        d_z = d_z
        k = kinematics.joints_button(d_x, d_y,d_z,0.0,self.sec_1_length, self.sec_2_length,self.sec_3_length+0.07, 0.20)
        print('k: ', k)
        print("type is: ", k.dtype)
        # pos1 = 1.44
        pos0 = k[0]
        pos1 = k[1]
        # pos2 = 0.64
        pos2 = k[2]
        print("head rotation at start is = ", self.position_sensors[4].getValue())
        pos3 = k[3]
        pos4 = 0.0
        # pos2 = 0.5
        vel0 = 0.5
        vel1 = 0.1
        vel2 = 1
        vel3 = 1
        vel4 = 1
        print(type(pos1))
        self.rotational_motors[0].setPosition(pos0)
        self.rotational_motors[1].setPosition(pos1)
        self.rotational_motors[2].setPosition(pos2)
        self.rotational_motors[3].setPosition(pos3)
        self.rotational_motors[4].setPosition(pos4)
        self.rotational_motors[0].setVelocity(vel0)
        self.rotational_motors[1].setVelocity(vel1)
        self.rotational_motors[2].setVelocity(vel2)
        self.rotational_motors[3].setVelocity(vel3)
        self.rotational_motors[4].setVelocity(vel4)
        # # print("TORQUE", self.rotational_motors[1].getAvailableTorque())
        # # print("HIGHEST_TORQUE", self.rotational_motors[1].getMaxTorque())
        # # print("FORCE", self.rotational_motors[1].getAvailableForce())
        # # print("MAX FORCE", self.rotational_motors[1].getAvailableTorque())
        while self.robot.step(self.time_step) != -1:
            self.last_4_positions.append([self.position_sensors[1].getValue(),self.position_sensors[2].getValue(), self.position_sensors[3].getValue()])
            if (round(self.position_sensors[1].getValue(), 2) == round(pos1, 2) and
                    round(self.position_sensors[2].getValue(), 2) == round(pos2, 2) and
                    round(self.position_sensors[3].getValue(), 2) == round(pos3,2) and
                    round(self.position_sensors[4].getValue(), 2) == round(pos4, 2)) or \
                    self.is_stationary(self.last_4_positions):
                print("bingo, Setting done")
                print(self.rotational_motors[1].getVelocity())
                print(self.rotational_motors[2].getVelocity())
                print("head rotation is = ", self.position_sensors[4].getValue())

                return
    def sweep(self, distance_to_wall, table_height = -0.20):
        # It assumes the arm is tucked in and try to do the sweep action from reaching the
        # top end of the table. After it finishes, it returns to tuck in positions
        print("heeeeeeyaaaaaaaaaaa")
        distance_to_wall, height = self.set_sweeping_action_sensor(distance_to_wall, table_height)
        print("pumpkin: ", self.pressure_sensors[0].getValue())
        print("sweeping action about to begin with height = ", height)
        self.sweep_action(distance_to_wall, height)
        self.tuck_in_action()

