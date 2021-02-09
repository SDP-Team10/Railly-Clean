"""arm_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor

robot = Robot()

class ArmController(object):
    def __init__(self, robot=robot):
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
        self.left_wipe = False
        self.right_wipe = False
        self.wipe_velocity = 5
        self.left_goal = [1.2, 1.2, 1.3, 1.35]
        self.right_goal = [0.3, 0.3, 0.1, 0.05]

        self.wipe = True
        self.move = False
        self.counter = 0

        self.sec_1_ready = False
        self.sec_2_ready = False
        self.head_ready = False

        self.sec_1_movement = [1.1, 1.1, 0.9, 0.8]
        self.sec_2_movement = [0.9, 1.3, 2, 2]
        self.head_movement = [-0.4, -0.6, -1, -0.5]

        self.sec_1_store = [1.12, 0.66]
        self.sec_2_store = [1.02, 0.92]
        self.head_store = [-0.45, -0.66]

        self.set_env = False

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
            # motor.enable(self.time_step)
            rotational_motors.append(motor)
        return rotational_motors

    def sweep(self):
        # Version 3 - Left and Right

        # 1st Iteration
        # print(self.left_goal)
        base_position = round(self.position_sensors[0].getValue(),1)
        print('current: ',base_position)
        if (not self.left_wipe and not self.right_wipe):
            if (base_position < self.left_goal[self.counter]):
                print(self.left_goal[self.counter])
                print("{} + 0.1 ".format(base_position))
                base_position += 0.1
                print('now base pos: ', base_position)
                print('left wipe: ', self.left_wipe)
            if (base_position == self.left_goal[self.counter]):
                self.left_wipe = True
        if (not self.right_wipe and self.left_wipe):
            if (base_position > self.right_goal[self.counter]):
                base_position -= 0.1
                print('now head pos2: ', base_position)
                print('Trying to do right wipe')
            if (base_position == self.right_goal[self.counter]):
                self.right_wipe = True 
        elif (self.right_wipe and self.left_wipe):
            print('Go back to origin')
            if (base_position != 0.7):
                base_position += 0.1
            else:
                print('Done 1 iteration')
                self.wipe = False
                self.left_wipe = False
                self.right_wipe = False
                self.sec_1_ready = False
                self.sec_2_ready = False
                self.head_ready = False
                # self.counter += 1
                
        print('Updating value: ', base_position)
        self.rotational_motors[0].setPosition(base_position)
        self.rotational_motors[0].setVelocity(self.wipe_velocity)

        

    def test_next_iteration(self):
        if (round(self.position_sensors[1].getValue(),1) > self.sec_1_movement[self.counter]):
            self.rotational_motors[1].setPosition(self.position_sensors[1].getValue() -0.2)
        else:
            self.sec_1_ready = True
        if (round(self.position_sensors[2].getValue(),1) < self.sec_2_movement[self.counter]):
            # print("target: ",self.sec_2_movement[self.counter])
            print("now adding 0.2 to {}".format(self.position_sensors[2].getValue()))
            self.rotational_motors[2].setPosition(self.position_sensors[2].getValue() + 0.2)
        else:
            self.sec_2_ready = True
        if (round(self.position_sensors[3].getValue(),1) > self.head_movement[self.counter]):
            self.rotational_motors[3].setPosition(self.position_sensors[3].getValue() - 0.1)
        else:
            self.head_ready = True
        print("{} {} {}".format(self.sec_1_ready, self.sec_2_ready, self.head_ready) )
        if (self.sec_1_ready == self.sec_2_ready == self.head_ready == True):
            print('moved for next iteration')
            self.wipe = True
            self.counter += 1
        


        

    def prepare_next_iteration(self): # Test function to tuck in the arm a bit 
        print(self.position_sensors[1].getValue())
        print(self.position_sensors[2].getValue())
        # self.rotational_motors[1].setPosition(self.position_sensors[1].getValue()-0.005)
        # self.rotational_motors[2].setPosition(self.position_sensors[2].getValue()+5)
        # self.rotational_motors[3].setPosition(self.position_sensors[3].getValue())
        # # self.rotational_motors[3].setVelocity(0.5)
        self.rotational_motors[1].setPosition(self.sec_1_movement[1])
        self.rotational_motors[2].setPosition(self.sec_2_movement[1])
        self.rotational_motors[3].setPosition(self.head_movement[1])

        print('-------------------------------------')

    
        


# Main
if __name__ == "__main__":
    arm_controller = ArmController()


    while arm_controller.robot.step(64) != -1:
        # arm_controller.prepare_next_iteration()
        # arm_controller.test_next_iteration()
        
        print('base_sensor: ',arm_controller.position_sensors[0].getValue())
        print('sec_1_sensor: ',arm_controller.position_sensors[1].getValue())
        print('sec_2_sensor: ',arm_controller.position_sensors[2].getValue())
        print('head_sensor: ',arm_controller.position_sensors[3].getValue())
        print('------------------------------------------')

        print(arm_controller.counter)
        if (arm_controller.counter <= 3):
            if (arm_controller.wipe):
                print('wiping? ',arm_controller.wipe)
                arm_controller.sweep()
                
            elif (not arm_controller.wipe):
                print('next iteration')
                arm_controller.test_next_iteration()
        else:
            break
        

        
        # print('Real: ',arm_controller.base_sensor.getValue())
        # print('position: ', arm_controller.position_sensor.getValue())
        # arm_controller.sweep()
        # arm_controller.stretched()
        # arm_controller.sec1_motor.setPosition(0.5)
        # arm_controller.sec2_motor.setPosition(5)
        # arm_controller.base_motor.setPosition(1.1)
        # arm_controller.sec2_motor.setPosition(0)


# Enter here exit cleanup code.
        
 