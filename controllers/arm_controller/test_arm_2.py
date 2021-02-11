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

        self.table_top_sec_1 = 0
        self.table_bottom_sec_1 = 0.84
        self.table_top_sec_2 = 0
        self.table_bottom_sec_2 = 2.3

        self.tuck_in = False
        self.on_table = False
        self.can_wipe = False
        
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
   
    def check_on_table(self):
        if (round(self.position_sensors[1].getValue(),2) == 1.53 and round(self.position_sensors[2].getValue(),1) == 0.5):
            self.on_table = True
            self.can_wipe = True
            self.table_top_sec_1 = self.position_sensors[1].getValue()
            self.table_top_sec_2 = self.position_sensors[2].getValue()

    def sweep_action(self):
        arm_controller.rotational_motors[1].setPosition(self.table_bottom_sec_1)
        arm_controller.rotational_motors[2].setPosition(self.table_bottom_sec_2)
        arm_controller.rotational_motors[1].setVelocity(0.3)
        arm_controller.rotational_motors[2].setVelocity(0.5)
        if (round(self.position_sensors[1].getValue(),2) == self.table_bottom_sec_1 and round(self.position_sensors[2].getValue(),2) == self.table_bottom_sec_2):
            print('Finished sweeping')
            self.can_wipe = False
            self.tuck_in = True

    def tuck_in_action(self):
        self.on_table = False
        arm_controller.rotational_motors[1].setPosition(0.5)
        arm_controller.rotational_motors[2].setPosition(2.7)
        arm_controller.rotational_motors[1].setVelocity(0.3)
        arm_controller.rotational_motors[2].setVelocity(0.5)
        if (round(self.position_sensors[1].getValue(),2) == 0.5 and round(self.position_sensors[2].getValue(),2) == 2.7):
            print('Finished tuck in')
            self.can_wipe = True
            self.tuck_in = False

    def set_sweeping_action(self):
        arm_controller.rotational_motors[1].setPosition(1.5)
        arm_controller.rotational_motors[2].setPosition(0.5)
        arm_controller.rotational_motors[1].setVelocity(0.05)
        arm_controller.rotational_motors[2].setVelocity(0.6)
        if (round(self.position_sensors[1].getValue(),2) == 1.5 and round(self.position_sensors[2].getValue(),1) == 0.5):
            print('Ready to sweep')
            self.can_wipe = True
            self.on_table = True
            
# Main
if __name__ == "__main__":
    arm_controller = ArmController()

    while arm_controller.robot.step(64) != -1:
        
        print('base_sensor: ', arm_controller.position_sensors[0].getValue())
        print('sec_1_sensor: ', arm_controller.position_sensors[1].getValue())
        print('sec_2_sensor: ', arm_controller.position_sensors[2].getValue())
        print('head_sensor: ', arm_controller.position_sensors[3].getValue())
        print('------------------------------------------')

        arm_controller.check_on_table()
        if (arm_controller.on_table and arm_controller.can_wipe):
            print('sweeping')
            print('Goal: {} {}'.format(arm_controller.table_bottom_sec_1, arm_controller.table_bottom_sec_2))
            arm_controller.sweep_action()
          
        if (arm_controller.tuck_in):
            print('tucking in')
            arm_controller.tuck_in_action()
            print(arm_controller.on_table)
            print(arm_controller.can_wipe)

        if (not arm_controller.on_table and arm_controller.can_wipe):
            print('setting sweep position')
            arm_controller.set_sweeping_action()
 


# Enter here exit cleanup code.
        
 