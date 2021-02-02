"""raillyCleanController controller."""


from controller import Robot, DistanceSensor

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = 64
# list to hold distance sensors
ds = []
# Add number and names of the distance sensors - front sensor must be at zero
ds_num = 3
ds_names = ["front distance sensor", "left distance sensor", "right distance sensor"]

# enable all distance sensors
for i in range(ds_num):
    ds.append(robot.getDevice(ds_names[i]))
    ds[i].enable(timestep)


def should_stop():
    """function shouldStop
    params: none
    returns: boolean
    Checks front distance sensor, if distance is <40cm, return true, otherwise false 
    """
    # 0.4 is placeholder, need to set up "lookupTable field
    return ds[0].getValue < 4e4


if __name__ == "__main__":
    # Main loop: perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()

        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass

# Enter here exit cleanup code.
