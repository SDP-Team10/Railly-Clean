"""raillyCleanController controller."""


from controller import Robot, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64


ds = [] #list to hold distance sensors
#Add number and names of the distance sensors - front sensor must be at zero
dsNum = 3;
dsNames = ['front distance sensor', 'left distance sensor', 'right distance sensor']

#enable all distance sensors    
for i in range(dsNum):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)


def shouldStop():
    """function shouldStop
    params: none
    returns: boolean
    Checks front distance sensor, if distance is <40cm, return true, otherwise false """
    if(ds[0].getValue<4e+4): #0.4 is placeholder, need to set up "lookupTable field
        return True
    else:
        return False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
