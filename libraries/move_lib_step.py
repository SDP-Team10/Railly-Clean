from numpy import inf
from controller import Robot, Keyboard
import math

MAX_SPEED = 0       # maximum velocity of the turtlebot motors ( set to motors max velocity in setup() )
TURN_MULT = 0.25    # constant to slow down turn speed ( too high and turning will be inaccurate)
MOVE_MULT = 0.25    # constant to slow down move speed ( too high and moving will be inaccurate)
TIME_STEP = 0       # constant to control time step ( set to robots basic time step in setup() )

motor_names = ['wheel_left_joint', 'wheel_right_joint']
grobot = None
gleft_motor = None
gright_motor = None


######################   Setup   #########################

def set_turn_mult(tm=0.25):
    global TURN_MULT
    TURN_MULT = tm
    return True


def set_move_mult(mm=0.25):
    global MOVE_MULT
    MOVE_MULT = mm
    return True


def setup(robot):
    global grobot
    global gleft_motor
    global gright_motor
    global MAX_SPEED
    global TIME_STEP
    grobot = robot
    TIME_STEP = int(robot.getBasicTimeStep())
    gleft_motor = grobot.getDevice(motor_names[0])
    gright_motor = grobot.getDevice(motor_names[1])
    gleft_motor.getPositionSensor().enable(TIME_STEP)
    gright_motor.getPositionSensor().enable(TIME_STEP)
    if gleft_motor is None or gright_motor is None:
        print('Cant find left or right motors: %s and %s' % (motor_names[0], motor_names[1]))
        return False
    MAX_SPEED = gleft_motor.getMaxVelocity()
    enable_speed_control(gleft_motor)
    enable_speed_control(gright_motor)
    return True

##############################################################




###############   Velocity Control Functions   ###############

def get_left_speed(robot):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    return gleft_motor.getVelocity()


def get_right_speed(robot):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    return gright_motor.getVelocity()


def turn_right(robot, left_vel=-1, right_vel=-1):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    new_left_vel = MAX_SPEED if left_vel == -1 else left_vel
    new_right_vel = -MAX_SPEED if right_vel == -1 else right_vel
    limit_vel(new_left_vel*TURN_MULT, new_right_vel*TURN_MULT)


def turn_left(robot, left_vel=-1, right_vel=-1):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    new_left_vel = -MAX_SPEED if left_vel == -1 else left_vel
    new_right_vel = MAX_SPEED if right_vel == -1 else right_vel
    limit_vel(new_left_vel*TURN_MULT, new_right_vel*TURN_MULT)


def move_forward(robot, speed=-1):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    new_speed = MAX_SPEED if speed == -1 else speed
    limit_vel(new_speed*MOVE_MULT, new_speed*MOVE_MULT)


def move_back(robot, speed=-1):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    new_speed = MAX_SPEED if speed == -1 else speed
    limit_vel(-new_speed*MOVE_MULT, -new_speed*MOVE_MULT)

##############################################################




###############   Position Control Functions   ###############

### robot parameters ( HARD CODED MEASUREMENTS )

wheel_radius = 0.0985
distance_between_wheels = 0.404
deg_to_rad = math.pi / 180
wheel_circ = 2 * math.pi * wheel_radius

eps = 0.5  # how close motors position needs to be to target to stop ( needs to be quite high )

# offsets needed to keep non-resettable position values to 0
left_offset = 0
right_offset = 0

###


def check_move(robot, left_target, right_target):
    global left_offset
    global right_offset
    print('TARGETS: ', left_target, right_target)
    while True:
        robot.step(TIME_STEP)
        left_pos = gleft_motor.getPositionSensor().getValue()
        right_pos = gright_motor.getPositionSensor().getValue()
        print(left_pos, right_pos, left_offset, right_offset)
        if abs(left_pos-left_target) < eps and abs(right_pos-right_target) < eps:
            left_offset = left_pos
            right_offset = right_pos
            return True


def turn_angle(robot, angle=90):
    global left_offset
    global right_offset
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False

    rads = abs(angle) * deg_to_rad * 2
    sector_len = rads * distance_between_wheels / 2
    pos = sector_len / wheel_radius
    turn_sign = angle / abs(angle)
    if turn_sign == -1:
        gleft_motor.setPosition(left_offset)
        gright_motor.setPosition(right_offset + pos)
        limit_vel(MAX_SPEED*TURN_MULT, MAX_SPEED*TURN_MULT)
        return check_move(robot, left_offset, right_offset + pos)
    else:
        gleft_motor.setPosition(left_offset + pos)
        gright_motor.setPosition(right_offset)
        limit_vel(MAX_SPEED*TURN_MULT, MAX_SPEED*TURN_MULT)
        return check_move(robot, left_offset + pos, right_offset)


def move_distance(robot, dist=1):
    global left_offset
    global right_offset
    if dist < 0:
        return False
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    pos = dist / wheel_radius
    gleft_motor.setPosition(left_offset + pos)
    gright_motor.setPosition(right_offset + pos)
    limit_vel(MAX_SPEED*MOVE_MULT, MAX_SPEED*MOVE_MULT)
    return check_move(robot, left_offset + pos, right_offset + pos)

#############################################################




###################   Utility Functions   ###################

def stop(robot):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    enable_speed_control(gleft_motor)
    enable_speed_control(gright_motor)


def enable_speed_control(motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0)


def limit_vel(left_vel, right_vel):
    new_left_vel = MAX_SPEED if left_vel > MAX_SPEED else -MAX_SPEED if left_vel < -MAX_SPEED else left_vel
    new_right_vel = MAX_SPEED if right_vel > MAX_SPEED else -MAX_SPEED if right_vel < -MAX_SPEED else right_vel
    gleft_motor.setVelocity(new_left_vel)
    gright_motor.setVelocity(new_right_vel)

#############################################################
