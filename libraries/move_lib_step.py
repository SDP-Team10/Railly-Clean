from numpy import inf
from controller import Robot, Keyboard
import math
from libraries import side_check as sc

MAX_SPEED = 0       ### maximum velocity of the turtlebot motors ( set to motors max velocity in setup() )
TURN_MULT = 0.25    ### constant to slow down turn speed ( too high and turning will be inaccurate)
MOVE_MULT = 0.25    ### constant to slow down move speed ( too high and moving will be inaccurate)
TIME_STEP = 0       ### constant to control time step ( set to robots basic time step in setup() )

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

eps = .01 ### how close motors position needs to be to target to stop ( necessary so check_move knows when to return )

### offsets needed to keep non-resettable position values to 0
left_offset = 0
right_offset = 0
###

def check_move(robot, left_start, right_start, pos):
    left_pos = 0
    right_pos = 0
    while True:
        robot.step(TIME_STEP)
        prev_left_pos = left_pos
        prev_right_pos = right_pos
        left_pos = gleft_motor.getPositionSensor().getValue()
        right_pos = gright_motor.getPositionSensor().getValue()
        close_to_pos = abs(abs(left_pos-left_start) + abs(right_pos-right_start) - abs(pos)*2) < eps
        slowing_down = abs(abs(prev_left_pos-left_pos) + abs(prev_right_pos-right_pos)) < eps/2
        if  close_to_pos and slowing_down:
            print('STOPPING')
            stop(robot)
            return True

def turn_angle(robot, angle):
    global left_offset
    global right_offset
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    left_offset = gleft_motor.getPositionSensor().getValue()
    right_offset = gright_motor.getPositionSensor().getValue()
    rads = abs(angle) * deg_to_rad * 1.1
    sector_len = rads * distance_between_wheels / 2
    pos = sector_len / wheel_radius
    turn_sign = angle/abs(angle)
    if turn_sign == -1:
        gright_motor.setPosition(right_offset + pos)
        gleft_motor.setPosition(left_offset - pos)
        limit_vel(MAX_SPEED*TURN_MULT, MAX_SPEED*TURN_MULT)
        check_move(robot, left_offset, right_offset, pos)
    else:
        gright_motor.setPosition(right_offset - pos)
        gleft_motor.setPosition(left_offset + pos)
        limit_vel(MAX_SPEED*TURN_MULT, MAX_SPEED*TURN_MULT)
        check_move(robot, left_offset, right_offset, pos)

def move_distance(robot, dist=1):
    global left_offset
    global right_offset
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    left_offset = gleft_motor.getPositionSensor().getValue()
    right_offset = gright_motor.getPositionSensor().getValue()
    pos = dist / wheel_radius
    gleft_motor.setPosition(left_offset + pos)
    gright_motor.setPosition(right_offset + pos)
    limit_vel(MAX_SPEED*MOVE_MULT,MAX_SPEED*MOVE_MULT)
    return check_move(robot, left_offset, right_offset, pos)

def check_move_side_check(robot, left_start, right_start, pos, table_check, dist_sensor):
    left_pos = 0
    right_pos = 0
    table_length, pole_length = None, None
    while True:
        robot.step(TIME_STEP)

        if not table_length:
            table_length, pole_length = table_check.side_check(dist_sensor)

        prev_left_pos = left_pos
        prev_right_pos = right_pos
        left_pos = gleft_motor.getPositionSensor().getValue()
        right_pos = gright_motor.getPositionSensor().getValue()
        close_to_pos = abs(abs(left_pos-left_start) + abs(right_pos-right_start) - pos*2) < eps
        slowing_down = abs(abs(prev_left_pos-left_pos) + abs(prev_right_pos-right_pos)) < eps/2
        if  close_to_pos and slowing_down:
            stop(robot)
            print('STOPPING SIDE CHECK MOVE')
            return table_length, pole_length


def move_distance_check_sides(robot, dist, table_check, dist_sensor):
    global left_offset
    global right_offset
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    left_offset = gleft_motor.getPositionSensor().getValue()
    right_offset = gright_motor.getPositionSensor().getValue()
    pos = dist / wheel_radius
    gleft_motor.setPosition(left_offset + pos)
    gright_motor.setPosition(right_offset + pos)
    limit_vel(MAX_SPEED*MOVE_MULT,MAX_SPEED*MOVE_MULT)
    return check_move_side_check(robot, left_offset, right_offset, pos, table_check, dist_sensor)


#############################################################




###################   Utility Functions   ###################

def stop(robot):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    gleft_motor.setVelocity(0)
    gright_motor.setVelocity(0)
    while gleft_motor.getVelocity() > 0 and gright_motor.getVelocity() > 0:
        robot.step(TIME_STEP)


def enable_speed_control(motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0)


def limit_vel(left_vel, right_vel):
    new_left_vel = MAX_SPEED if left_vel > MAX_SPEED else -MAX_SPEED if left_vel < -MAX_SPEED else left_vel
    new_right_vel = MAX_SPEED if right_vel > MAX_SPEED else -MAX_SPEED if right_vel < -MAX_SPEED else right_vel
    gleft_motor.setVelocity(new_left_vel)
    gright_motor.setVelocity(new_right_vel)

#############################################################
