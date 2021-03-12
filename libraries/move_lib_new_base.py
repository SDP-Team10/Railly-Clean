from numpy import inf
from controller import Robot, Keyboard
import math
from libraries import side_check as sc

MAX_SPEED = 0       ### maximum velocity of the turtlebot motors ( set to motors max velocity in setup() )
TURN_MULT = 0.25    ### constant to slow down turn speed ( too high and turning will be inaccurate)
MOVE_MULT = 0.25    ### constant to slow down move speed ( too high and moving will be inaccurate)
TIME_STEP = 0       ### constant to control time step ( set to robots basic time step in setup() )
NORM_SPEED = 0
WHEEL_RADIUS = 0.05

motor_names = ['wheel_1_motor', # top left
               'wheel_3_motor', # back left
               'wheel_4_motor', # top right
               'wheel_2_motor'] # back right
grobot = None
gtop_left_motor = None
gtop_right_motor = None
gback_left_motor = None
gback_right_motor = None
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
    global gtop_left_motor
    global gtop_right_motor
    global gback_left_motor
    global gback_right_motor
    global MAX_SPEED
    global NORM_SPEED
    global TIME_STEP
    grobot = robot
    TIME_STEP = int(robot.getBasicTimeStep())

    gtop_left_motor = grobot.getDevice(motor_names[0])
    gtop_right_motor = grobot.getDevice(motor_names[2])
    gback_left_motor = grobot.getDevice(motor_names[1])
    gback_right_motor = grobot.getDevice(motor_names[3])

    gtop_left_motor.getPositionSensor().enable(TIME_STEP)
    gtop_right_motor.getPositionSensor().enable(TIME_STEP)
    gback_left_motor.getPositionSensor().enable(TIME_STEP)
    gback_right_motor.getPositionSensor().enable(TIME_STEP)

    any_none = False
    if gtop_left_motor == None:
        any_none = True
        print('Cant find motor: %s' % motor_names[0])
    if gtop_right_motor == None:
        any_none = True
        print('Cant find motor: %s' % motor_names[2])
    if gback_left_motor == None:
        any_none = True
        print('Cant find motor: %s' % motor_names[1])
    if gback_right_motor == None:
        any_none = True
        print('Cant find motor: %s' % motor_names[3])
    if any_none:
        return False

    MAX_SPEED = gtop_left_motor.getMaxVelocity() ### arbitary motor selection for max speed
    NORM_SPEED = MAX_SPEED * MOVE_MULT
    enable_speed_control()

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

wheel_radius = 0.05
distance_between_wheels = 0.58
deg_to_rad = math.pi / 180
wheel_circ = 2 * math.pi * wheel_radius

eps = .01 ### how close motors position needs to be to target to stop ( necessary so check_move knows when to return )


def check_move(robot, start_positions, pos, stop_at_end=True):
    positions = [0, 0, 0, 0]
    while True:
        robot.step(TIME_STEP)
        prev_positions = positions
        positions = get_offsets()

        close_to_pos = abs(
                       abs(positions[0]-start_positions[0]) +
                       abs(positions[1]-start_positions[1]) +
                       abs(positions[2]-start_positions[2]) +
                       abs(positions[3]-start_positions[3]) - abs(pos)*4) < eps

        slowing_down = abs(
                       abs(positions[0]-start_positions[0]) +
                       abs(positions[1]-start_positions[1]) +
                       abs(positions[2]-start_positions[2]) +
                       abs(positions[3]-start_positions[3]) - abs(pos)*4) < eps/2

        if  close_to_pos and slowing_down:
            if stop_at_end:
                print('STOPPING')
                stop(robot)
            else:
                enable_speed_control()
            return True

def turn_angle(robot, angle, stop_at_end=True):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    offsets = get_offsets()
    rads = abs(angle) * deg_to_rad * 1.075
    ###
    multiplier = 0.66
    ###
    sector_len = rads * distance_between_wheels * multiplier
    pos = sector_len / wheel_radius
    turn_sign = angle/abs(angle)
    if turn_sign == 1:
        gtop_left_motor.setPosition(offsets[0] + pos)
        gtop_right_motor.setPosition(offsets[1] - pos)
        gback_left_motor.setPosition(offsets[2] + pos)
        gback_right_motor.setPosition(offsets[3] - pos)
        limit_vel(MAX_SPEED*TURN_MULT)
        check_move(robot, offsets, pos, stop_at_end)
    else:
        gtop_left_motor.setPosition(offsets[0] - pos)
        gtop_right_motor.setPosition(offsets[1] + pos)
        gback_left_motor.setPosition(offsets[2] - pos)
        gback_right_motor.setPosition(offsets[3] + pos)
        limit_vel(MAX_SPEED*TURN_MULT)
        check_move(robot, offsets, pos, stop_at_end)

def move_distance(robot, dir, dist=1):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    offsets = get_offsets()
    pos = dist / wheel_radius
    if dir=='forward':
        gtop_left_motor.setPosition(offsets[0] + pos)
        gtop_right_motor.setPosition(offsets[1] + pos)
        gback_left_motor.setPosition(offsets[2] + pos)
        gback_right_motor.setPosition(offsets[3] + pos)
    if dir=='side':
        gtop_left_motor.setPosition(offsets[0] + pos)
        gtop_right_motor.setPosition(offsets[1] - pos)
        gback_left_motor.setPosition(offsets[2] - pos)
        gback_right_motor.setPosition(offsets[3] + pos)
    limit_vel(MAX_SPEED*MOVE_MULT)
    return check_move(robot, offsets, pos)


###
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
###

#############################################################




###################   Utility Functions   ###################

def stop(robot):
    if robot != grobot or grobot is None:
        if not setup(robot):
            print('Error in setting up robot')
            return False
    gtop_left_motor.setVelocity(0)
    gtop_right_motor.setVelocity(0)
    gback_left_motor.setVelocity(0)
    gback_right_motor.setVelocity(0)
    #while gleft_motor.getVelocity() > 0 or gright_motor.getVelocity() > 0:
    #    robot.step(TIME_STEP)
    enable_speed_control()

def enable_speed_control():
    gtop_left_motor.setPosition(float('inf'))
    gtop_left_motor.setVelocity(0)
    gtop_right_motor.setPosition(float('inf'))
    gtop_right_motor.setVelocity(0)
    gback_left_motor.setPosition(float('inf'))
    gback_left_motor.setVelocity(0)
    gback_right_motor.setPosition(float('inf'))
    gback_right_motor.setVelocity(0)

def limit_vel(vel):
    new_vel = MAX_SPEED if vel > MAX_SPEED else -MAX_SPEED if vel < -MAX_SPEED else vel
    gtop_left_motor.setVelocity(new_vel)
    gtop_right_motor.setVelocity(new_vel)
    gback_left_motor.setVelocity(new_vel)
    gback_right_motor.setVelocity(new_vel)

def get_offsets():
    return ( gtop_left_motor.getPositionSensor().getValue(),
             gtop_right_motor.getPositionSensor().getValue(),
             gback_left_motor.getPositionSensor().getValue(),
             gback_right_motor.getPositionSensor().getValue() )

###############################################################
