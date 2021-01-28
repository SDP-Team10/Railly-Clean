# """aisle_navigation controller."""
import math
import time
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor


# create the Robot instance.
robot = Robot()

# parameters for robot's world
TIME_STEP = 64
MAX_SPEED = 6.28
timestep = int(robot.getBasicTimeStep())

# get the required devices for the rebot
leftMotor = robot.getDevice('wheel_right_joint')
rightMotor = robot.getDevice('wheel_left_joint')

leftPs = robot.getDevice('wheel_left_joint_sensor')
rightPs = robot.getDevice('wheel_right_joint_sensor')

frontDs = robot.getDevice('front distance sensor')
backDs = robot.getDevice('back distance sensor')


leftPs.enable(TIME_STEP)
rightPs.enable(TIME_STEP)

frontDs.enable(TIME_STEP)
backDs.enable(TIME_STEP)

#set the target position of the motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))



# static parameters
wheel_radius = 0.0985
angle_of_rotation = 6.28 / 4 # rotate 90 degree


linear_velocity = wheel_radius * MAX_SPEED
distance_between_wheels = 0.404
rate_of_rotation = (2* linear_velocity)/distance_between_wheels #circular motion
duration_turn = angle_of_rotation/rate_of_rotation
duration_time = 0


print(duration_turn)

## Initial speed
speed = 0.5 * MAX_SPEED
left_speed = 0
right_speed = 0

# flags
stop = False
turn = False

detected = False
origin = False
calculate = False


def robot_go_forward():
    left_speed = speed
    right_speed = speed
    return left_speed, right_speed
   
    
def robot_go_back():
    print('Going back')
    left_speed = -speed
    right_speed = -speed
    return left_speed, right_speed    
    
def robot_stop():
    print('stop')
    time.sleep(0.5)
    left_speed = 0
    right_speed = 0
    return left_speed, right_speed
    
    
def calculate_turn_duration(current_time):
    rot_start_time = current_time
    duration_time = rot_start_time + duration_turn
    return duration_time
    
def robot_turn():
    left_speed = -MAX_SPEED
    right_speed = MAX_SPEED
    return left_speed, right_speed
    
def detect_front_object():
    if (frontDs.getValue() < 5e+4):
        return True

    

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    current_time = robot.getTime()
    
    print('Wheel L {}   R {}\n'.format(leftPs.getValue(),rightPs.getValue()))
    # print('detection: {}'.format(frontDs.getValue))
    # print('look up, ', frontDs.getLookupTable())
    # front_distance = frontDs.getValue()
    # print('fd ', front_distance)
    ## Use Case 1
    # if (not detected and not stop):
        # if int(leftPs.getValue()) == 5:
            # detected = True
            # stop = True
        # else:
            # left_speed, right_speed = robot_go_forward()
    # else:
        # if (stop and not origin):
            # left_speed, right_speed = robot_stop()
            # stop = False
        # if (leftPs.getValue() > 0):
            # left_speed, right_speed = robot_go_back()
            # leftMotor.setVelocity(left_speed)
            # rightMotor.setVelocity(right_speed)
        # elif (leftPs.getValue() < 0):
            # left_speed, right_speed = robot_stop()
     
     ## Use Case 2
    # if (not detected):
        # if int(leftPs.getValue()) == 5:
            # detected = True
            # turn = True
            # stop = True
            # print('in')
        # else:
            # left_speed, right_speed = robot_go_forward()
    
    # if (detected and turn):
        # if (stop):
            # left_speed, right_speed = robot_stop()
            # stop = False

        # if (not calculate):
            # calculate = True
            # duration_time = calculate_turn_duration(current_time)
       
        # else:
            # if ( current_time < duration_time):
                # print('current time: {} Duration time: {}'.format(current_time, duration_time))
                # print('turning with MAX_SPEED: ', MAX_SPEED)
                # left_speed, right_speed = robot_turn()
            # else:
                # stop = True
                # detected = False
                # turn = False
                

      
                
            
            
            
            


    print('speed: ',left_speed)
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    
    
    
    
    # current_time = robot.getTime()

    
    # print(stop)
    # print(turn)
    # if (stop == False or turn == False):
        # print('Wheel L {}   R {}\n'.format(leftPs.getValue(),rightPs.getValue()))
        # print('Wheel R {}\n'.format(rightPs.getValue()))
        # robot_go_straight()
        
        # if (int(leftPs.getValue()) == 5 and int(rightPs.getValue()) == 5):
            # stop = True
            # robot_stop()
            
        # elif (int(leftPs.getValue()) == 15 and int(rightPs.getValue()) == 15):
            # print('now im in')
            # stop = True
            # turn = True
            # rot_start_time = current_time
            # duration_time = rot_start_time + duration_turn
            # time.sleep(1)
    # elif (turn == True):
        # print('in1?')
        # print('start: ', rot_start_time)
        # print('current ', current_time)
        # print('duration ',duration_time)
        # if (rot_start_time <= current_time < duration_time):
            # print('turning')
            # right_speed = -MAX_SPEED
            # left_speed = MAX_SPEED
        # else:
            # print(stop)
            # print(turn)
            # print(left_speed)
            # print(right_speed)
            # robot_stop()
            # print('breaked')
            # print(speed)
            # left_speed = 0 * MAX_SPEED
            # right_speed = 0 * MAX_SPEED
        
    # leftMotor.setVelocity(left_speed)
    # rightMotor.setVelocity(right_speed)
    
  
    # if (int(leftPs.getValue()) == 270 and int(rightPs.getValue()) == 270 and stop == False):
        # print('in\n')
        # stop = True
        # leftMotor.setVelocity(0)
        # rightMotor.setVelocity(0)
    # else :
        
     
    
    
    
    