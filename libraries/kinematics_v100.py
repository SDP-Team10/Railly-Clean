import numpy as np
import sympy
from sympy import Matrix, diff, sin, cos
import math
import time
import matplotlib.pyplot as plt

PI = math.pi
error = np.array([0,0])
error_button = np.array([0,0,0])
previous_time = np.array([time.time()])


def translation(theta, d , a, alpha):
    A_theta = Matrix([[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0,0,1,0], [0,0,0,1]])
    T_a = Matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T_d= Matrix([[1,0,0,0], [0,1,0,0],[0,0,1,d],[0,0,0,1]])
    A_alpha = Matrix([[1,0,0,0],[0,cos(alpha), -sin(alpha), 0], [0,sin(alpha), cos(alpha), 0], [0,0,0,1]])
    return A_theta * T_a * T_d * A_alpha


def kinematics3joint(theta_1, theta_2,theta_3,length1, length2, length3):
    d_1 = 0
    d_2 = 0
    d_3 = 0

    a_1 = length1
    a_2 = length2
    a_3 = length3

    alpha_1 = 0
    alpha_2 = 0
    alpha_3 = 0

    A_10 = translation(theta_1,d_1,a_1,alpha_1)
    A_21 = translation(theta_2,d_2,a_2,alpha_2)
    A_32 = translation(theta_3,d_3,a_3,alpha_3)

    fk = A_10*A_21*A_32
    return fk

def kinematics4joint(theta_1, theta_2,theta_3, theta_4, length1, length2, length3, length4):
    d_1 = 0
    d_2 = 0
    d_3 = 0
    d_4 = 0

    a_1 = length1
    a_2 = length2
    a_3 = length3
    a_4 = length4

    alpha_1 = 0
    alpha_2 = 0
    alpha_3 = 0
    alpha_4 = 0

    A_10 = translation(theta_1,d_1,a_1,alpha_1)
    A_21 = translation(theta_2,d_2,a_2,alpha_2)
    A_32 = translation(theta_3,d_3,a_3,alpha_3)
    A_43 = translation(theta_4,d_4,a_4,alpha_4)

    fk = A_10*A_21*A_32*A_43
    return fk

def kinematics_button(theta_1, theta_2,theta_3, theta_4, length1, length2, length3, length4):
    d_1 = 0
    d_2 = 0
    d_3 = 0
    d_4 = 0

    a_1 = length1
    a_2 = length2
    a_3 = length3
    a_4 = length4

    alpha_1 = PI/2
    alpha_2 = 0
    alpha_3 = 0
    alpha_4 = 0

    A_10 = translation(3*PI/2+theta_1,d_1,a_1,alpha_1)
    A_21 = translation(PI/2+theta_2,d_2,a_2,alpha_2)
    A_32 = translation(theta_3,d_3,a_3,alpha_3)
    A_43 = translation(theta_4,d_4,a_4,alpha_4)

    fk = A_10*A_21*A_32*A_43
    return fk


def kinematics(theta_1, theta_2, length1, length2):

    d_1 = 0
    d_2 = 0

    a_1 = length1
    a_2 = length2

    alpha_1 = 0
    alpha_2 = 0

    A_10 = translation(theta_1,d_1,a_1,alpha_1)
    A_21 = translation(theta_2,d_2,a_2,alpha_2)

    fk = A_10*A_21
    return fk

def jacobian(current_theta_1, current_theta_2, l1, l2):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    k = kinematics(sym_theta_1,sym_theta_2,l1,l2)
    print(k[3])
    print(k[7])
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2)], [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    return np.array(j).astype(np.float64)

def jacobian3(current_theta_1, current_theta_2, current_theta_3, l1, l2, l3):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    sym_theta_3 = sympy.Symbol('theta_3')
    k = kinematics3joint(sym_theta_1,sym_theta_2,sym_theta_3,l1,l2,l3)
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2),diff(k[3], sym_theta_3)], [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2),diff(k[7], sym_theta_3)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    j = j.subs(sym_theta_3, current_theta_3)
    return np.array(j).astype(np.float64)

def jacobian_just_head(current_theta_1, current_theta_2, current_theta_3, current_theta_4, l1, l2, l3, l4):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    sym_theta_3 = sympy.Symbol('theta_3')
    sym_theta_4 = sympy.Symbol('theta_4')
    k = kinematics4joint(sym_theta_1,sym_theta_2,sym_theta_3,sym_theta_4,l1,l2,l3,l4)
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2), diff(k[3], sym_theta_3), diff(k[3], sym_theta_4)],
                [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2), diff(k[7], sym_theta_3), diff(k[7], sym_theta_4)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    j = j.subs(sym_theta_3, current_theta_3)
    j = j.subs(sym_theta_4, current_theta_4)
    return np.array([[0.0,0.0,0.0,j[3]],[0.0,0.0,0.0,j[7]]]).astype(np.float64)

def jacobian4(current_theta_1, current_theta_2, current_theta_3, current_theta_4, l1, l2, l3, l4):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    sym_theta_3 = sympy.Symbol('theta_3')
    sym_theta_4 = sympy.Symbol('theta_4')
    k = kinematics4joint(sym_theta_1,sym_theta_2,sym_theta_3,sym_theta_4,l1,l2,l3,l4)
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2),diff(k[3], sym_theta_3),diff(k[3], sym_theta_4)], [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2), diff(k[7], sym_theta_3),diff(k[7], sym_theta_4)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    j = j.subs(sym_theta_3, current_theta_3)
    j = j.subs(sym_theta_4, current_theta_4)
    return np.array(j).astype(np.float64)

def jacobian_button(current_theta_1, current_theta_2, current_theta_3, current_theta_4, l1, l2, l3, l4):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    sym_theta_3 = sympy.Symbol('theta_3')
    sym_theta_4 = sympy.Symbol('theta_4')
    k = kinematics_button(sym_theta_1,sym_theta_2,sym_theta_3,sym_theta_4,l1,l2,l3,l4)
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2),diff(k[3], sym_theta_3),diff(k[3], sym_theta_4)], [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2), diff(k[7], sym_theta_3),diff(k[7], sym_theta_4)],[diff(k[11], sym_theta_1), diff(k[11], sym_theta_2), diff(k[11], sym_theta_3),diff(k[11], sym_theta_4)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    j = j.subs(sym_theta_3, current_theta_3)
    j = j.subs(sym_theta_4, current_theta_4)
    return np.array(j).astype(np.float64)

def desired_joint_angles(theta1, theta2, l1, l2, theta1_d, theta2_d):
    global error, previous_time
    jac = jacobian(theta1, theta2, l1, l2)
    # P gain
    K_p = np.array([[0.001,0],[0,0.001]])
    # D gain
    K_d = np.array([[0.00001,0],[0,0.00001]])

    kin = kinematics(theta1,theta2,l1,l2)
    # robot end-effector position
    pos = np.array([kin[0], kin[1]])
    # desired trajectory
    pos_d = np.array([theta1_d,theta2_d])
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
    e_d_dot = np.dot(K_d, error_d.transpose())
    e_dot = np.dot(K_p, error.transpose())
    dq_d =np.dot(J_inv, ( np.dot(K_d,error_d.transpose()) + np.dot(K_p, error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + dq_d  # control input (angular position of joints)
    # q = np.array([theta1,theta2])
    # x = np.array([theta1_d,theta2_d])
    # dt = 0.00002
    # J_inv = np.linalg.pinv(jac)
    # k = kinematics(q[0], q[1], 0.6, 0.6)
    # curr_pos = np.array([k[3], k[7]])
    #
    # dq_d = dt * np.dot(J_inv, ((x-curr_pos)/dt).transpose())
    # print(dq_d)

    return q_d % 2*PI

def big_boi(d_x, d_y,len1,len2,len3,len4,theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0):
    # Position you want it to go
    desired_x_position = d_x
    desired_y_position = d_y

    # Lengths of arms sections
    l1 = len1
    l2 = len2
    l3 = len3
    l4 = len4

    # Starting angle
    theta_1 = theta1
    theta_2 = theta2
    theta_3 = theta3
    theta_4 = theta4

    k = kinematics4joint(theta_1, theta_2,theta_3,theta_4, l1, l2,l3,l4)
    actual_x = k[3]
    actual_y = k[7]
    times = []
    ac_xs = []
    ac_ys = []
    d_xs = []
    d_ys = []
    threshold = 0.005
    count = 0
    angles = np.array([theta_1,theta_2,theta_3,theta_4]).astype(np.float64)
    while (actual_x < desired_x_position - threshold or actual_x > desired_x_position + threshold) or (actual_y < desired_y_position - threshold or actual_y > desired_y_position + threshold):
        angles = desired_joint_angles4(theta_1, theta_2,theta_3,theta_4, l1, l2, l3, l4, desired_x_position, desired_y_position)
        k = kinematics4joint(angles[0], angles[1],angles[2],angles[3], l1, l2,l3,l4)
        actual_x = k[3]
        actual_y = k[7]
        theta_1 = angles[0]
        theta_2 = angles[1]
        theta_3 = angles[2]
        theta_4 = angles[3]

        count += 1
        #print("ac x = ", actual_x)
        #print("ac y = ", actual_y)
        #print("angle 1 =", angles[0])
        #print("angle 2 =", angles[1])
    print("angle 1 =", angles[0])
    print("angle 2 =", angles[1])
    print(actual_x)
    print(actual_y)
    return angles

def big_boi_3(d_x, d_y,len1,len2,len3,theta1=0.0,theta2=0.0,theta3=0.0):
    # Position you want it to go
    desired_x_position = d_x
    desired_y_position = d_y

    # Lengths of arms sections
    l1 = len1
    l2 = len2
    l3 = len3

    # Starting angle
    theta_1 = theta1
    theta_2 = theta2
    theta_3 = theta3

    k = kinematics3joint(theta_1, theta_2,theta_3, l1, l2,l3)
    actual_x = k[3]
    actual_y = k[7]
    times = []
    ac_xs = []
    ac_ys = []
    d_xs = []
    d_ys = []
    threshold = 0.005
    count = 0
    angles = np.array([theta_1,theta_2,theta_3]).astype(np.float64)
    for desired_x_position in np.arange(d_x, d_x+1.0, 0.1):
        for desired_y_position in np.arange(d_y, d_y-1.0, -0.1):
            count = 0
            while count < 15:
                if (actual_x < desired_x_position - threshold or actual_x > desired_x_position + threshold) or (actual_y < desired_y_position - threshold or actual_y > desired_y_position + threshold):
                    angles = desired_joint_angles3(theta_1, theta_2,theta_3, l1, l2, l3, desired_x_position, desired_y_position)
                    k = kinematics3joint(angles[0], angles[1],angles[2], l1, l2,l3)
                    actual_x = k[3]
                    actual_y = k[7]
                    theta_1 = angles[0]
                    theta_2 = angles[1]
                    theta_3 = angles[2]
                else:
                    print("angle 1 =", angles[0])
                    print("angle 2 =", angles[1])
                    print(actual_x)
                    print(actual_y)
                    return angles

                count += 1
                #print("ac x = ", actual_x)
                #print("ac y = ", actual_y)
                #print("angle 1 =", angles[0])
                #print("angle 2 =", angles[1])

    return None

def big_boi_button(d_x, d_y,d_z, len1,len2,len3,len4,theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0):
    # Position you want it to go
    desired_x_position = d_x
    desired_y_position = d_y
    desired_z_position = d_z

    # Lengths of arms sections
    l1 = len1
    l2 = len2
    l3 = len3
    l4 = len4

    # Starting angle
    theta_1 = theta1
    theta_2 = theta2
    theta_3 = theta3
    theta_4 = theta4

    k = kinematics_button(theta_1, theta_2, theta_3, theta_4,l1,l2,l3,l4)
    actual_x = k[3]
    actual_y = k[7]
    actual_z = k[11]
    times = []
    ac_xs = []
    ac_ys = []
    d_xs = []
    d_ys = []
    threshold = 0.005
    count = 0
    angles = np.array([theta_1,theta_2,theta_3,theta_4]).astype(np.float64)
    while (actual_x < desired_x_position - threshold or actual_x > desired_x_position + threshold) or (actual_y < desired_y_position - threshold or actual_y > desired_y_position + threshold) or (actual_z < desired_z_position - threshold or actual_z > desired_z_position + threshold):
        angles = desired_joint_angles_button(theta_1, theta_2,theta_3,theta_4, l1, l2, l3,l4, desired_x_position, desired_y_position,desired_z_position)
        k = kinematics_button(angles[0], angles[1],angles[2],angles[3], l1, l2, l3, l4)
        actual_x = k[3]
        actual_y = k[7]
        actual_z = k[11]
        theta_1 = angles[0]
        theta_2 = angles[1]
        theta_3 = angles[2]
        theta_4 = angles[3]

        count += 1
        #print("ac x = ", actual_x)
        #print("ac y = ", actual_y)
        #print("angle 1 =", angles[0])
        #print("angle 2 =", angles[1])
    print("angle 1 =", angles[0])
    print("angle 2 =", angles[1])
    print(actual_x)
    print(actual_y)
    return angles

def big_boi_head(d_x,d_y,len1,len2,len3,len4,theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0):
    # Position you want it to go
    desired_x_position = d_x
    desired_y_position = d_y

    # Lengths of arms sections
    l1 = len1
    l2 = len2
    l3 = len3
    l4 = len4

    # Starting angle
    theta_1 = theta1
    theta_2 = theta2
    theta_3 = theta3
    theta_4 = theta4

    k = kinematics4joint(theta_1, theta_2,theta_3,theta_4, l1, l2,l3,l4)
    actual_x = k[3]
    actual_y = k[7]
    times = []
    ac_xs = []
    ac_ys = []
    d_xs = []
    d_ys = []
    threshold = 0.005
    count = 0
    angles = np.array([theta_1, theta_2, theta_3, theta_4]).astype(np.float64)
    print("actual x, y = ",actual_x, " | ", actual_y)
    print("desired x, y = ",desired_x_position, " | ", desired_y_position)
    while (actual_x < desired_x_position - threshold or actual_x > desired_x_position + threshold) or (actual_y < desired_y_position - threshold or actual_y > desired_y_position + threshold):
        angles = desired_joint_angles_just_head(theta_1, theta_2,theta_3,theta_4, l1, l2, l3, l4, desired_x_position, desired_y_position)
        k = kinematics4joint(angles[0], angles[1],angles[2],angles[3], l1, l2,l3,l4)
        actual_x = k[3]
        actual_y = k[7]
        theta_1 = angles[0]
        theta_2 = angles[1]
        theta_3 = angles[2]
        theta_4 = angles[3]

        count += 1
        #print("ac x = ", actual_x)
        #print("ac y = ", actual_y)
        #print("angle 1 =", angles[0])
        #print("angle 2 =", angles[1])
    print("angle 1 =", angles[0])
    print("angle 2 =", angles[1])
    print(actual_x)
    print(actual_y)
    return angles

def all_joints(d_x, d_y,len1,len2,len3,len4,theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0):
    first_3_angles = big_boi_3(d_x+len4, d_y,len1,len2,len3,theta1,theta2,theta3)
    kine = kinematics3joint(first_3_angles[0],first_3_angles[1],first_3_angles[2], len1, len2, len3)
    angles = big_boi_head(kine[3]-len4,kine[7],len1,len2,len3,len4,first_3_angles[0],first_3_angles[1],first_3_angles[2],theta4)
    return angles
def joints_button(d_x, d_y, d_z,len1,len2,len3,len4,theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0):
    return big_boi_button(d_x,d_y,d_z,len1,len2,len3,len4,theta1,theta2,theta3,theta4)

def testy_boi():
    k = kinematics(-PI, PI,0.6,0.6)
    print(k)
    k = kinematics3joint(-PI, PI, -0.5, 0.6, 0.3, 0.3)
    print(k)
    print(jacobian(-PI, PI,0.6,0.6))
    print(k)

def brute_force3(d_x, d_y, l1, l2,l3, curr_theta_1 = None, curr_theta_2 = None, curr_theta_3 = None):
    print("i made it to brute force 3")
    print("desired x = ",d_x)
    print("desired y = ", d_y)
    print("curr_theta_3 = ", curr_theta_3)
    desired_x_position = d_x
    desired_y_position = d_y
    # Lengths of arms sections
    l1 = l1
    l2 = l2
    l3 = l3
    # Starting angle
    if curr_theta_1 is not None:
        start_theta_1 = curr_theta_1
        end_theta_1 = curr_theta_1
    else:
        start_theta_1 = 0.0
        end_theta_1 = PI/2
    if curr_theta_2 is not None:
        start_theta_2 = curr_theta_2
        end_theta_2 = curr_theta_2
    else:
        start_theta_2 = 0.0
        end_theta_2 = PI
    if curr_theta_3 is not None:
        start_theta_3 = curr_theta_3 - PI/6
        end_theta_3 = curr_theta_3 + PI/6
    else:
        start_theta_3 = 0.0
        end_theta_3 = PI + 0.4
    threshold_x = 0.03
    threshold_y = 0.03
    incrementer = 0.04
    for theta3 in np.arange(start_theta_3, end_theta_3, incrementer):
        k = kinematics3joint(curr_theta_1, curr_theta_2, theta3, l1, l2, l3)
        actual_x = k[3]
        actual_y = k[7]
        if (actual_x > desired_x_position and actual_x < desired_x_position + threshold_x):
            if (actual_y > desired_y_position - threshold_y and actual_y < desired_y_position + threshold_y):
                return [curr_theta_1,curr_theta_2,theta3]

def brute_force(d_x, d_y, l1, l2, curr_theta_1 = None, curr_theta_2 = None):
    print("brute force 1")

    desired_x_position = d_x
    desired_y_position = d_y
    # Lengths of arms sections
    l1 = l1
    l2 = l2
    # Starting angle
    if curr_theta_1 is not None:
        start_theta_1 = curr_theta_1 - PI/6
        end_theta_1 = curr_theta_1 + PI/6
    else:
        start_theta_1 = 0.0
        end_theta_1 = PI/2
    if curr_theta_2 is not None:
        start_theta_2 = curr_theta_2 - PI/8
        end_theta_2 = curr_theta_2 + PI/8
    else:
        start_theta_2 = 0.0
        end_theta_2 = PI
    threshold_x = 0.01
    threshold_y = 0.08
    incrementer = 0.02
    for theta1 in np.arange(start_theta_1, end_theta_1, incrementer):
        for theta2 in np.arange(start_theta_2, end_theta_2, incrementer):
            k = kinematics(theta1, theta2, l1, l2)
            actual_x = k[3]
            actual_y = k[7]
            if (actual_x > desired_x_position - threshold_x and actual_x < desired_x_position + threshold_x):
                if (actual_y > desired_y_position - threshold_y and actual_y < desired_y_position + threshold_y):
                    print(theta1)
                    print(theta2)
                    return [theta1,theta2]
    return 0

def desired_joint_angles_button(theta1, theta2, theta3, theta4, l1, l2, l3,l4, x_d, y_d, z_d):
    global error_button, previous_time
    error = error_button
    jac = jacobian_button(theta1, theta2, theta3, theta4, 0.0, l1, l2, l3)
    print("TYPE = ", jac.dtype)
    # P gain
    K_p = np.array([[0.5,0,0],[0,0.5,0],[0,0,0.5]])
    # D gain
    K_d = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]])
    kin = kinematics_button(theta1,theta2, theta3,theta4,l1,l2,l3,l4)
    # robot end-effector position
    pos = np.array([kin[3], kin[7], kin[11]]).astype(np.float64)
    print("TYPEPOS = ", pos.dtype)
    print(pos)
    # desired trajectory
    pos_d = np.array([x_d,y_d,z_d]).astype(np.float64)
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2,theta3, theta4]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
    print("TYPINV= ", J_inv.dtype)
    e_d_dot = np.dot(K_d, error_d.transpose())
    e_dot = np.dot(K_p, error.transpose())
    dq_d =np.dot(J_inv, ( np.dot(K_d,error_d.transpose()) + np.dot(K_p, error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + dq_d  # control input (angular position of joints)
    print("TYPEQD", q_d.dtype)
    # q = np.array([theta1,theta2])
    # x = np.array([theta1_d,theta2_d])
    # dt = 0.00002
    # J_inv = np.linalg.pinv(jac)
    # k = kinematics(q[0], q[1], 0.6, 0.6)
    # curr_pos = np.array([k[3], k[7]])
    #
    # dq_d = dt * np.dot(J_inv, ((x-curr_pos)/dt).transpose())
    # print(dq_d)
    for i in range(len(q_d)):
        if q_d[i] < 0:
            q_d[i] = abs(q_d[i]) % PI
            q_d[i] = -q_d[i]
        else:
            q_d[i] = q_d[i] % PI

    return q_d

def desired_joint_angles3(theta1, theta2, theta3, l1, l2, l3, x_d, y_d):
    global error, previous_time
    jac = jacobian3(theta1, theta2, theta3, l1, l2, l3)
    print("TYPE = ", jac.dtype)
    # P gain
    K_p = np.array([[0.5,0],[0,0.5]])
    # D gain
    K_d = np.array([[0.001,0],[0,0.001]])
    kin = kinematics3joint(theta1,theta2, theta3,l1,l2,l3)
    # robot end-effector position
    pos = np.array([kin[3], kin[7]]).astype(np.float64)
    print("TYPEPOS = ", pos.dtype)
    print(pos)
    # desired trajectory
    pos_d = np.array([x_d,y_d]).astype(np.float64)
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2,theta3]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
    print("TYPINV= ", J_inv.dtype)
    e_d_dot = np.dot(K_d, error_d.transpose())
    e_dot = np.dot(K_p, error.transpose())
    dq_d =np.dot(J_inv, ( np.dot(K_d,error_d.transpose()) + np.dot(K_p, error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + dq_d  # control input (angular position of joints)
    print("TYPEQD", q_d.dtype)
    # q = np.array([theta1,theta2])
    # x = np.array([theta1_d,theta2_d])
    # dt = 0.00002
    # J_inv = np.linalg.pinv(jac)
    # k = kinematics(q[0], q[1], 0.6, 0.6)
    # curr_pos = np.array([k[3], k[7]])
    #
    # dq_d = dt * np.dot(J_inv, ((x-curr_pos)/dt).transpose())
    # print(dq_d)
    for i in range(len(q_d)):
        if q_d[i] < 0:
            q_d[i] = abs(q_d[i]) % PI
            q_d[i] = -q_d[i]
        else:
            q_d[i] = q_d[i] % PI

    return q_d

def desired_joint_angles_just_head(theta1, theta2, theta3, theta4, l1, l2, l3,l4, x_d, y_d):
    global error, previous_time
    print("t1: ", theta1, "t2: ", theta2, "t3: ", theta3, "t4: ", theta4)
    jac = jacobian_just_head(theta1, theta2, theta3, theta4, l1, l2, l3, l4)
    print("TYPE = ", jac.dtype)
    print("jaccy = ", jac)
    # P gain
    K_p = np.array([[0.5,0],[0,0.5]])
    # D gain
    K_d = np.array([[0.001,0],[0,0.001]])
    kin = kinematics4joint(theta1,theta2, theta3,theta4,l1,l2,l3,l4)
    # robot end-effector position
    pos = np.array([kin[3], kin[7]]).astype(np.float64)
    print("TYPEPOS = ", pos.dtype)
    print(pos)
    # desired trajectory
    pos_d = np.array([x_d,y_d]).astype(np.float64)
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2,theta3,theta4]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
    print("TYPINV= ", J_inv.dtype)
    e_d_dot = np.dot(K_d, error_d.transpose())
    e_dot = np.dot(K_p, error.transpose())
    dq_d =np.dot(J_inv, ( np.dot(K_d,error_d.transpose()) + np.dot(K_p, error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + dq_d  # control input (angular position of joints)
    print("TYPEQD", q_d.dtype)
    # q = np.array([theta1,theta2])
    # x = np.array([theta1_d,theta2_d])
    # dt = 0.00002
    # J_inv = np.linalg.pinv(jac)
    # k = kinematics(q[0], q[1], 0.6, 0.6)
    # curr_pos = np.array([k[3], k[7]])
    #
    # dq_d = dt * np.dot(J_inv, ((x-curr_pos)/dt).transpose())
    # print(dq_d)
    for i in range(len(q_d)):
        if q_d[i] < 0:
            q_d[i] = abs(q_d[i]) % PI
            q_d[i] = -q_d[i]
        else:
            q_d[i] = q_d[i] % PI

    return q_d

def desired_joint_angles4(theta1, theta2, theta3, theta4, l1, l2, l3,l4, x_d, y_d):
    global error, previous_time
    jac = jacobian4(theta1, theta2, theta3, theta4, l1, l2, l3, l4)
    jac[0][0] = 0.0
    jac[0][1] = 0.0
    print("TYPE = ", jac.dtype)
    # P gain
    K_p = np.array([[0.1,0],[0,0.1]])
    # D gain
    K_d = np.array([[0.001,0],[0,0.001]])
    kin = kinematics4joint(theta1,theta2, theta3,theta4,l1,l2,l3,l4)
    # robot end-effector position
    pos = np.array([kin[3], kin[7]]).astype(np.float64)
    print("TYPEPOS = ", pos.dtype)
    print(pos)
    # desired trajectory
    pos_d = np.array([x_d,y_d]).astype(np.float64)
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2,theta3,theta4]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
    print("TYPINV= ", J_inv.dtype)
    e_d_dot = np.dot(K_d, error_d.transpose())
    e_dot = np.dot(K_p, error.transpose())
    dq_d =np.dot(J_inv, ( np.dot(K_d,error_d.transpose()) + np.dot(K_p, error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + dq_d  # control input (angular position of joints)
    print("TYPEQD", q_d.dtype)
    # q = np.array([theta1,theta2])
    # x = np.array([theta1_d,theta2_d])
    # dt = 0.00002
    # J_inv = np.linalg.pinv(jac)
    # k = kinematics(q[0], q[1], 0.6, 0.6)
    # curr_pos = np.array([k[3], k[7]])
    #
    # dq_d = dt * np.dot(J_inv, ((x-curr_pos)/dt).transpose())
    # print(dq_d)
    for i in range(len(q_d)):
        if q_d[i] < 0:
            q_d[i] = abs(q_d[i]) % PI
            q_d[i] = -q_d[i]
        else:
            q_d[i] = q_d[i] % PI

    return q_d

def brute4joint(d_x, d_y, l1,l2,l3, l4, curr_theta_1 = None, curr_theta_2 = None, curr_theta_3 = None, curr_theta_4 = None):
    print("brute force 1")
    print("desired x = ",d_x)
    print("desired y = ", d_y)
    # Lengths of arms sections
    l1 = l1
    l2 = l2
    l3 = l3
    l4 = l4
    desired_x_position_head = d_x
    desired_x_position_3 = d_x + l4
    desired_y_position = d_y
    # Starting angle
    if curr_theta_1 is not None:
        start_theta_1 = curr_theta_1 - PI/6
        end_theta_1 = curr_theta_1 + 0
    else:
        start_theta_1 = 0.0
        end_theta_1 = PI/2
    if curr_theta_2 is not None:
        start_theta_2 = curr_theta_2 - PI/8
        end_theta_2 = curr_theta_2 + PI/8
    else:
        start_theta_2 = PI/7
        end_theta_2 = PI/2
    if curr_theta_3 is not None:
        start_theta_3 = curr_theta_3 - PI / 8
        end_theta_3 = curr_theta_3 + PI/8
    else:
        start_theta_3 = 0.0
        end_theta_3 = PI/2
    if curr_theta_4 is not None:
        start_theta_4 = curr_theta_4 - PI / 8
        end_theta_4 = curr_theta_4 + PI / 8
    else:
        start_theta_4 = 0.0
        end_theta_4 = PI/2

    threshold_x_3 = 0.01
    threshold_y_3 = 0.06
    threshold_x_head = 0.01
    threshold_y_head = 0.08
    incrementer = 0.04
    for theta1 in np.arange(start_theta_1, end_theta_1, incrementer):
        for theta2 in np.arange(start_theta_2, end_theta_2, incrementer):
            for theta3 in np.arange(start_theta_3, end_theta_3, incrementer):
                k = kinematics3joint(theta1, theta2, theta3, l1, l2, l3)
                actual_x = k[3]
                actual_y = k[7]
                if (actual_x > desired_x_position_3 and actual_x < desired_x_position_3 + threshold_x_3):
                    if (actual_y > desired_y_position - threshold_y_3 and actual_y < desired_y_position + threshold_y_3):
                        print(theta1)
                        print(theta2)
                        for theta4 in np.arange(start_theta_4,end_theta_4, incrementer):
                            k = kinematics4joint(theta1, theta2, theta3, theta4, l1, l2, l3, l4)
                            actual_x = k[3]
                            actual_y = k[7]
                            if (actual_x > desired_x_position_head and actual_x < desired_x_position_head + threshold_x_head):
                                if (actual_y > desired_y_position - threshold_y_head and actual_y < desired_y_position + threshold_y_head):
                                    return [theta1, theta2, theta3, theta4]


    return 0




if __name__ == "__main__":
    #brute_force3(-0.28,0.98,0.78,0.7, 0.1,1.36,0.7,1.7)
    #testy_boi()
    big_boi()