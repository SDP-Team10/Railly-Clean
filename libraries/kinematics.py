import numpy as np
import sympy
from sympy import Matrix, diff, sin, cos
import math
import time
import matplotlib.pyplot as plt

PI = math.pi
error = np.array([0,0])
previous_time = np.array([time.time()])


def translation(theta, d , a, alpha):
    A_theta = Matrix([[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0,0,1,0], [0,0,0,1]])
    T_a = Matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T_d= Matrix([[1,0,0,0], [0,1,0,0],[0,0,1,d],[0,0,0,1]])
    A_alpha = Matrix([[1,0,0,0],[0,cos(alpha), -sin(alpha), 0], [0,sin(alpha), cos(alpha), 0], [0,0,0,1]])
    return A_theta * T_a * T_d * A_alpha

def kinematics(theta_1, theta_2, length1, length2):

    d_1 = 0
    d_2 = 0

    a_1 = length1
    a_2 = length2

    alpha_1 = 0
    alpha_2 = 0

    A_10 = translation(theta_1,d_1, a_1,alpha_1)
    A_21 = translation(theta_2,d_2,a_2,alpha_2)

    fk = A_10*A_21
    return fk

def jacobian(current_theta_1, current_theta_2, l1, l2):
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    sym_length_1 = sympy.Symbol('l1')
    sym_length_2 = sympy.Symbol('l2')
    k = kinematics(sym_theta_1,sym_theta_2,l1,l2)
    j = Matrix([[diff(k[3], sym_theta_1), diff(k[3], sym_theta_2)], [diff(k[7], sym_theta_1), diff(k[7], sym_theta_2)]])
    j = j.subs(sym_theta_1, current_theta_1)
    j = j.subs(sym_theta_2, current_theta_2)
    return np.array(j).astype(np.float64)

def desired_joint_angles(theta1, theta2, l1, l2, theta1_d, theta2_d):
    global error, previous_time
    jac = jacobian(theta1, theta2, l1, l2)
    # P gain
    K_p = np.array([[0.000005,0],[0,0.000005]])
    # D gain
    K_d = np.array([[0.00000001,0],[0,0.00000001]])

    kin = kinematics(theta1,theta2,l1,l2)
    # robot end-effector position
    pos = np.array([kin[0], kin[1]])
    # desired trajectory
    pos_d = np.array([theta1_d,theta2_d])
    # estimate derivative of error
    error_d = ((pos_d - pos) - error)
    print(error_d)
    # estimate error
    error = pos_d-pos
    q = np.array([theta1, theta2]) # estimate initial value of joints'
    J_inv = np.linalg.pinv(jac)  # calculating the psudeo inverse of Jacobian
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

def big_boi():
    # Position you want it to go
    desired_x_position = 0.6
    desired_y_position = -0.6

    # Lengths of arms sections
    l1 = 0.6
    l2 = 0.6
    # Starting angle
    theta_1 = 0.0
    theta_2 = 0.0

    k = kinematics(theta_1, theta_2, l1, l2)
    actual_x = k[0]
    actual_y = k[1]
    times = []
    ac_xs = []
    ac_ys = []
    d_xs = []
    d_ys = []
    threshold = 0.05
    count = 0
    while (actual_x < desired_x_position - threshold or actual_x > desired_x_position + threshold) or (actual_y < desired_y_position - threshold or actual_y > desired_y_position + threshold) and count < 200:
        curr_time = time.time()
        angles = desired_joint_angles(theta_1, theta_2, l1, l2, desired_x_position, desired_y_position)
        k = kinematics(angles[0], angles[1], l1, l2)
        actual_x = k[3]
        actual_y = k[7]
        theta_1 = angles[0]
        theta_2 = angles[1]
        times.append(curr_time)
        ac_xs.append(actual_x)
        ac_ys.append(actual_y)
        d_xs.append(desired_x_position)
        d_ys.append(desired_y_position)
        count += 1
        print("ac x = ", actual_x)
        print("ac y = ", actual_y)
        print("angle 1 =", angles[0])
        print("angle 2 =", angles[1])

    plt.scatter(times, ac_xs, c='r', s=0.1)
    plt.scatter(times, ac_ys, c='b', s=0.1)
    plt.scatter(times, d_xs, c='y', s=0.1)
    plt.scatter(times, d_ys, c='g', s=0.1)
    plt.show()
    print("angle 1 =", angles[0])
    print("angle 2 =", angles[1])
    print(actual_x)
    print(actual_y)

def testy_boi():
    k = kinematics(-0.5, 0.0,0.6,0.6)
    #print(jacobian(0.1,0.5,0.6,0.6))
    print(k)

def brute_force():
    desired_x_position = 0.6
    desired_y_position = -0.6
    # Lengths of arms sections
    l1 = 0.78
    l2 = 0.7
    # Starting angle
    theta_1 = -PI/2
    theta_2 = -PI
    threshold_x = 0.05
    threshold_y = 0.05
    incrementer = 0.08
    for theta1 in np.arange(theta_1, PI/2, incrementer):
        for theta2 in np.arange(theta_2, 0, incrementer):
            k = kinematics(theta1, theta2, l1, l2)
            actual_x = k[3]
            actual_y = k[7]
            print(actual_x, actual_y)
            if (actual_x > desired_x_position - threshold_x and actual_x < desired_x_position + threshold_x):
                if (actual_y > desired_y_position - threshold_y and actual_y < desired_y_position + threshold_y):
                    print(theta1)
                    print(theta2)
                    return 1
    return 0

if __name__ == "__main__":
    brute_force()
    #testy_boi()
    #big_boi()
