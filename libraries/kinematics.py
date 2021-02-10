import numpy as np
import sympy
from sympy import Matrix, cos, sin, diff
import math

PI = sympy.pi

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

def jacobian( theta_1, theta_2,length1,length2):
    k = kinematics(theta_1,theta_2,length1,length2)
    j = Matrix([[diff(k[3], theta_1), diff(k[3], theta_2)],[diff(k[7], theta_1), diff(k[7], theta_2)]])
    return j

def desired_joint_angles(jacobian, theta1, theta2):
    x = np.array([theta1,theta2])
    print(x)
    print(jacobian)
    J_inv = np.linalg.pinv(jacobian)
    dq_d = np.dot(J_inv, x)
    return dq_d

if __name__ == "__main__":
    sym_theta_1 = sympy.Symbol('theta_1')
    sym_theta_2 = sympy.Symbol('theta_2')
    length1 = 0.6
    length2 = 0.6
    fk_sym = kinematics(sym_theta_1, sym_theta_2, length1, length2)
    print(fk_sym)
    theta_1 = 0.0
    theta_2 = 0.0
    fk = kinematics(theta_1, theta_2, length1, length2)
    print(fk)

    J = jacobian(sym_theta_1, sym_theta_2,length1,length2)
    print(J)
    J_array = [[-0.6*math.sin(theta_1)*math.cos(theta_2) - 0.6*math.sin(theta_1) - 0.6*math.sin(theta_2)*math.cos(theta_1),-0.6*math.sin(theta_1)*math.cos(theta_2) - 0.6*math.sin(theta_2)*math.cos(theta_1)],
    [-0.6*math.sin(theta_1)*math.sin(theta_2) + 0.6*math.cos(theta_1)*math.cos(theta_2) + 0.6*math.cos(theta_1),-0.6*math.sin(theta_1)*math.sin(theta_2) + 0.6*math.cos(theta_1)*math.cos(theta_2)]]
    jac = np.array(J_array)
    desired_x_position = 6.0
    desired_y_position = 6.0
    print(desired_joint_angles(jac,6.0,6.0))

