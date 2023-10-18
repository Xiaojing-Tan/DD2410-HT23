#! /usr/bin/env python3

"""
    # {Xiaojing Tan}
    # {xta@kth.se}
"""

import numpy as np
pi = np.pi
sin = np.sin
cos = np.cos
asin = np.arcsin
acos = np.arccos
atan = np.arctan
atan2 = np.arctan2

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35
    x = x - l0
    
    q[2] = z
    q[1] = acos((x**2+y**2-l1**2-l2**2)/2/l1/l2)
    q[0] = atan2(y,x) - acos((l1**2+x**2+y**2-l2**2)/2/l1/np.sqrt(x*x+y*y))
    
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """
    # Target
    P = point
    R = np.array(R)
    tolerance = 1e-3

    # DH table
    L = 0.4
    M = 0.39
    alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0]
    d = [0.311, 0, L, 0, M, 0, 0.078]
    a = np.zeros(7);
    P_E = np.array([0, 0, 0, 1])

    for i in range(100):
        T = get_trans(alpha, d, a, q)
        J = get_jacobian(T, q)
        
        P_hat = (T[-1] @ P_E)[:3]
        error_P = P_hat - P
        R_hat = T[-1][:3,:3]
        error_O = -0.5*(np.cross(R_hat[:,0],R[:,0])+np.cross(R_hat[:,1],R[:,1])+np.cross(R_hat[:,2],R[:,2])) # Take negative or positive? pp137 R-MPC

        error_X = np.concatenate((error_P, error_O))
        error_q = np.linalg.pinv(J) @ error_X
        q = q - error_q

        if max(abs(error_X)) < tolerance:
            print("Reach tolerance: ",end = '')
            break

    print(max(abs(error_X)))
    return q

def get_trans(alpha, d, a, q):
    T = []
    T_m = np.eye(4)
    for i in range(len(q)):
        T_temp = np.array([[cos(q[i]),    -sin(q[i])*cos(alpha[i]),   sin(q[i])*sin(alpha[i]),    a[i]*cos(q[i])],
                           [sin(q[i]),    cos(q[i])*cos(alpha[i]),    -cos(q[i])*sin(alpha[i]),   a[i]*sin(q[i])],
                           [0,            sin(alpha[i]),              cos(alpha[i]),              d[i]],
                           [0,            0,                          0,                          1]])
        
        T_m = T_m @ T_temp
        T.append(T_m)
    return T

def get_jacobian(T, q):
    p_e = T[-1][:3,3]
    J = np.empty((0,6))
    for i in range(0, len(q)):
        if i == 0:
            z_m = np.array([0,0,1])
            p_m = np.array([0,0,0])
        else:
            z_m = T[i-1][:3,2]
            p_m = T[i-1][:3,3]
        JPi = np.cross(z_m,p_e-p_m)
        JOi = z_m
        Ji = np.concatenate((JPi,JOi))
        J = np.row_stack((J,Ji))
    J = np.transpose(J)
    return J
