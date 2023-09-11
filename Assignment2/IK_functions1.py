#! /usr/bin/env python3

"""
    # {Xiaojing Tan}
    # {xta@kth.se}
"""
import numpy as np
pi = np.pi
sin = np.sin
cos = np.cos
acos = np.arccos
atan = np.arctan
atan2 = np.arctan2
tolerance = 1e-3

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
    l = np.sqrt(x*x+y*y)

    q[2] = z
    q[1] = acos(-(l1**2+l2**2-x**2-y**2)/2/l1/l2)
    q[0] = atan(y/x) - acos((l1*l1+x*x+y*y-l2*l2)/2/l1/l)
    
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """
    # Convert into column vectors
    P = np.array([[x], [y], [z]])
    R = np.array(R)
    q = np.array(q).reshape(-1,1)

    # DH table
    L = 0.4
    M = 0.39
    alpha = np.array([pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0])
    d = np.array([0.311, 0, L, 0, M, 0, 0.078])
    # a = np.zeros(7)

    while True:
        # Get transform matrix
        T = np.eye(4)
        z = np.array([[0],[0],[1]])
        p = np.array([[0],[0],[0]])
        for i in range(7):
            T_temp = np.array([[cos(q[i,0]),    -sin(q[i,0])*cos(alpha[i]), sin(q[i,0])*sin(alpha[i]),  0   ],
                            [sin(q[i,0]),       cos(q[i,0])*cos(alpha[i]),  -cos(q[i,0])*sin(alpha[i]), 0   ],
                            [0,                 sin(alpha[i]),              cos(alpha[i]),              d[i]],
                            [0,                 0,                          0,                          1   ]])
            T = T @ T_temp
            z_temp = T[0:3,2].reshape(-1,1)
            z = np.column_stack((z,z_temp))
            p_temp = T[0:3,3].reshape(-1,1)
            p = np.column_stack((p,p_temp))

        # Get Jacobian matrix
        J = np.empty((6,0))
        for i in range(7):
            JP = np.cross(z[:,i],p[:,7]-p[:,i])
            JO = z[:,i]
            J = np.column_stack((J, np.concatenate((JP, JO)).reshape(-1,1)))

        # Calculate position and pose error
        P_hat = T[0:3,3].reshape(-1,1)
        error_P = P_hat - P
        R_hat = T[0:3,0:3]
        error_Euler1 = 0.5*(np.cross(R_hat[:,0],R[:,0])+np.cross(R_hat[:,1],R[:,1])+np.cross(R_hat[:,2],R[:,2])).reshape(-1,1)
        error_R = R.T @ R_hat
        error_Euler2 = np.array([atan2(error_R[2, 1], error_R[2, 2]),  # Roll
                                atan2(-error_R[2, 0], np.sqrt(error_R[2, 1] ** 2 + error_R[2, 2] ** 2)),  # Pitch
                                atan2(error_R[1, 0], error_R[0, 0])]).reshape(-1,1) # Yaw
        error_Euler3 = error_R - np.eye(3)
        error_Euler3 = (error_Euler3[:,0]+error_Euler3[:,1]+error_Euler3[:,2]).reshape(-1,1)
        error_X = np.vstack((error_P, error_Euler1))
        # print("Maximum error: ",max(abs(error_X)))

        # Calculate joint error
        error_q = np.dot(np.linalg.pinv(J),error_X)
        q = q - error_q

        if max(abs(error_X)) < tolerance:
            break
    return q
