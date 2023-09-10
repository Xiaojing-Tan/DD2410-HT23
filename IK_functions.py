#! /usr/bin/env python3

"""
    # {Xiaojing Tan}
    # {xta@kth.se}
"""
import numpy as np
from scipy.spatial.transform import Rotation
pi = np.pi
cos = np.cos
sin = np.sin
acos = np.arccos
atan = np.arctan
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
    # X = np.array([x, y, z])
    # R = np.array(R)

    # # DH table
    # L = 0.4
    # M = 0.39
    # alpha = np.array([pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0])
    # d = np.array([0, 0, L, 0, M, 0, 0])
    # # a = np.zeros(7)

    # while True:
    #     # Get transform matrix
    #     T = np.eye(4)
    #     z = [np.array([0,0,1])]
    #     p = [np.array([0,0,0])]
    #     for i in range(7):
    #         T_temp = np.array([[cos(q[i]),  -sin(q[i])*cos(alpha[i]),   sin(q[i])*sin(alpha[i]),    0   ],
    #                         [sin(q[i]),  cos(q[i])*cos(alpha[i]),    -cos(q[i])*sin(alpha[i]),   0   ],
    #                         [0,          sin(alpha[i]),              cos(alpha[i]),              d[i]],
    #                         [0,          0,                          0,                          1   ]])
    #         T = np.dot(T,T_temp)
    #         z_temp = T[0:3,2]
    #         z.append(z_temp)
    #         p_temp = T[0:3,3]
    #         p.append(p_temp)

    #     # Get Jacobian matrix
    #     J = np.array([])
    #     for i in range(7):
    #         JP = np.cross(z[i],p[7]-p[i])
    #         JO = z[i]
    #         J = np.concatenate((J, JP))
    #         J = np.concatenate((J, JO))
    #     J = J.reshape(7,6).T
        
    #     # Calculate position error
    #     X_hat = T[0:3, 3]
    #     print("X_hat: ", X_hat)
    #     print("X: ", X)
    #     error_X = X_hat - X

    #     # Calculate pose error
    #     R_hat = T[0:3,0:3]
    #     error_R = np.dot(R.T, R_hat)
    #     error_Euler = np.array([np.arctan2(error_R[2, 1], error_R[2, 2]),  # Roll
    #                           np.arctan2(-error_R[2, 0], np.sqrt(error_R[2, 1] ** 2 + error_R[2, 2] ** 2)),  # Pitch
    #                           np.arctan2(error_R[1, 0], error_R[0, 0])]) # Yaw
        
    #     # quat_R = Rotation.from_matrix(R).as_quat()
    #     # quat_R_hat = Rotation.from_matrix(R_hat).as_quat()
    #     # diff_quat = np.quaternion(quat_R_hat) * np.quaternion(quat_R).conj()
    #     # diff_rotation = R.from_quat([diff_quat.x, diff_quat.y, diff_quat.z, diff_quat.w])
    #     # error_R = diff_rotation.as_euler('zyx')

    #     print("error_X: ", error_X)
    #     print("error_Euler: ", error_Euler)
    #     error_X = np.concatenate((error_X,error_Euler))
    #     print("Maximum error: ",max(abs(error_X)))
    #     if max(abs(error_X)) < tolerance:
    #         break

    #     # Calculate q error
    #     error_q = np.dot(np.linalg.pinv(J),error_X)
    #     q = q - error_q

    # return q
