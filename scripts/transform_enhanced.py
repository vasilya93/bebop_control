from transformations import quaternion_matrix
from transformations import inverse_matrix
from transformations import concatenate_matrices
from transformations import quaternion_from_matrix
from transformations import euler_from_matrix

import numpy as np

def quaternion_to_array(q):
    q_array = np.array([q.w, q.x, q.y, q.z])
    return q_array

def get_yaw_from_quaternion(quaternion):
    q_array = quaternion_to_array(quaternion)
    rot_mat = quaternion_matrix(q_array)
    euler = euler_from_matrix(rot_mat, 'rxyz')
    yaw_deg = euler[1] / np.pi * 180.0
    return yaw_deg
