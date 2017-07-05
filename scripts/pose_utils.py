#!/usr/bin/python

import numpy as np
from numpy import linalg as la

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def get_mean_point(array_points):
    x_array = []
    y_array = []
    z_array = []
    for point in array_points:
        x_array.append(point.x)
        y_array.append(point.y)
        z_array.append(point.z)

    point_mean = Point()
    point_mean.x = np.mean(x_array)
    point_mean.y = np.mean(y_array)
    point_mean.z = np.mean(z_array)

    point_std = Point()
    point_std.x = np.mean(x_array)
    point_std.y = np.mean(y_array)
    point_std.z = np.mean(z_array)
 
    return (point_mean, point_std)

def get_mean_pose(array_poses):
    if (array_poses is None) or (len(array_poses) <= 0):
        return None

    num_poses = len(array_poses)
    x_array = []
    y_array = []
    z_array = []
    q_mat = np.zeros([4, 4])
    for pose in array_poses:
        x_array.append(pose.position.x)
        y_array.append(pose.position.y)
        z_array.append(pose.position.z)

        q_new = np.array([[pose.orientation.w], \
            [pose.orientation.x], \
            [pose.orientation.y], \
            [pose.orientation.z]])

        q_mat += np.dot(q_new, q_new.transpose())

    q_mat /= num_poses

    eigenval, eigenvec = la.eig(q_mat)
    eigenvec_max = eigenvec[:, eigenval.argmax()]

    orientation_mean = Quaternion()
    orientation_mean.w = eigenvec_max[0]
    orientation_mean.x = eigenvec_max[1]
    orientation_mean.y = eigenvec_max[2]
    orientation_mean.z = eigenvec_max[3]

    point_mean = Point()
    point_mean.x = np.mean(x_array)
    point_mean.y = np.mean(y_array)
    point_mean.z = np.mean(z_array)

    point_std = Point()
    point_std.x = np.std(x_array)
    point_std.y = np.std(y_array)
    point_std.z = np.std(z_array)

    pose_mean = Pose(point_mean, orientation_mean)
 
    return pose_mean
