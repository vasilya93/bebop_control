#!/usr/bin/python

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from scipy.optimize import basinhopping
from pose_utils import get_mean_pose
from numpy import sqrt, power
from copy import deepcopy

import numpy as np

class MarkerPair:
    def __init__(self, id_minor = None, id_major = None):
        if id_minor is None:
            id_minor = 0
        if id_major is None:
            id_major = 0
        self.id_minor = id_minor
        self.id_major = id_major
        self.list_poses = []
        self._mean_pose = None

    def is_id(self, id_minor, id_major):
        if self.id_minor == id_minor and self.id_major == id_major:
            return True
        else:
            return False

    def calculate_mean_pose(self):
        if len(self.list_poses) <= 0:
            self._mean_pose = None
        else:
            self._mean_pose = get_mean_pose(self.list_poses)

    def get_mean_pose(self):
        return self._mean_pose
 
class CalibrationPairs:
    def __init__(self, calibration_set_size = None, banned_major_ids = None, markers_num = None):
        if calibration_set_size is None:
            calibration_set_size = 1000

        if banned_major_ids is None:
            banned_major_ids = []

        if markers_num is None:
            markers_num = 0
            
        self._calibration_pairs = []
        self._calibration_set_size = calibration_set_size

        self._markers = []
        self._banned_major_ids = banned_major_ids
        self._req_markers_num = markers_num

        self._pose_a_opt = None
        self._pose_b_opt = None
        self._pose_c_opt = None

        self._pose_a_pr = None
        self._pose_b_pr = None
        self._pose_c_pr = None

    def create_pairs(self, ids):
        ids_sorted = sorted(ids)
        ids_num = len(ids_sorted)
        for i in range(0, ids_num - 1):
            for j in range(i + 1, ids_num):
                id_minor = ids_sorted[i]
                id_major = ids_sorted[j]
                if id_major in self._banned_major_ids:
                    continue
                self._calibration_pairs.append(MarkerPair(id_minor, id_major))
                if not id_major in self._markers:
                    self._markers.append(id_major)

    def is_pair_present(self, id_minor, id_major):
        is_pair_present = False
        for pair in self._calibration_pairs:
            if pair.is_id(id_minor, id_major):
                is_pair_present = True
                break
        return is_pair_present

    def add_pose(self, id_minor, id_major, pose_new):
        for pair in self._calibration_pairs:
            if pair.is_id(id_minor, id_major):
                if len(pair.list_poses) < self._calibration_set_size:
                    pair.list_poses.append(pose_new)
                break

    def force_add_pose(self, id_minor, id_major, pose_new):
        if id_major in self._banned_major_ids:
            return
        current_pair = None
        is_pair_present = False
        for pair in self._calibration_pairs:
            if pair.is_id(id_minor, id_major):
                if len(pair.list_poses) >= self._calibration_set_size:
                    return
                current_pair = pair
                is_pair_present = True
                break

        if not is_pair_present:
            current_pair = MarkerPair(id_minor, id_major)
            self._calibration_pairs.append(current_pair)
            if not id_major in self._markers:
                self._markers.append(id_major)

        current_pair.list_poses.append(pose_new)

    def is_set_full(self):
        if len(self._calibration_pairs) <= 0:
            return False

        is_set_full = True
        for pair in self._calibration_pairs:
            if len(pair.list_poses) < self._calibration_set_size:
                is_set_full = False
                break

        return is_set_full

    # If for every major value in the dictionary, at least three different minor values
    # are present (i. e. position of a marker should be calculated on the basis of at
    # least three markers with smaller id), the function returns true, otherwise - false.
    def is_set_coherent(self):
        num_minor_values = {}
        for pair in self._calibration_pairs:
            if pair.id_major in num_minor_values:
                num_minor_values[pair.id_major] += 1
            else:
                num_minor_values[pair.id_major] = 1

        result = True
        for key in num_minor_values:
            if num_minor_values[key] < 3: 
                result = False
                break

        return result

    def are_all_markers_present(self):
        if self._req_markers_num > len(self._markers):
            return False
        else:
            return True

    def get_status_string(self):
        if len(self._calibration_pairs) <= 0:
            return "The set is empty"

        status_string = "%.3d-%.3d: %4d/%d" % \
                (self._calibration_pairs[0].id_minor, \
                self._calibration_pairs[0].id_major, \
                len(self._calibration_pairs[0].list_poses), \
                self._calibration_set_size)

        for i in range(1, len(self._calibration_pairs)):
            if len(self._calibration_pairs[i].list_poses) >= \
                    self._calibration_set_size:
                continue
            added_string = "\r\n%.3d-%.3d: %4d/%d" % \
                    (self._calibration_pairs[i].id_minor, \
                self._calibration_pairs[i].id_major, \
                len(self._calibration_pairs[i].list_poses), \
                self._calibration_set_size)
            status_string += added_string

        return status_string

    def get_mean_poses(self):
        if len(self._calibration_pairs) <= 0:
            return None

        self._calibration_pairs.sort(key = lambda x: x.id_major)
        if self._calibration_pairs[1].id_minor > self._calibration_pairs[2].id_minor:
            self._calibration_pairs[1], self._calibration_pairs[2] = \
                    self._calibration_pairs[2], self._calibration_pairs[1]

        mean_poses = []
        for pair in self._calibration_pairs:
            pair.calculate_mean_pose()
            mean_pose = pair.get_mean_pose()
            mean_poses.append(mean_pose)

        return mean_poses

    def get_corrected_poses(self, poses_preset, calibration_ids):
        num_calib_pairs = len(self._calibration_pairs)
        num_preset_poses = len(poses_preset)
        if num_calib_pairs != 3 or num_preset_poses != 3:
            print("Error: get_corrected_poses is called for %d calibration \
            pairs and for %d preset poses" % (num_calib_pairs, num_preset_poses))
            return

        poses = self.get_mean_poses()
        if poses is None or None in poses:
            print("Error: get_corrected_poses could not get mean pose")
            return

        pose_b_a, pose_c_a = self._correct_poses_to_minor(poses[0], \
                poses[1], \
                poses[2])

        self._shift_to_preset_poses(pose_b_a, pose_c_a, poses_preset)

        return {str(calibration_ids[0]): self._pose_a_opt, \
                str(calibration_ids[1]): self._pose_b_opt, \
                str(calibration_ids[2]): self._pose_c_opt}

    def get_absolute_poses(self, dict_marker_poses):
        self._calibration_pairs.sort(key = lambda x: x.id_major)
        current_major_poses = []
        #dict_absolute_poses = {}
        if len(self._calibration_pairs) != 0:
            prev_id_major = self._calibration_pairs[0].id_major

        for pair in self._calibration_pairs:
            if str(pair.id_minor) in dict_marker_poses:
                if prev_id_major == pair.id_major: 
                    pose_reference = dict_marker_poses[str(pair.id_minor)]
                    pair.calculate_mean_pose()
                    pose_major_abs = deepcopy(pair.get_mean_pose())
                    pose_major_abs.position.x += pose_reference.position.x
                    pose_major_abs.position.y += pose_reference.position.y
                    pose_major_abs.position.z += pose_reference.position.z
                    current_major_poses.append(pose_major_abs)
                else: 
                    print "id major finished"
                    pose_abs_mean = get_mean_pose(current_major_poses)
                    dict_marker_poses[str(prev_id_major)] = pose_abs_mean
                    current_major_poses = []
                    prev_id_major = pair.id_major
            else:
                print("Warning: get_absolute_poses could not find marker %d in the dictionary" % pair.id_minor)

        pose_abs_mean = get_mean_pose(current_major_poses)
        #absolute_poses[prev_id_major] = pose_abs_mean
        dict_marker_poses[str(prev_id_major)] = pose_abs_mean
        #return dict_absolute_poses

    def _correct_poses_to_minor(self, pose_b_a, pose_c_a, pose_c_b):
        # we begin with coordinate z, as its direction coincides for all of the markers
        point_a_b = Point(- pose_b_a.position.x, \
                - pose_b_a.position.y, \
                - pose_b_a.position.z)
        point_c_a = pose_c_a.position
        point_c_b = pose_c_b.position

        sum_x = point_a_b.x + point_c_a.x
        sum_y = point_a_b.y + point_c_a.y
        sum_z = point_a_b.z + point_c_a.z
        delta_x = sum_x - point_c_b.x
        delta_y = sum_y - point_c_b.y
        delta_z = sum_z - point_c_b.z

        print("delta x: %d; delta y: %f; delta z: %f" % (delta_x, delta_y, delta_z))

        point_c_a.x -= delta_x * point_c_a.x / sum_x
        point_a_b.x -= delta_x * point_a_b.x / sum_x
        point_c_a.y -= delta_y * point_c_a.y / sum_y
        point_a_b.y -= delta_y * point_a_b.y / sum_y
        point_c_a.z -= delta_z * point_c_a.z / sum_z
        point_a_b.z -= delta_z * point_a_b.z / sum_z

        pose_b_a.position.x = -point_a_b.x
        pose_b_a.position.y = -point_a_b.y
        pose_b_a.position.z = -point_a_b.z
        pose_c_a.position.x = point_c_a.x
        pose_c_a.position.y = point_c_a.y
        pose_c_a.position.z = point_c_a.z
        return (pose_b_a, pose_c_a)

    def _shift_to_preset_poses(self, pose_b_a, pose_c_a, poses_preset):
        self._pose_a_pr = poses_preset[0]; 
        self._pose_b_pr = poses_preset[1];
        self._pose_c_pr = poses_preset[2];

        self._pose_a_opt = deepcopy(self._pose_a_pr)

        self._pose_b_opt = deepcopy(pose_b_a)
        self._point_add_point(self._pose_b_opt.position, self._pose_a_opt.position)

        self._pose_c_opt = deepcopy(pose_c_a)
        self._point_add_point(self._pose_c_opt.position, self._pose_a_opt.position)

        x_range = np.arange(-0.01, 0.01, 0.001); x_size = len(x_range)
        y_range = np.arange(-0.01, 0.01, 0.001); y_size = len(y_range)
        z_range = np.arange(-0.05, 0.05, 0.001); z_size = len(z_range)

        distance_min = float("Inf")
        x_min = 0.0; y_min = 0.0; z_min = 0.0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    distance = self._calculate_distance_total([x, y, z])
                    if distance < distance_min:
                        distance_min = distance
                        x_min = x; y_min = y; z_min = z

        self._point_add_vector(self._pose_a_opt.position, [x_min, y_min, z_min])
        self._point_add_vector(self._pose_b_opt.position, [x_min, y_min, z_min])
        self._point_add_vector(self._pose_c_opt.position, [x_min, y_min, z_min])

    def _calculate_distance_total(self, shifts):
        pose_a = deepcopy(self._pose_a_opt)
        pose_b = deepcopy(self._pose_b_opt)
        pose_c = deepcopy(self._pose_c_opt)

        self._point_add_vector(pose_a.position, shifts)
        self._point_add_vector(pose_b.position, shifts)
        self._point_add_vector(pose_c.position, shifts)

        distance_a = sqrt(power((pose_a.position.x - self._pose_a_pr.position.x), 2) + \
            power((pose_a.position.y - self._pose_a_pr.position.y), 2) + \
            power((pose_a.position.z - self._pose_a_pr.position.z), 2))

        distance_b = sqrt(power((pose_b.position.x - self._pose_b_pr.position.x), 2) + \
            power((pose_b.position.y - self._pose_b_pr.position.y), 2) + \
            power((pose_b.position.z - self._pose_b_pr.position.z), 2))

        distance_c = sqrt(power((pose_c.position.x - self._pose_c_pr.position.x), 2) + \
            power((pose_c.position.y - self._pose_c_pr.position.y), 2) + \
            power((pose_c.position.z - self._pose_c_pr.position.z), 2))

        distance_total = distance_a + distance_b + distance_c

        return distance_total

    def _point_add_vector(self, point, added_vector):
        added_x = added_vector[0]
        added_y = added_vector[1]
        added_z = added_vector[2]
        point.x += added_x
        point.y += added_y
        point.z += added_z

    def _point_add_point(self, point, added_point):
        point.x += added_point.x
        point.y += added_point.y
        point.z += added_point.z
