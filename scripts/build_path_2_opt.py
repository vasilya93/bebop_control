from geometry_msgs.msg import Point
from copy import copy
import numpy as np

def flip_segments(array_segments, index_beginning, index_end):
    columns = copy(array_segments[index_beginning + 1:index_end, :])
    columns = np.flip(columns, 0)
    columns = np.flip(columns, 1)
    array_segments[index_beginning + 1:index_end, :] = columns[:, :]

def build_path_2_opt(points):
    num_points = len(points)
    indeces = range(num_points)

    line_segments = np.zeros((num_points - 1, 2), np.int)
    line_segments[:, 0] = np.array(indeces[0:num_points - 1])
    line_segments[:, 1] = np.array(indeces[1:num_points])

    while True:
        was_order_changed = False
        for i in range(num_points - 2):
            a_beg = points[line_segments[i, 0]]
            a_end = points[line_segments[i, 1]]
            a_len = np.linalg.norm([a_beg.x - a_end.x, \
                a_beg.y - a_end.y,
                a_beg.z - a_end.z])
            for j in range(i + 1, num_points - 1):
                b_beg = points[line_segments[j, 0]]
                b_end = points[line_segments[j, 1]]
                b_len = np.linalg.norm([b_beg.x - b_end.x, \
                    b_beg.y - b_end.y,
                    b_beg.z - b_end.z])
                len_total = a_len + b_len
     
                ab1_len = np.linalg.norm([a_beg.x - b_beg.x, \
                    a_beg.y - b_beg.y,
                    a_beg.z - b_beg.z])
                ab2_len = np.linalg.norm([a_end.x - b_end.x, \
                    a_end.y - b_end.y,
                    a_end.z - b_end.z])
                len_total_alt = ab1_len + ab2_len

                if len_total_alt < len_total:
                    was_order_changed = True
                    swap_var = line_segments[i, 1]
                    line_segments[i, 1] = line_segments[j, 0]
                    line_segments[j, 0] = swap_var
                    flip_segments(line_segments, i, j)
        if not was_order_changed:
            break

    indeces = np.zeros((num_points), np.int)
    indeces[0:num_points - 1] = line_segments[:, 0]
    indeces[num_points - 1] = line_segments[num_points - 2, 1]
    return indeces
