from geometry_msgs.msg import Point
from copy import copy
import numpy as np

ERROR_THRESHOLD = 0.1
TIME_POINT_REACHED = 1.0

# what is the usage scenario of the class?

class RouteManager:
    def __init__(self):
        self._waypoints = [Point(-0.5, 0.5, 0.9), \
            Point(-2.0, 0.5, 0.9), \
            Point(-2.0, 2.0, 0.9), \
            Point(-0.5, 2.0, 0.9), \
            Point(-0.5, 0.5, 0.9)]
        self._current_waypoint = -1
        self._is_route_finished = False
        self._time_big_error = 0.0
        self._current_position = Point(0.0, 0.0, 0.0)
        self._current_error = Point(0.0, 0.0, 0.0)

    def set_current_position(self, current_position, time_current):
        self._current_position = copy(current_position)

        if self._current_waypoint == -1:
            if len(self._waypoints) > 0:
                self._current_waypoint = 1
                print("Info: copter went for waypoint number %d" % self._current_waypoint)
            else:
                print("Warning: there are no waypoints in RouteManager!")
                return

        self._calculate_current_error()

        if np.abs(self._current_error.x) < ERROR_THRESHOLD and \
                np.abs(self._current_error.y) < ERROR_THRESHOLD and \
                np.abs(self._current_error.z) < ERROR_THRESHOLD:
            time_difference = time_current - self._time_big_error
            if time_difference > TIME_POINT_REACHED:
                print("waypoint was reached")
                self._time_big_error = time_current
                self._current_waypoint += 1
                print("Info: copter went for waypoint number %d" % self._current_waypoint)
                if self._current_waypoint >= len(self._waypoints):
                    self._current_waypoint = -1
                    self._is_route_finished = True
                self._calculate_current_error()
        else:
            self._time_big_error = time_current

    def get_current_waypoint(self):
        if self._current_waypoint == -1:
            return None
        else:
            return self._waypoints[self._current_waypoint]

    def get_current_error(self):
        return self._current_error

    def is_route_finished(self):
        return self._is_route_finished

    def _calculate_current_error(self):
        if self._current_waypoint == -1:
            self._current_error = Point(0.0, 0.0, 0.0)
            return

        x_error = self._current_position.x - self._waypoints[self._current_waypoint].x
        y_error = self._current_position.y - self._waypoints[self._current_waypoint].y
        z_error = self._current_position.z - self._waypoints[self._current_waypoint].z
        self._current_error = Point(x_error, y_error, z_error)
