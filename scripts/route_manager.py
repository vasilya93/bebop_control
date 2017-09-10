from geometry_msgs.msg import Point
from bebop_control.msg import Path
from copy import copy
import numpy as np

ERROR_THRESHOLD = 0.1
TIME_POINT_REACHED = 1.0

RM_STATE_MOVE_TO_POINT = 1
RM_STATE_STICK_TO_POINT = 2

class StickyWaypoint:
    def __init__(self, point_new, time_new):
        self.point = copy(point_new)
        self.time = copy(time_new)

class RouteManager:
    def __init__(self):
        # whole shelf observation
        """
        self._waypoints = [Point(-0.2, 0.5, 0.9), \
            Point(-3.6, 0.5, 0.9), \
            Point(-3.6, 1.0, 0.9), \
            Point(-0.2, 1.0, 0.9), \
            Point(-0.2, 1.5, 0.9), \
            Point(-3.6, 1.5, 0.9), \
            Point(-3.6, 2.0, 0.9), \
            Point(-0.2, 2.0, 0.9), \
            Point(-0.2, 2.5, 0.9), \
            Point(-3.6, 2.5, 0.9), \
            Point(-3.6, 0.5, 0.9), \
            Point(-0.2, 0.5, 0.9)]
        """

        # change of x coordinate
        """
        self._waypoints = [Point(-1.5, 1.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-1.5, 1.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-1.5, 1.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-1.5, 1.0, 0.9)]
        """

        # change of y coordinate
        """
        self._waypoints = [Point(-2.5, 2.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-2.5, 2.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-2.5, 2.0, 0.9), \
            Point(-2.5, 1.0, 0.9), \
            Point(-2.5, 2.0, 0.9), \
            Point(-2.5, 1.0, 0.9)]
        """

        # change of z coordinate
        """
        self._waypoints = [Point(-2.5, 1.25, 0.75), \
            Point(-2.5, 1.25, 1.25), \
            Point(-2.5, 1.25, 0.75), \
            Point(-2.5, 1.25, 1.25), \
            Point(-2.5, 1.25, 0.75), \
            Point(-2.5, 1.25, 1.25), \
            Point(-2.5, 1.25, 0.75)]
        """

        # change of z coordinate
        """
        self._waypoints = [Point(-2.5, 1.25, 0.9), \
            Point(-2.5, 1.25, 1.4), \
            Point(-2.5, 1.25, 0.9), \
            Point(-2.5, 1.25, 1.4), \
            Point(-2.5, 1.25, 0.9), \
            Point(-2.5, 1.25, 1.4), \
            Point(-2.5, 1.25, 0.9)]
        """

        # looking over all of the shelves
        self._waypoints = [StickyWaypoint(Point(-0.5, 0.5, 0.9), 1.0), \
            StickyWaypoint(Point(-1.5, 0.5, 0.9), 1.0), \
            StickyWaypoint(Point(-2.5, 0.5, 0.9), 1.0), \
            StickyWaypoint(Point(-3.5, 0.5, 0.9), 1.0), \
            StickyWaypoint(Point(-3.5, 1.0, 0.9), 1.0), \
            StickyWaypoint(Point(-2.5, 1.0, 0.9), 1.0), \
            StickyWaypoint(Point(-1.5, 1.0, 0.9), 1.0), \
            StickyWaypoint(Point(-0.5, 1.0, 0.9), 1.0), \
            StickyWaypoint(Point(-0.5, 0.5, 0.9), 1.0)]

        #self._waypoints = [Point(-1.5, 0.5, 0.9)] # first shelf
        #self._waypoints = [Point(-1.5, 1.0, 0.9)] # second shelf
        #self._waypoints = [Point(-3.6, 1.5, 0.9)] # third shelf
        #self._waypoints = [Point(-2.5, 2.0, 0.9)] # fourth shelf
        #self._waypoints = [Point(-.5, 2.5, 0.9)] # fourth shelf
        self._current_waypoint = -1
        self._is_route_finished = False
        self._time_big_error = 0.0
        self._current_position = Point(0.0, 0.0, 0.0)
        self._current_error = Point(0.0, 0.0, 0.0)

        self._state = RM_STATE_MOVE_TO_POINT
        self._time_stick_begin = 0.0

    def set_current_position(self, current_position, time_current):
        self._current_position = copy(current_position)

        if self._current_waypoint == -1:
            if len(self._waypoints) > 0:
                self._current_waypoint = 0
                print("Info: copter went for waypoint number %d" % self._current_waypoint)
            else:
                print("Warning: there are no waypoints in RouteManager!")
                return

        self._calculate_current_error()
        if self._is_route_finished:
            return

        if self._state == RM_STATE_MOVE_TO_POINT:
            if np.abs(self._current_error.x) < ERROR_THRESHOLD and \
                    np.abs(self._current_error.y) < ERROR_THRESHOLD and \
                    np.abs(self._current_error.z) < ERROR_THRESHOLD:
                time_difference = time_current - self._time_big_error
                if time_difference > TIME_POINT_REACHED:
                    print("waypoint was reached")
                    if self._waypoints[self._current_waypoint].time == 0:
                        self._switch_next_waypoint(time_current)
                    else:
                        self._state = RM_STATE_STICK_TO_POINT
                        self._time_stick_begin = time_current
            else:
                self._time_big_error = time_current
        elif self._state == RM_STATE_STICK_TO_POINT:
            time_difference = time_current - self._time_stick_begin
            if time_difference >= self._waypoints[self._current_waypoint].time:
                print("waypoint was kept for %f seconds" % self._waypoints[self._current_waypoint].time)
                self._state = RM_STATE_MOVE_TO_POINT
                self._switch_next_waypoint(time_current)
            else:
                pass

    def get_current_waypoint(self):
        if self._current_waypoint == -1:
            return None
        else:
            return self._waypoints[self._current_waypoint].point

    def get_current_error(self):
        return self._current_error

    def is_route_finished(self):
        return self._is_route_finished

    def set_route(self, route):
        waypoints = route.waypoints
        points_num = len(waypoints)
        self._waypoints = []
        self._waypoints.append(StickyWaypoint(waypoints[0], 0.0))
        for i in range(1, points_num - 1):
            self._waypoints.append(StickyWaypoint(waypoints[i], 1.0))
        self._waypoints.append(StickyWaypoint(waypoints[points_num - 1], 0.0))

    def _switch_next_waypoint(self, time_current):
        self._time_big_error = time_current
        self._current_waypoint += 1
        print("Info: copter went for waypoint number %d" % self._current_waypoint)
        if self._current_waypoint >= len(self._waypoints):
            self._current_waypoint = -1
            self._is_route_finished = True
        self._calculate_current_error()

    def _calculate_current_error(self):
        if self._current_waypoint == -1:
            self._current_error = Point(0.0, 0.0, 0.0)
            return

        x_error = self._current_position.x - self._waypoints[self._current_waypoint].point.x
        y_error = self._current_position.y - self._waypoints[self._current_waypoint].point.y
        z_error = self._current_position.z - self._waypoints[self._current_waypoint].point.z
        self._current_error = Point(x_error, y_error, z_error)
