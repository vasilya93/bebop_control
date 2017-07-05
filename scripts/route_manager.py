import numpy as np

ERROR_THRESHOLD = 0.1

class RouteManager:
    def __init__(self):
        self._waypoints = [Point(-0.5, 0.5, 0.9), \
            Point(-2.0, 0.5, 0.9), \
            Point(-2.0, 2.0, 0.9), \
            Point(-0.5, 2.0, 0.9), \
            Point(-0.5, 0.5, 0.9)
        self._current_waypoint = -1
        self._time_big_error = 0.0

    def get_current_waypoint(self):
        if self._current_waypoint == -1:
            return None
        else:
            return self._waypoints[self._current_waypoint]

    def is_waypoint_reached(self, position, time_current):
        if self._current_waypoint == -1:
            if len(self._waypoints) > 0:
                self._current_waypoint = 1
            else:
                print("Warning: there are no waypoints in RouteManager!")
                return True

        is_waypoint_reached = False
        x_error, y_error, z_error = self._calculate_errors(position)

        if np.abs(x_error) < ERROR_THRESHOLD and \
                np.abs(y_error) < ERROR_THRESHOLD and \
                np.abs(z_error) < ERROR_THRESHOLD:
            time_difference = time_current - self._time_big_error
            if time_difference > TIME_POINT_REACHED:
                is_waypoint_reached = True
                print("waypoint was reached")

                self._current_waypoint += 1
                if self._current_waypoint >= len(self._waypoints):
                    self._current_waypoint = -1
        else:
            self._time_big_error = rospy.get_time()
            is_waypoint_reached = False

        return is_waypoint_reached

    def is_route_finished(self):
        return True if self._current_waypoint == -1 else False

    def _calculate_errors(self, position):
        x_error = position.x - self._waypoints[self._current_waypoint].x
        y_error = position.y - self._waypoints[self._current_waypoint].y
        z_error = position.z - self._waypoints[self._current_waypoint].z
        return (x_error, y_error, z_error)

