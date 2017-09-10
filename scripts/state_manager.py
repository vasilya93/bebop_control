COPTER_STATE_LANDED = 1
COPTER_STATE_TAKING_OFF = 2 # can be set explicitly
COPTER_STATE_HOVERING = 3 # can be set explicitly in some case
COPTER_STATE_NAVIGATING = 4 # can be set explicitly
COPTER_STATE_LANDING = 5 # can be set explicitly
COPTER_STATE_WAITING_PATH = 6
COPTER_STATE_APPROVING_PATH = 7

COPTER_TRANSITION_TIME = 5.0
COPTER_PATH_APPROVAL_TIME = 2.0

class StateManager:
    def __init__(self):
        self._current_state = COPTER_STATE_WAITING_PATH
        self._state_change_time = 0.0

    def set_state(self, new_state, time_current):
        self._current_state = new_state
        self._state_change_time = time_current

    def get_state(self, time_current):
        if self._current_state == COPTER_STATE_LANDING or \
                self._current_state == COPTER_STATE_TAKING_OFF:
            if time_current - self._state_change_time >= COPTER_TRANSITION_TIME:
                self._change_state_automatically()

        if self._current_state == COPTER_STATE_APPROVING_PATH:
            if time_current - self._state_change_time >= COPTER_PATH_APPROVAL_TIME:
                self._current_state = COPTER_STATE_TAKING_OFF
                self._state_change_time = time_current

        return self._current_state

    def _change_state_automatically(self):
        if self._current_state == COPTER_STATE_LANDING:
            self._current_state = COPTER_STATE_LANDED
        elif self._current_state == COPTER_STATE_TAKING_OFF:
            self._current_state = COPTER_STATE_NAVIGATING
