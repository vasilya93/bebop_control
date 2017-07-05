class pd_controller:

    def __init__(self, p_coef, d_coef, limit_out):
        self._p_coef = p_coef
        self._d_coef = d_coef
        self._limit_out = limit_out

        self._previous_error = 0.0
        self._is_error_initialized = False

    def set_current_error(self, error):
        output = error * self._p_coef

        if self._is_error_initialized:
            error_diff = error - self._previous_error
            output += self._d_coef * error_diff
            self._previous_error = error
        else:
            self._previous_error = error
            self._is_error_initialized = True

	if output > self._limit_out:
            output = self._limit_out
        elif output < (-self._limit_out):
            output = (-self._limit_out)
        return output
