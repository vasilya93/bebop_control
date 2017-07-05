class pd_controller:

    def __init__(self, p_coef, d_coef, limit_out):
        self._p_coef = p_coef
        self._d_coef = d_coef
        self._limit_out = limit_out
        self._value_desired = 0.0
	self._input_current = 0.0
	self._error_previous = 0.0

    def set_value_desired(self, value_desired):
        desired_diff = value_desired - self._value_desired
        self._value_desired = value_desired
        error_current = self._value_desired - self._input_current
        output = self._p_coef * error_current + self._d_coef * desired_diff
	if output > self._limit_out:
            output = self._limit_out
        elif output < (-self._limit_out):
            output = (-self._limit_out)
        return output

    def set_value_current(self, value_current):
        value_diff = self._input_current - value_current 
        self._input_current = value_current
        error_current = self._value_desired - self._input_current
        output = self._p_coef * error_current + self._d_coef * value_diff
	if output > self._limit_out:
            output = self._limit_out
        elif output < (-self._limit_out):
            output = (-self._limit_out)
        return output

    def get_error(self):
        return self._input_current - self._value_desired

