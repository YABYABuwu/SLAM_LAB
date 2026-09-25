class PIDController:
    """Return a bounded correction from a target and current value."""

    def __init__(self, kp, ki, kd, max_output):
        if max_output <= 0:
            raise ValueError("max_output must be positive")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.reset()

    def reset(self):
        """Clear the history before a new destination."""
        self.integral = 0.0
        self.previous_error = None

    def compute(self, error, dt):
        """Calculate PID output; dt is the elapsed time in seconds."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt

        self.integral += error * dt
        if self.ki:
            limit = self.max_output / abs(self.ki)
            self.integral = max(-limit, min(limit, self.integral))

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return max(-self.max_output, min(self.max_output, output))
