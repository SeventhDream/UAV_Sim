class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0

    def compute(self, setPoint, measuredValue, dt):
        error = setPoint - measuredValue
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (self.kp * error) + (self.ki *self.integral) + (self.kd * derivative)