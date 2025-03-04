class PIDController:
    def __init__(self, kp, ki, kd, dt=1./120.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, current):
        error = setpoint - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def update_constants(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd