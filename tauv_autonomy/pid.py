class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ff=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, current, dt, current_dot=None):
        error = setpoint - current
        self.integral += error * dt
        
        if current_dot is not None:
            derivative = -current_dot 
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative) + self.ff