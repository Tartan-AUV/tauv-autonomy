from collections import deque

class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ff=0.0, integral_window_sec=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        
        # Anti-windup: Sliding window settings
        self.integral_window_sec = integral_window_sec
        self.history = deque()  # Will store tuples of (area, dt)
        self.current_window_time = 0.0  # Tracks total time currently in the queue
        
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, current, dt, current_dot=None):
        error = setpoint - current
        
        # 1. Calculate the current integral area and add it to the running total
        area = error * dt
        self.integral += area
        
        # 2. Record this step in our history
        self.history.append((area, dt))
        self.current_window_time += dt
        
        # 3. Slide the window: remove oldest entries if we exceed the time limit
        while self.current_window_time > self.integral_window_sec and self.history:
            old_area, old_dt = self.history.popleft()
            self.integral -= old_area
            self.current_window_time -= old_dt

        # 4. Derivative calculation (with a safety check for zero dt)
        if current_dot is not None:
            derivative = -current_dot 
        else:
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative) + self.ff