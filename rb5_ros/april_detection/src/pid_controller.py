import math
import numpy as np

class PID():
    """
    A simple PID controller inspired by https://github.com/jellevos/simple-ros-pid.
    """
    def __init__(self, Kp=0.2, Ki=0.0, Kd=0.0, setpoint=np.zeros(3)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.last_output = None
        self.last_input = None

        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)
    
    def update(self, dt, curpoint):
        error = self.setpoint - curpoint
        error[2] = (error[2] + math.pi) % (2 * math.pi) - math.pi

        self.integral = self.integral + error * dt

        derivative = error - self.previous_error
        derivative = derivative / dt
        
        output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

        self.last_output = output
        self.last_input = curpoint
        self.previous_error = error

        return output, error

    def clear(self, Kp=0.2, Ki=0.0, Kd=0.0, setpoint=[0.0, 0.0, 0.0]):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)
 
        self.last_output = None
        self.last_input = None