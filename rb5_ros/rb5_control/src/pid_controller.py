import math

class PID():
    """
    A simple PID controller inspired by https://github.com/jellevos/simple-ros-pid.
    """

    def __init__(self, Kp=0.2, Ki=0.0, Kd=0.0, setpoint=[0.0, 0.0, 0.0], min_error=0.01):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.last_output = None
        self.last_input = None
        self.cur_point = None

        self.integral = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.min_error = min_error
    
    def update(self, dt, cur_point):
        self.cur_point = cur_point

        error = [self.setpoint[0] - self.cur_point[0], 
            self.setpoint[1] - self.cur_point[1],
            self.setpoint[2] - self.cur_point[2]
        ]

        self.integral += [i * dt for i in error]
        derivative = [error[0] - self.previous_error[0], 
            error[1] - self.previous_error[1],
            error[2] - self.previous_error[2]
        ]

        derivative = [i / dt for i in derivative]
        
        output = [error[i] * self.Kp + self.integral[i] * self.Ki + derivative[i] * self.Kd for i in range(3)]

        self.last_output = output
        self.last_input = self.cur_point
        self.previous_error = error

        return output

    def clear(self, Kp=0.2, Ki=0.0, Kd=0.0, setpoint=[0.0, 0.0, 0.0]):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.last_output = None
        self.last_input = None
