import math

class PID():
    """
    A simple PID controller inspired by https://github.com/jellevos/simple-ros-pid.
    """

    def __init__(self, Kp=[0.2, 0.3, 0.4], Ki=[0.2, 0.3, 0.4], Kd=[0.2, 0.3, 0.4], setpoint=[0.0, 0.0, 0.0]):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.last_output = None
        self.last_input = None

        self.integral = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
    
    def update(self, dt, curpoint):
        error = [self.setpoint[i] - curpoint[i] for i in range(len(self.setpoint))]

        self.integral += [e * dt for e in error]

        derivative = [error[i] - self.previous_error[i] for i in range(len(error))]
        derivative = [d / dt for d in derivative]
        
        output = [error[i] * self.Kp[i] + self.integral[i] * self.Ki[i] + derivative[i] * self.Kd[i] for i in range(len(error))]

        self.last_output = output
        self.last_input = curpoint
        self.previous_error = error

        return output, error

    def clear(self, Kp=[0.2, 0.3, 0.4], Ki=[0.0, 0.0, 0.0], Kd=[0.0, 0.0, 0.0], setpoint=[0.0, 0.0, 0.0]):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.previous_error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
 
        self.last_output = None
        self.last_input = None
