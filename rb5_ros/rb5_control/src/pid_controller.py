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
        if self.last_input is not None:
            self.cur_point = self.last_input + [i * dt for i in self.last_output]
        else:
            self.cur_point = cur_point

        error = self.setpoint - self.cur_point

        dis_error = math.sqrt(math.pow(error[0], 2) + math.pow(error[1], 2))
        if (dis_error <= self.min_error):
            return [0.0, 0.0, 0.0]
        
        self.integral += [i * dt for i in error]
        derivative = (error - self.previous_error)/dt

        output = [i * self.Kp for i in error] + [i * self.Ki for i in self.integral] + [i * self.Kd for i in derivative]

        self.last_output = output
        self.last_input = self.cur_point
        self.previous_error = error

        return output

    def clear(self, Kp=0.2, Ki=0.0, Kd=0.0, setpoint=[0.0, 0.0, 0.0]):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.last_output = None
        self.last_input = None
