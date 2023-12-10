#!/usr/bin/env python
import rospy
from pid_controller import PID
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, euler_from_quaternion
from path_planning import generate_coverage1

class WayPointsNode:
    def __init__(self):
        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=1)

        self.V_X, self.V_Y, self.angular = 0.0, 0.0, 0.0     # velocity under world coordinate
        self.v_x, self.v_y = 0.0, 0.0                        # velocity under robot coordinate
        self.theta = 0.0                                    # angle between the robot coordinate and the world coordinate
        self.km = self.get_kinematic_matrix()               # kinematic matrix
        self.wheels_angular = [0.0] * 4                     # angular speed of the four wheels 
        self.min_angular = 6.5                            # minimum angular speed of each wheel (rad/s)
        self.max_angular = 8.5                            # maximum angular speed of each wheel (rad/s)

        self.curpoint = np.zeros(3)                         # current position of the robot
        self.dt = 0.05                                       # timestep for pid controller
        self.cur_error = np.zeros(3)

        self.min_error = 0.1                  
        self.pid_control = PID(Kp=0.2, Ki=0.0, Kd=0.0, setpoint=self.curpoint)

        self.listener = tf.TransformListener()
    
    def drive(self):
        print(self.curpoint)
        pid_output, self.cur_error = self.pid_control.update(self.dt, self.curpoint)
        self.theta = self.curpoint[2]

        self.V_X, self.V_Y, self.angular = pid_output[0], pid_output[1], pid_output[2]  # world coordinate

        self.v_x, self.v_y = self.world_to_robot(self.V_X, self.V_Y, self.theta)

        self.wheels_angular = [
            self.km[0][0] * self.v_x + self.km[0][1] * self.v_y + self.km[0][2] * self.angular,
            self.km[1][0] * self.v_x + self.km[1][1] * self.v_y + self.km[1][2] * self.angular,
            self.km[2][0] * self.v_x + self.km[2][1] * self.v_y + self.km[2][2] * self.angular,
            self.km[3][0] * self.v_x + self.km[3][1] * self.v_y + self.km[3][2] * self.angular,
        ]
        
        self._clamp()       # set angular speed of the wheels to threshold and recompute vx, vy

        self.V_X, self.V_Y = self.robot_to_world(self.v_x, self.v_y, self.theta)

        self.curpoint[0] += self.dt * self.V_X
        self.curpoint[1] += self.dt * self.V_Y
        self.curpoint[2] += self.dt * self.angular
        self.curpoint[2] = (self.curpoint[2] + math.pi) % (2 * math.pi) - math.pi
        
        points_msg = Float64MultiArray(data=self.wheels_angular)
        self.publisher.publish(points_msg)
    
    def send_end_signal(self):
        end_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        self.publisher.publish(end_msg)
    
    def get_kinematic_matrix(self):
        lx = 0.066      # distance between front wheels (m)
        ly = 0.056      # distance between front wheels and back wheels (m)
        r = 0.03        # radius of the wheels (m)

        kinematic_matrix = np.array([
            [1, -1, lx+ly],
            [1, 1, -(lx+ly)],
            [1, 1, lx+ly],
            [1, -1, -(lx+ly)],
        ]) / r

        return kinematic_matrix
    
    def get_error(self):
        # In this task, we care less about the angle of the car
        error = [self.cur_error[0], self.cur_error[1]]
        return np.linalg.norm(error)

    def _clamp(self):
        lx = 0.066      # distance between front wheels (m)
        ly = 0.056      # distance between front wheels and back wheels (m)
        r = 0.03        # radius of the wheels (m)
        ratio = 1.0

        for i in range(len(self.wheels_angular)):
            if abs(self.wheels_angular[i]) < self.min_angular and self.wheels_angular[i] != 0:
                ratio = max(ratio, self.min_angular/abs(self.wheels_angular[i]))

        for i in range(len(self.wheels_angular)):
            sign = self.wheels_angular[i] / abs(self.wheels_angular[i]) if self.wheels_angular[i] != 0.0 else 1.0
            self.wheels_angular[i] *= ratio

            if abs(self.wheels_angular[i]) > self.max_angular:
                self.wheels_angular[i] = sign * self.max_angular
        
        self.v_x = (self.wheels_angular[0] + self.wheels_angular[1] + self.wheels_angular[2] + self.wheels_angular[3]) * r / 4.0
        self.v_y = (-self.wheels_angular[0] + self.wheels_angular[1] + self.wheels_angular[2] - self.wheels_angular[3]) * r / 4.0
        self.angular = -(-self.wheels_angular[0] + self.wheels_angular[1] - self.wheels_angular[2] + self.wheels_angular[3]) * r / 4.0 / (lx+ly)

    def world_to_robot(self, V_X, V_Y, theta):
        v_x = math.cos(theta) * V_X + math.sin(theta) * V_Y
        v_y = -math.sin(theta) * V_X + math.cos(theta) * V_Y

        return v_x, v_y

    def robot_to_world(self, v_x, v_y, theta):
        V_X = math.cos(theta) * v_x - math.sin(theta) * v_y
        V_Y = math.sin(theta) * v_x + math.cos(theta) * v_y

        return V_X, V_Y

    def update_pos(self):
        result = None
        foundSolution = False

        for i in range(8):
            body_name = "body_" + str(i)
            if self.listener.frameExists(body_name):
                try:
                    now = rospy.Time()
                    # wait for the transform ready from the map to the camera for 0.1 second.
                    self.listener.waitForTransform("world", body_name, now, rospy.Duration(0.1))
                    # extract the transform camera pose in the map coordinate.
                    (trans, rot) = self.listener.lookupTransform("world", body_name, now)
                    # convert the rotate matrix to theta angle in 2d
                    (roll, pitch, yaw) = euler_from_quaternion(rot)
                    yaw = pi2pi(yaw)

                    result = np.array([trans[0], trans[1], yaw])
                    foundSolution = True
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                    print("meet error")
        self.listener.clear()

        if foundSolution:
            self.curpoint = result

def pi2pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

if __name__ == "__main__":
    rospy.init_node("way_points")
    waypoints_node = WayPointsNode()

    _, waypoints = generate_coverage1()
    # points_list = generate_speed_path()

    for i in range(len(waypoints)-1):
        setpoint = waypoints[i+1]
        prepoint = waypoints[i]

        if i == 0:
            waypoints_node.curpoint = prepoint

        waypoints_node.cur_error = setpoint - waypoints_node.curpoint
        waypoints_node.cur_error[2] = pi2pi(waypoints_node.cur_error[2])

        waypoints_node.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=setpoint)

        while waypoints_node.get_error() >= waypoints_node.min_error:
            waypoints_node.drive()
            time.sleep(0.05)
            # waypoints_node.update_pos()

    waypoints_node.send_end_signal()