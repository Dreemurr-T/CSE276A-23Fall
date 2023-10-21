#!/usr/bin/env python
import rospy
from pid_controller import PID  
# from megapi import MegaPi
# from mpi_control import MegaPiController
# from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import math
import time


class WayPointsNode:
    def __init__(self, waypoint_path = '/root/rb5_ws/src/rb5_ros/rb5_control/src/waypoints.txt'):

        self.points_list = self.waypoint_reader(waypoint_path)

        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=10)

        self.v_x, self.v_y, self.angular = 0.0, 0.0, 0.0    # velocity under robot coordinate
        self.V_X, self.V_Y = 0.0, 0.0          # velocity under world coordinate
        self.theta = 0.0                            # angle between the robot coordinate and the world coordinate
        self.km = self.get_kinematic_matrix()       # kinematic matrix
        self.wheels_angular = [0.0] * 4             # angular speed of the four wheels 
        self.min_angular = 8.69                        # minimum angular speed of each wheel (rad/s)
        self.max_angular = 11.3                    # maximum angular speed of each wheel (rad/s)
        
        self.curpoint = [0.0, 0.0, 0.0]             # current position of the robot
        self.setpoint = [0.0, 0.0, 0.0]             # desired position of the robot
        self.interpoint = [0.0, 0.0, 0.0]           # tmp point for robot to rotate to the deisred direction
        self.dt = 0.1                               # timestep for pid controller
        self.cur_error = [1.0, 1.0, 1.0]
        self.min_dis_error = 0.01
        self.min_angular_error = 0.06                     
        self.pid_control = PID(Kp=[0.2, 0.3, 0.4], Ki=[0.0,0.0,0.0], Kd=[0.0,0.0,0.0], setpoint=self.curpoint)
        self.ratio = 1.0                            # balance anti-clockwise and clockwise rotation

    
    def drive(self, mode="forward"):
        pid_output, self.cur_error = self.pid_control.update(self.dt, self.curpoint)

        if mode == "rotate" and pid_output[2] < 0:
            self.ratio = 1.1
        else:
            self.ratio = 1.0

        self.V_X, self.V_Y, self.angular = pid_output[0], pid_output[1], pid_output[2]  # world coordinate

        if mode == "forward":
            self.v_x = math.sqrt(self.V_X * self.V_X + self.V_Y * self.V_Y)
            self.theta = self.interpoint[2]
            self.v_y = 0.0
            self.angular = 0.0
        elif mode == "rotate":
            self.v_x, self.v_y = 0.0, 0.0

        self.wheels_angular = [
            self.km[0][0] * self.v_x + self.km[0][1] * self.v_y + self.km[0][2] * self.angular,
            self.km[1][0] * self.v_x + self.km[1][1] * self.v_y + self.km[1][2] * self.angular,
            self.km[2][0] * self.v_x + self.km[2][1] * self.v_y + self.km[2][2] * self.angular,
            self.km[3][0] * self.v_x + self.km[3][1] * self.v_y + self.km[3][2] * self.angular,
        ]
        
        self._clamp()       # set angular speed of the wheels to threshold and recompute vx, vy

        if mode == "forward":
            self.angular = 0.0
            self.v_y = 0.0
        elif mode == "rotate":
            self.v_x, self.v_y = 0.0, 0.0

        self.V_X, self.V_Y = self.robot_to_world(self.v_x, self.v_y, self.theta)

        self.curpoint[0] += self.dt * self.V_X
        self.curpoint[1] += self.dt * self.V_Y
        self.curpoint[2] += self.dt * self.angular
        
        points_msg = Float64MultiArray(data=self.wheels_angular)
        self.publisher.publish(points_msg)
    
    def send_end_signal(self):
        end_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        self.publisher.publish(end_msg)
    
    def get_kinematic_matrix(self):
        lx = 0.112      # distance between front wheels (m)
        ly = 0.112      # distance between front wheels and back wheels (m)
        r = 0.03        # radius of the wheels (m)
        
        l1 = [1, -1, (lx+ly)]
        l2 = [1, 1, -(lx+ly)]
        l3 = [1, 1, (lx+ly)]
        l4 = [1, -1, -(lx+ly)]

        l1 = [d * (1.0 / r) for d in l1]
        l2 = [d * (1.0 / r) for d in l2]
        l3 = [d * (1.0 / r) for d in l3]
        l4 = [d * (1.0 / r) for d in l4]

        kinematic_matrix = [l1, l2, l3, l4]

        return kinematic_matrix

    def waypoint_reader(self, file_path):
        waypoints = np.loadtxt(file_path, delimiter=',')

        return waypoints.tolist()
    
    def get_dis_error(self):
        dis_error = math.sqrt(math.pow(self.cur_error[0], 2) + math.pow(self.cur_error[1], 2))
        
        return dis_error
    
    def get_ang_error(self):

        return self.cur_error[2]

    def _clamp(self):
        # ratio = 1.0

        # for i in range(len(self.wheels_angular)):
        #     if abs(self.wheels_angular[i]) < self.min_angular and abs(self.wheels_angular[i]) != 0.0:
        #         ratio = max(ratio, self.min_angular / abs(self.wheels_angular[i]))

        # self.wheels_angular = [d * ratio for d in self.wheels_angular]

        # for i in range(len(self.wheels_angular)):
        #     sign = self.wheels_angular[i] / abs(self.wheels_angular[i]) if self.wheels_angular[i] != 0.0 else 1.0
        #     if abs(self.wheels_angular[i]) > self.max_angular:
        #         self.wheels_angular[i] = self.max_angular * sign
        lx = 0.112      # distance between front wheels (m)
        ly = 0.112      # distance between front wheels and back wheels (m)
        r = 0.03        # radius of the wheels (m)

        for i in range(len(self.wheels_angular)):
            sign = self.wheels_angular[i] / abs(self.wheels_angular[i]) if self.wheels_angular[i] != 0.0 else 1.0
            if abs(self.wheels_angular[i]) < self.min_angular:
                self.wheels_angular[i] = sign * self.min_angular
            elif abs(self.wheels_angular[i]) > self.max_angular:
                self.wheels_angular[i] = sign * self.max_angular
        
        self.v_x = (self.wheels_angular[0] + self.wheels_angular[1] + self.wheels_angular[2] + self.wheels_angular[3]) * r / 4.0
        self.v_y = (-self.wheels_angular[0] + self.wheels_angular[1] + self.wheels_angular[2] - self.wheels_angular[3]) * r / 4.0
        self.angular = -(-self.wheels_angular[0] + self.wheels_angular[1] - self.wheels_angular[2] + self.wheels_angular[3]) * r / 4.0 / (lx+ly)

        if self.ratio != 1.0:
            self.wheels_angular = [d * self.ratio for d in self.wheels_angular]
    
    def world_to_robot(self, V_X, V_Y, theta):
        v_x = math.cos(theta) * V_X + math.sin(theta) * V_Y
        v_y = -math.sin(theta) * V_X + math.cos(theta) * V_Y

        return v_x, v_y

    def robot_to_world(self, v_x, v_y, theta):
        V_X = math.cos(theta) * v_x - math.sin(theta) * v_y
        V_Y = math.sin(theta) * v_x + math.cos(theta) * v_y

        return V_X, V_Y
    
    def get_inter_point(self):
        for i in range(2):
            self.interpoint[i] = self.curpoint[i]

        space = 0
        dy = self.setpoint[1] - self.curpoint[1]
        dx = self.setpoint[0] - self.curpoint[0]

        d = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if d != 0:
            theta = math.acos(dx/d)
        else:
            theta = self.curpoint[2]

        if dy >= 0 and dx >= 0:
            space = 1
        elif dy >= 0 and dx <= 0:
            space = 2
        elif dy <= 0 and dx <= 0:
            space = 3
        elif dy <= 0 and dx >= 0:
            space = 4
        
        if space == 3 or space == 4:
            theta *= -1
        
        self.interpoint[2] = theta

if __name__ == "__main__":
    rospy.init_node("way_points")
    waypoints_node = WayPointsNode()
    points_list = waypoints_node.points_list

    for i in range(len(points_list)-1):
        waypoints_node.setpoint = points_list[i+1]
        waypoints_node.curpoint = points_list[i]
        
        waypoints_node.get_inter_point()

        waypoints_node.cur_error = [waypoints_node.interpoint[i] - waypoints_node.curpoint[i] for i in range(3)]

        # print(waypoints_node.setpoint, waypoints_node.curpoint, waypoints_node.interpoint)

        # step 1: rotate to the desired direction
        waypoints_node.pid_control.clear(Kp=[0.2, 0.2, 0.05], Ki=[0.0,0.0,0.0], Kd=[0.0,0.0,0.0], setpoint=waypoints_node.interpoint)

        print(waypoints_node.curpoint)
        while abs(waypoints_node.get_ang_error()) >= waypoints_node.min_angular_error:
            try:
                waypoints_node.drive(mode="rotate")
                time.sleep(0.1)
            except KeyboardInterrupt:
                print('Interrupted')
                break
        
        print(waypoints_node.curpoint)
        # step 2: forward to the desired position
        waypoints_node.curpoint = waypoints_node.interpoint
        waypoints_node.pid_control.clear(Kp=[0.2, 0.2, 0.05], Ki=[0.0,0.0,0.0], Kd=[0.0,0.0,0.0], setpoint=waypoints_node.setpoint)
        waypoints_node.cur_error = [waypoints_node.setpoint[i] - waypoints_node.curpoint[i] for i in range(3)]

        while waypoints_node.get_dis_error() >= waypoints_node.min_dis_error:
            try:
                waypoints_node.drive(mode="forward")
                time.sleep(0.1)
            except KeyboardInterrupt:
                print('Interrupted')
                break
        
        print(waypoints_node.curpoint)
        # step 3: rotate to the desired angular position
        waypoints_node.pid_control.clear(Kp=[0.2, 0.2, 0.05], Ki=[0.0,0.0,0.0], Kd=[0.0,0.0,0.0], setpoint=waypoints_node.setpoint)
        waypoints_node.cur_error = [waypoints_node.setpoint[i] - waypoints_node.curpoint[i] for i in range(3)]

        while abs(waypoints_node.get_ang_error()) >= waypoints_node.min_angular_error:
            try:
                waypoints_node.drive(mode="rotate")
                time.sleep(0.1)
            except KeyboardInterrupt:
                print('Interrupted')
                break

    waypoints_node.send_end_signal()
