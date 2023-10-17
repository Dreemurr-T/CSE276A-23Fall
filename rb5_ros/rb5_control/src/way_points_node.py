#!/usr/bin/env python
import rospy

from megapi import MegaPi
from mpi_control import MegaPiController
from sensor_msgs.msg import Joy
import math
import numpy as np


MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class WayPointsNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=10)
        self.points_list = [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 1.57],
            [2, 1, 0],
            [2, 2, -1.57],
            [1, 1, -0.78],
            [0, 0, 0],    
        ]
        self.v_x, self.v_y, self.rotate = 0, 0, 0
        self.v_x_base = 50
        self.v_y_base = 50
        self.rotate_base = 50
        self.max_v = 100
        self.km = self.get_kinematic_matrix()
        self.wheels_rotate = [0] * 4
        
    
    def drive(self):
        for i in range(len(self.points_list) - 1):
            diff = [
                self.points_list[i + 1][0] - self.points_list[i][0],
                self.points_list[i + 1][1] - self.points_list[i][1],
                self.points_list[i + 1][1] - self.points_list[i][1],
            ]
            
            if diff[0] >= 0:
                self.v_x = self.v_x_base
            elif diff[0] == 0:
                self.v_x = 0
            elif diff[0] < 0:
                self.v_x = -self.v_x_base
                
            if diff[1] >= 0:
                self.v_y = self.v_y_base
            elif diff[1] == 0:
                self.v_y = 0
            elif diff[1] < 0:
                self.v_y = -self.v_y_base
                
            if diff[2] >= 0:
                self.rotate = self.rotate_base
            elif diff[0] == 0:
                self.rotate = 0
            elif diff[0] < 0:
                self.rotate = -self.rotate_base
            
            self.wheels_rotate = [
                self.km[0][0] * self.v_x + self.km[0][1] * self.v_y + self.km[0][2] * self.rotate,
                self.km[1][0] * self.v_x + self.km[1][1] * self.v_y + self.km[1][2] * self.rotate,
                self.km[2][0] * self.v_x + self.km[2][1] * self.v_y + self.km[2][2] * self.rotate,
                self.km[3][0] * self.v_x + self.km[3][1] * self.v_y + self.km[3][2] * self.rotate,
            ]
            
            max_rotate = max(self.wheels_rotate)
            if max_rotate > 100:
                self.wheels_rotate = [
                    self.wheels_rotate[0] / max_rotate * self.max_v,
                    self.wheels_rotate[0] / max_rotate * self.max_v,
                    self.wheels_rotate[0] / max_rotate * self.max_v,
                    self.wheels_rotate[0] / max_rotate * self.max_v,
                ]
            
            points_msg = self.wheels_rotate
            
            self.pub_joy.publish(points_msg)
    
    
    def get_kinematic_matrix(self):
        lfw = 0.112      # distance between front wheels and back wheels (m)
        lw = 0.132       # distance between front wheels (m)

        L = [math.sqrt(math.pow(lfw/2, 2) + math.pow(lw/2, 2))] * 4     # distance between wheels and the base
        alpha = [math.pi / 4, -math.pi / 4, 3 * math.pi / 4, -3 * math.pi / 4]
        beta = [math.pi / 2, -math.pi/2, math.pi/2, -math.pi/2]
        gamma = [-math.pi/4, math.pi/4, math.pi/4, -math.pi/4]

        l1, l2, l3 = [], [], []

        for i in range(4):
            l1.append(math.cos(beta[i]-gamma[i])/math.sin(gamma[i]))
            l2.append(math.sin(beta[i]-gamma[i])/math.sin(gamma[i]))
            l3.append(L[i] * math.sin(beta[i]-gamma[i]-alpha[i])/math.sin(gamma[i]))

        l = [l1, l2, l3]
        kinematic_matrix = np.transpose(l).tolist()

        return kinematic_matrix






if __name__ == "__main__":
    way_points_node = WayPointsNode()
    rospy.init_node("way_points")
    way_points_node.drive()
