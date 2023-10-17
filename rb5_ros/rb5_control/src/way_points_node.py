#!/usr/bin/env python
import rospy
from pid_controller import PID
from megapi import MegaPi
from mpi_control import MegaPiController
# from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import numpy as np

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class WayPointsNode:
    def __init__(self, waypoint_path = 'src/waypoints.txt'):
        # self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=10)
        # self.points_list = [
        #     [0, 0, 0],
        #     [1, 0, 0],
        #     [1, 1, 1.57],
        #     [2, 1, 0],
        #     [2, 2, -1.57],
        #     [1, 1, -0.78],
        #     [0, 0, 0],    
        # ]

        self.points_list = self.waypoint_reader(waypoint_path)
        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=10)

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
            
            self.publisher.publish(points_msg)
    
    
    def get_kinematic_matrix(self):
        ly = 0.112      # distance between front wheels and back wheels (m)
        lx = 0.132      # distance between front wheels (m)
        r = 0.06        # radius of the wheels (m)
        
        l1 = [1, -1, -(lx+ly)]
        l2 = [1, 1, (lx+ly)]
        l3 = [1, 1, -(lx+ly)]
        l4 = [1, -1, (lx+ly)]

        l1 = [i * (1.0 / r) for i in l1]
        l2 = [i * (1.0 / r) for i in l1]
        l3 = [i * (1.0 / r) for i in l1]
        l4 = [i * (1.0 / r) for i in l1]

        kinematic_matrix = [l1, l2, l3, l4]

        return kinematic_matrix

    def waypoint_reader(self, file_path):
        waypoints = np.loadtxt(file_path, delimiter=',')

        return waypoints.tolist()


if __name__ == "__main__":
    way_points_node = WayPointsNode()
    rospy.init_node("way_points")
    way_points_node.drive()
