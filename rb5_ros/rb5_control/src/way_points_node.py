#!/usr/bin/env python
import rospy
from pid_controller import PID  
# from megapi import MegaPi
# from mpi_control import MegaPiController
# from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class WayPointsNode:
    def __init__(self, waypoint_path = '/root/rb5_ws/src/rb5_ros/rb5_control/src/waypoints.txt'):

        self.points_list = self.waypoint_reader(waypoint_path)
        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=10)

        self.v_x, self.v_y, self.rotate = -1, -1, -1
        # self.v_x_base = 50
        # self.v_y_base = 50
        # self.rotate_base = 50
        # self.max_v = 100
        self.km = self.get_kinematic_matrix()
        self.wheels_rotate = [0] * 4
        self.rate = rospy.Rate(10)      # 10hz
        self.cur_point = [0.0, 0.0, 0.0]
        self.dt = 0.1
        self.min_error = 0.01

    
    def drive(self):
        for i in range(len(self.points_list)):
            pid_control = PID(Kp=0.2, Ki=0.0, Kd=0.0, setpoint=self.points_list[i+1])
            self.cur_point = self.points_list[i]

            while not self.get_error(self.points_list[i+1], self.cur_point):
                pid_output = pid_control.update(self.dt, self.cur_point)

                # self.cur_point += [k * self.dt for k in pid_output]
                for k in range(3):
                    self.cur_point[k] += self.dt * pid_output[k]
                
                self.v_x, self.v_y, self.rotate = pid_output[0], pid_output[1], pid_output[2]
                self.wheels_rotate = [
                    self.km[0][0] * self.v_x + self.km[0][1] * self.v_y + self.km[0][2] * self.rotate,
                    self.km[1][0] * self.v_x + self.km[1][1] * self.v_y + self.km[1][2] * self.rotate,
                    self.km[2][0] * self.v_x + self.km[2][1] * self.v_y + self.km[2][2] * self.rotate,
                    self.km[3][0] * self.v_x + self.km[3][1] * self.v_y + self.km[3][2] * self.rotate,
                ]
                
                # max_rotate = max(abs(max(self.wheels_rotate)), abs(min(self.wheels_rotate)))
                # if max_rotate > 100:
                #     self.wheels_rotate = [
                #         self.wheels_rotate[0] / max_rotate * self.max_v,
                #         self.wheels_rotate[1] / max_rotate * self.max_v,
                #         self.wheels_rotate[2] / max_rotate * self.max_v,
                #         self.wheels_rotate[3] / max_rotate * self.max_v,
                #     ]
                
                points_msg = Float64MultiArray(data=self.wheels_rotate)
                self.publisher.publish(points_msg)
                # self.rate.sleep()
                time.sleep(0.1)
    
    
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
    
    def get_error(self, setpoint, curpoint):
        error = [setpoint[i] - curpoint[i] for i in range(len(setpoint))]
        dis_error = math.sqrt(math.pow(error[0], 2) + math.pow(error[1], 2))

        print(dis_error)
        if (dis_error <= self.min_error):
            return True
        
        return False


if __name__ == "__main__":
    rospy.init_node("way_points")
    way_points_node = WayPointsNode()
    way_points_node.drive()
