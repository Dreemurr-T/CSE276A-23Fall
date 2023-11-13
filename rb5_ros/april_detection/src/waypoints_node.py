#!/usr/bin/env python
import rospy
from pid_controller import PID
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np
import tf
from tf.transformations import euler_from_quaternion

marker_translation = [[0, -0.1], [1.4, 0], [1, 2.28]]

class WayPointsNode:
    def __init__(self):

        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=10)

        self.V_X, self.V_Y, self.angular = 0.0, 0.0, 0.0     # velocity under world coordinate
        self.v_x, self.v_y = 0.0, 0.0                        # velocity under robot coordinate
        self.theta = 0.0                                    # angle between the robot coordinate and the world coordinate
        self.km = self.get_kinematic_matrix()               # kinematic matrix
        self.wheels_angular = [0.0] * 4                     # angular speed of the four wheels 
        self.min_angular = 7.53                            # minimum angular speed of each wheel (rad/s)
        self.max_angular = 15.06                            # maximum angular speed of each wheel (rad/s)

        
        self.curpoint = [0.0, 0.0, 0.0]                     # current position of the robot
        self.dt = 0.1                                       # timestep for pid controller
        self.cur_error = [0.0, 0.0, 0.0]
        self.min_dis_error = 0.12
        self.min_ang_error = 0.13                  
        self.pid_control = PID(Kp=0.2, Ki=0.0, Kd=0.0, setpoint=[0, 0, 0])
        self.points = []

        self.listener = tf.TransformListener()

    
    def drive(self, mode='forward', flag=False):
        pid_output, self.cur_error = self.pid_control.update(self.dt, self.curpoint)
        self.theta = self.curpoint[2]

        self.V_X, self.V_Y, self.angular = pid_output[0], pid_output[1], pid_output[2]  # world coordinate

        self.v_x, self.v_y = self.world_to_robot(self.V_X, self.V_Y, self.theta)

        if mode == "rotate":
            self.v_x, self.v_y = 0.0, 0.0
        # if not flag and mode == "forward":
        #     self.v_x = math.sqrt(self.v_x**2 + self.v_y**2)
        #     self.v_y = 0
        #     self.angular = 0

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

        self.points.append(self.curpoint)
        print(self.curpoint)
        
        points_msg = Float64MultiArray(data=self.wheels_angular)
        self.publisher.publish(points_msg)
    
    def send_end_signal(self):
        end_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        self.publisher.publish(end_msg)
    
    def get_kinematic_matrix(self):
        lx = 0.066      # distance between front wheels (m)
        ly = 0.056      # distance between front wheels and back wheels (m)
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
    
    def get_dis_error(self):
        error = math.sqrt(self.cur_error[0] ** 2 + self.cur_error[1] ** 2)

        return error

    def get_ang_error(self):
        return abs(self.cur_error[2])

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

    def curpoint_by_detection(self, mode):
        ids = [0, 1, 2]
        curpoint = [0, 0, 0]
        detection_flag = False
        body_names = ['body_0', 'body_1', 'body_2']
        camera_names = ['camera_0', 'camera_1', 'camera_2']
        marker_names = ['marker_0', 'marker_1', 'marker_2']
        now = rospy.get_rostime()
        latest = 0
        detection_id = 0

        for i in ids:
            try:
                (trans, rot) = self.listener.lookupTransform('world', body_names[i], rospy.Time(0))
                time = self.listener.getLatestCommonTime(marker_names[i], camera_names[i])
                error = [marker_translation[i][0] - self.curpoint[0], marker_translation[i][1] - self.curpoint[1]]
                dis_error = math.sqrt(error[0]**2 + error[1]**2)

                if time.secs > latest and (now.secs - time.secs) == 0 and dis_error <= 1.2:
                    latest = time.secs
                    (roll, pitch, yaw) = euler_from_quaternion(rot)
                    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
                    curpoint = [trans[0], trans[1], yaw]
                    detection_flag = True
                    detection_id = i

                    if i == 1 and mode == "rotate":
                        detection_flag = False
                    if i == 0 and mode == "forward":
                        curpoint[1] -= 0.1
                    if i == 2 and mode == "rotate":
                        curpoint[2] -= 0.1

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        return curpoint, detection_flag, detection_id
        

def get_interpoints(setpoint, curpoint):
    interpoint1 = [0.0, 0.0, 0.0]
    interpoint2 = [0.0, 0.0, 0.0]
    for i in range(2):
        interpoint1[i] = curpoint[i]
        interpoint2[i] = setpoint[i]

    space = 0
    dy = setpoint[1] - curpoint[1]
    dx = setpoint[0] - curpoint[0]

    d = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    if d != 0:
        theta = math.acos(dx/d)
    else:
        theta = curpoint[2]

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
    
    interpoint1[2] = theta
    interpoint2[2] = theta

    return interpoint1, interpoint2

if __name__ == "__main__":
    rospy.init_node("way_points")
    waypoints_node = WayPointsNode()
    points_list = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 2.0, math.pi],
        [0.0, 0.0, 0.0]
    ]

    for i in range(len(points_list)-1):
        setpoint = points_list[i+1]
        waypoints_node.curpoint = points_list[i]
        
        interpoint1, interpoint2 = get_interpoints(setpoint, waypoints_node.curpoint)

        waypoints_node.cur_error = [interpoint1[i] - waypoints_node.curpoint[i] for i in range(3)]
        waypoints_node.cur_error[2] = (waypoints_node.cur_error[2] + math.pi) % (2 * math.pi) - math.pi

        # print(waypoints_node.setpoint, waypoints_node.curpoint, waypoints_node.interpoint)

        # step 1: rotate to the desired direction
        waypoints_node.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=interpoint1)

        while waypoints_node.get_ang_error() >= waypoints_node.min_ang_error:
            try:
                curpoint, flag, detection_id = waypoints_node.curpoint_by_detection(mode="rotate")
                if flag :
                    waypoints_node.curpoint[2] = curpoint[2]

                waypoints_node.drive(mode="rotate", flag=flag)
                time.sleep(0.15)
                
            except KeyboardInterrupt:
                print('Interrupted')
                break
        
        # step 2: forward to the desired position
        # waypoints_node.curpoint = interpoint1
        waypoints_node.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=interpoint2)
        waypoints_node.cur_error = [interpoint2[i] - waypoints_node.curpoint[i] for i in range(3)]

        while waypoints_node.get_dis_error() >= waypoints_node.min_dis_error:
            try:
                curpoint, flag, detection_id = waypoints_node.curpoint_by_detection(mode="forward")
                if flag and detection_id == 1:
                    waypoints_node.curpoint[0] = curpoint[0]
                elif flag and detection_id != 1:
                    waypoints_node.curpoint = curpoint
                waypoints_node.drive(mode="forward", flag=flag)
                time.sleep(0.15)
            except KeyboardInterrupt:
                print('Interrupted')
                break
        
        
        # step 3: rotate to the desired angular position
        # waypoints_node.curpoint = interpoint2
        waypoints_node.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=setpoint)
        waypoints_node.cur_error = [setpoint[i] - waypoints_node.curpoint[i] for i in range(3)]

        while waypoints_node.get_ang_error() >= waypoints_node.min_ang_error:
            try:
                waypoints_node.drive(mode="rotate", flag=flag)
                time.sleep(0.15)

            except KeyboardInterrupt:
                print('Interrupted')
                break

    waypoints_node.send_end_signal()
    np.savetxt("points.txt", waypoints_node.points, fmt = '%f', delimiter = ',')