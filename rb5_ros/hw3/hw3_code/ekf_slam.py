"""
Reference: https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html
"""

#!/usr/bin/env python
import rospy
from pid_controller import PID
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np
import tf
import tf2_ros
from collections import defaultdict

STATE_SIZE = 3      # State size of the robot [x,y,yaw]
LM_SIZE = 2         # LM state size [x,y]

class EKF_SLAM:
    def __init__(self):
        # publish the velocity of each wheel to the velocity topic
        self.publisher = rospy.Publisher("/velocity", Float64MultiArray, queue_size=5)

        self.V_X, self.V_Y, self.angular = 0.0, 0.0, 0.0     # velocity under world coordinate
        self.v_x, self.v_y = 0.0, 0.0                        # velocity under robot coordinate

        self.theta = 0.0                                    # angle between the robot coordinate and the world coordinate
        self.km = self.get_kinematic_matrix()               # kinematic matrix
        self.wheels_angular = [0.0] * 4                     # angular speed of the four wheels 
        self.min_angular = 6.5                             # minimum angular speed of each wheel (rad/s)
        self.max_angular = 9.5                            # maximum angular speed of each wheel (rad/s)

        self.lmDict = defaultdict(int)                      # Store the index i for lm states in sEst
        self.lmNew = set()                                  # Store the lm which is first seen
        self.lmCur = set()                                  # Store the lm which has been seen before
        self.lmNum = 0                                      # Store the number of landmarks

        self.curpoint = np.zeros(3)                         # current position of the robot
        self.dt = 0.05                                       # timestep for pid controller
        self.cur_error = np.zeros(3)

        self.sEst = np.zeros((3,1))
        self.ut = np.zeros((3,1))
        self.sigmaEst = np.diag([1, 1, 1])

        self.min_error = 0.1                  
        self.pid_control = PID(Kp=0.2, Ki=0.0, Kd=0.0, setpoint=self.curpoint)

        self.listener = tf.TransformListener()

    def drive(self):
        self.curpoint = self.sEst[:3, 0]
        self.display_status()

        self.ut[:, 0], self.cur_error = self.pid_control.update(self.dt, self.curpoint)
        self.theta = self.curpoint[2]

        self.V_X, self.V_Y, self.angular = self.ut[0, 0], self.ut[1, 0], self.ut[2, 0]
        self.v_x, self.v_y = self.world_to_robot(self.V_X, self.V_Y, self.theta)

        self.wheels_angular = [
            self.km[0][0] * self.v_x + self.km[0][1] * self.v_y + self.km[0][2] * self.angular,
            self.km[1][0] * self.v_x + self.km[1][1] * self.v_y + self.km[1][2] * self.angular,
            self.km[2][0] * self.v_x + self.km[2][1] * self.v_y + self.km[2][2] * self.angular,
            self.km[3][0] * self.v_x + self.km[3][1] * self.v_y + self.km[3][2] * self.angular,
        ]
        
        self._clamp()       # set angular speed of the wheels to threshold and recompute vx, vy

        self.V_X, self.V_Y = self.robot_to_world(self.v_x, self.v_y, self.theta)
        self.ut[0, 0], self.ut[1, 0], self.ut[2, 0] = self.V_X, self.V_Y, self.angular

        # self.curpoint += self.ut[:, 0] * self.dt
        # self.curpoint[2] = pi2pi(self.curpoint[2])

        # print(self.curpoint)

        points_msg = Float64MultiArray(data=self.wheels_angular)
        self.publisher.publish(points_msg)
    
    def update_pos(self):
        self.sEst, self.sigmaEst = self.predict(self.sEst, self.sigmaEst, self.ut)  # prediction step
        self.sEst, self.sigmaEst = self.update(self.sEst, self.sigmaEst)            # updating step
        self.sEst, self.sigmaEst = self.observe(self.sEst, self.sigmaEst)           # Augment the state and sigma

        self.sEst[2, 0] = pi2pi(self.sEst[2, 0])
        
        self.lmNew.clear()
        self.lmCur.clear()
        self.listener.clear()

    def display_status(self):
        for frame_name in self.lmDict:
            i = self.lmDict[frame_name] - 1
            lm = self.sEst[STATE_SIZE + i * LM_SIZE: STATE_SIZE + (i+1) * LM_SIZE, 0]

            print(frame_name, lm)
        print(self.sEst[0:3])
        # print(np.diagonal(self.sigmaEst))

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
        return np.linalg.norm(self.cur_error)

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

    # Predict the robot pose
    def predict(self, sEst, sigmaEst, ut):
        assert len(sEst) == len(sigmaEst)

        F = np.eye(len(sEst))
        G = np.diag([self.dt, self.dt, self.dt])
        Qt = np.diag([0.5, 0.5, np.deg2rad(20.0)])**2 # Change in covariance

        sEst[0:STATE_SIZE] = sEst[0:STATE_SIZE] + np.dot(G, ut)
        
        sigmaEst = np.dot(np.dot(F, sigmaEst), F.T)
        sigmaEst[0:STATE_SIZE, 0:STATE_SIZE] += Qt

        return sEst, sigmaEst
    
    # Correct the prediction based on previously observed landmarks
    def update(self, sEst, sigmaEst):
        frame_list = self.listener.getFrameStrings()
        for frame in frame_list:
            if frame not in self.lmDict and frame[0] == 'm':
                self.lmNew.add(frame)
            elif frame in self.lmDict and frame[0] == 'm':
                self.lmCur.add(frame)

        for frame in self.lmCur:
            try:
                now = rospy.Time()
                camera_name = 'camera_' + frame[-1]
                # wait for the transform ready from the camera to the marker for 0.1 second.
                self.listener.waitForTransform(camera_name, frame, now, rospy.Duration(0.3))
                # extract the transform marker pose in the  coordinate.
                (trans, rot) = self.listener.lookupTransform(camera_name, frame, now)

                frame_id = self.lmDict[frame] - 1     # the id starts from 1
                lm = sEst[STATE_SIZE + LM_SIZE * frame_id : STATE_SIZE + LM_SIZE * (frame_id + 1), :]
                trans = np.array(trans)

                theta = sEst[2, 0]

                y, S, H = self.cal_innovation(lm, sEst, sigmaEst, trans, frame_id, theta)
                K = np.dot(np.dot(sigmaEst, H.T), np.linalg.inv(S))
                sEst = sEst + np.dot(K, y)
                sigmaEst = np.dot((np.eye(len(sEst)) - np.dot(K, H)), sigmaEst)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
        
        self.listener.clear()

        sEst[2] = pi2pi(sEst[2])
        return sEst, sigmaEst
    
    def cal_innovation(self, lm, sEst, sigmaEst, trans, frame_id, theta):
        correction = 0.274
        trans = trans + trans / (1 - correction) * correction         # correct the pose estimation

        dx, dy = self.robot_to_world(trans[2], -trans[0], theta)
        d = math.sqrt(dx**2 + dy**2)
        angle = pi2pi(math.atan2(dy, dx) - theta)
        z = np.array([[d, angle]])

        delta = lm - sEst[0:2]
        q = np.dot(delta.T, delta)[0, 0]
        zangle = pi2pi(math.atan2(delta[1, 0], delta[0, 0]) - theta)
        zp = np.array([[math.sqrt(q), pi2pi(zangle)]])

        y = (z - zp).T      # y = innovation
        y[1] = pi2pi(y[1])

        H = self.jacobH(q, delta, frame_id + 1)
        Rt = d * (np.diag([0.1, 0.1]) ** 2)
        S = np.dot(np.dot(H, sigmaEst), H.T) + Rt
        # S = np.dot(np.dot(H, sigmaEst), H.T)

        return y, S, H
        
    def jacobH(self, q, delta, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

        G = G / q
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * self.lmNum))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * self.lmNum - 2 * i))))

        F = np.vstack((F1, F2))
        H = np.dot(G, F)

        return H

    # Expand new landmarks
    def observe(self, sEst, sigmaEst):
        if len(self.lmNew) == 0:
            return sEst, sigmaEst
        
        sAug = sEst
        sigmaAug = sigmaEst
        correction = 0.274

        for frame_name in self.lmNew:
            try:
                now = rospy.Time()
                camera_name = 'camera_' + frame_name[-1]
                # wait for the transform ready from the camera to the marker for 0.1 second.
                self.listener.waitForTransform(camera_name, frame_name, now, rospy.Duration(0.1))
                # extract the transform marker pose in the  coordinate.
                (trans, rot) = self.listener.lookupTransform(camera_name, frame_name, now)

                trans = np.array(trans)
                trans = trans + trans / (1 - correction) * correction
                dx, dy = self.robot_to_world(trans[2], -trans[0], sEst[2, 0])
                lm_pos = np.zeros((2, 1))
                lm_pos[0, 0] = sEst[0, 0] + dx
                lm_pos[1, 0] = sEst[1, 0] + dy

                sAug = np.vstack((sEst, lm_pos))
                sigmaAug = np.vstack((np.hstack((sigmaEst, np.zeros((len(sEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(sEst))), np.diag([1.0, 1.0])))))
                sEst = sAug
                sigmaEst = sigmaAug

                self.lmNum += 1
                self.lmDict[frame_name] = self.lmNum

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")

        self.listener.clear()
        return sAug, sigmaAug

def get_interpoints(setpoint, curpoint):
    interpoint1 = np.zeros(3)
    interpoint2 = np.zeros(3)

    interpoint1[0:2] = curpoint[0:2]
    interpoint2[0:2] = setpoint[0:2]

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

def pi2pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

if __name__ == "__main__":
    rospy.init_node("waypoints")
    slam = EKF_SLAM()
    points_list = np.array([
        [0.0, 0.0, 0.0],
        # [0.0, 0.0, math.pi / 2],
        [0.5, 0.0, 0.0],
        [0.5, 0.5, math.pi / 2],
        [0.0, 0.5, math.pi],
        [0, 0, 0],
        # [0.5, 0.0, 0.0],
        # [0.5, 0.5, math.pi / 2],
        # [0.0, 0.5, math.pi],
        # [0, 0, 0],
    ])

    for i in range(len(points_list)-1):
        setpoint = points_list[i+1]
        prepoint = points_list[i]
        
        interpoint1, interpoint2 = get_interpoints(setpoint, prepoint)

        slam.cur_error = interpoint1 - slam.curpoint
        slam.cur_error[2] = pi2pi(slam.cur_error[2])

        # step 1: rotate to the desired direction
        slam.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=interpoint1)

        while slam.get_error() >= slam.min_error:
            slam.drive()
            time.sleep(0.075)
            slam.update_pos()

        # step 2: forward to the desired position
        slam.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=interpoint2)
        slam.cur_error = interpoint2 - slam.curpoint
        slam.cur_error[2] = pi2pi(slam.cur_error[2])

        while slam.get_error() >= slam.min_error:
            slam.drive()
            time.sleep(0.075)
            slam.update_pos()
            
        # step 3: rotate to the desired angular position
        slam.pid_control.clear(Kp=0.2, Ki=0, Kd=0, setpoint=setpoint)
        slam.cur_error = setpoint - slam.curpoint
        slam.cur_error[2] = pi2pi(slam.cur_error[2])

        while slam.get_error() >= slam.min_error:
            slam.drive()
            time.sleep(0.075)
            slam.update_pos()

    slam.send_end_signal()