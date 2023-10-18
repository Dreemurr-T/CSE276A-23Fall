#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64MultiArray
from mpi_control import MegaPiController


MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class WayPointsControllerNode:
    def __init__(self):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0')
        self.calibrated = [0.0] * 4
    
    def joy_callback(self, msg):
        self.calibrated = [
            1.0 * msg.data[0] / 7.53 * 50,
            1.0 * msg.data[1] / 7.53 * 50,
            1.27 * msg.data[2] / 7.53 * 50,
            1.0 * msg.data[3] / 7.53 * 50,
        ]

        if abs(max(self.calibrated)) > 110 or abs(min(self.calibrated)) > 110:
            self.mpi_ctrl.carStop()
        else:
            self.mpi_ctrl.setFourMotors(
                self.calibrated[0],
                self.calibrated[1],
                self.calibrated[2],
                self.calibrated[3],
            )
        time.sleep(0.1)
        
if __name__ == "__main__":
    rospy.init_node('motor_controller')
    way_points_ctrl_node = WayPointsControllerNode()
    
    # rospy.Subscriber('/joy', Joy, way_points_ctrl_node.joy_callback, queue_size=10)
    rospy.Subscriber("/velocity", Float64MultiArray, way_points_ctrl_node.joy_callback, queue_size=10)
    rospy.spin()