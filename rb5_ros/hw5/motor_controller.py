#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64MultiArray
from mpi_control import MegaPiController

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class MotorControllerNode:
    def __init__(self):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0')
        self.calibrated = [0.0] * 4
    
    def callback(self, msg):
        self.calibrated = [
            1.0 * msg.data[0] / 7.16 * 45,
            1.0 * msg.data[1] / 7.16 * 45,
            1.1 * msg.data[2] / 7.16 * 45,
            1.0 * msg.data[3] / 7.16 * 45,
        ]

        self.mpi_ctrl.setFourMotors(
            self.calibrated[0],
            -self.calibrated[1],
            self.calibrated[2],
            -self.calibrated[3],
        )
        # time.sleep(0.05)

        
if __name__ == "__main__":
    rospy.init_node('motor_controller')
    motor_ctrl_node = MotorControllerNode()
    
    rospy.Subscriber("/velocity", Float64MultiArray, motor_ctrl_node.callback, queue_size=1)
    rospy.spin()