#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from mpi_control import MegaPiController

class WayPointsControllerNode:
    def __init__(self):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0')
        self.calibrated = [0.0] * 4
    
    def joy_callback(self, cmd):
        self.calibrated = [
            1.0 * cmd[0],
            1.0 * cmd[1],
            1.0 * cmd[2],
            1.0 * cmd[3],
        ]
        if max(self.calibrated > 110):
            self.mpi_ctrl.carStop()
        self.mpi_ctrl.setFourMotors(
            self.calibrated[0],
            self.calibrated[1],
            self.calibrated[2],
            self.calibrated[3],
        )
        time.sleep(1)
        
if __name__ == "__main__":
    way_points_ctrl_node = WayPointsControllerNode()
    rospy.init_node('way_points_controller')
    rospy.Subscriber('/joy', Joy, way_points_ctrl_node.joy_callback, queue_size=10)
    rospy.spin()