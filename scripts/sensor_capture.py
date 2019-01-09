#!/usr/bin/env python

"""
This script is used to capture sensor data from all the 7 sensors (or actuators)- Decawave UWM, 360 Fish-eye (3 cameras), Downward facing fish-eye, Intel Real Sense, Hebi Motor angle
"""

__version__ = "0.0.1"
__author__ = "Harsh Sharma"
__email__ = "harshsha@cs.cmu.edu"
__website__ = "http://mrsdprojects.ri.cmu.edu/2018teami/"
__copyright__ = "Copyright (C) 2019, the H#SH Robotics. All rights reserved."

import rospy
import tf
# from std_msgs.msg import String, Float32, UInt16
# from sensor_msgs.msg import Imu, JointState, Joy
# from geometry_msgs.msg import Vector3Stamped, Twist
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import hebi
import time
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def sensor_capture():
    rospy.init_node('sensor_capture', anonymous=True)
    

    while not rospy.is_shutdown():
	msg = rospy.wait_for_message("ak2/joy", Joy)
	print("Sensor Capture file running..... ", int(msg.axes[7]))
	if(int(msg.axes[7])==1):
	    msg_hebi=rospy.wait_for_message('motor_para', PoseStamped)
	    print('Pan of the hebi motor is ', msg_hebi.pose.orientation.x)
	    continue
    
    rospy.spin()


if __name__ == '__main__':
    try:
        sensor_capture()
    except rospy.ROSInterruptException:
        pass
