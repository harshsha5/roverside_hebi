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
from geometry_msgs.msg import PoseStamped,Twist
import hebi
import time
import numpy as np
from sensor_msgs.msg import Joy,Image,Range
from std_msgs.msg import String
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2



def open_csv(filio):
    with open(filio,'w') as file_1:
    	pass
    file_1.close()

def write_csv(data_list,file_1):
    file_1.write(data_list)

def get_msg(topic_name,data_type):
    msg=rospy.wait_for_message(topic_name, data_type)
    return msg

def get_msg(topic_name,data_type,name_folder,count):
    msg=rospy.wait_for_message(topic_name, data_type)
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
	image_name=str(count)+'-'+str(time)+'.jpeg'
        cv2.imwrite(name_folder+image_name, cv2_img)
        #rospy.sleep(1)

    return [time,image_name]
	
    
def sensor_capture():
    rospy.init_node('sensor_capture', anonymous=True)
    open_csv('sensor_data.csv')
    count=0
    with open('sensor_data.csv','a') as file_1:
    

    	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/cmd_vel", Twist)
		print("Sensor Capture file running..... ", float(msg.linear.x))
		if(float(msg.linear.x)==0.5):
			print("hi")
			count+=1

	    		'''msg_hebi=get_msg('motor_para', PoseStamped)
	    		msg_real_sense=get_msg('/camera/color/image_raw', Image, '/home/ashish/data/real-sense/')
			msg_decawave=get_msg('/range', Range)
			msg_360_fish_0=get_msg('/usb_cam0/image_raw', Image, '/home/ashish/data/360-fish-0/')
			msg_360_fish_1=get_msg('/usb_cam1/image_raw', Image, '/home/ashish/data/360-fish-1/')
			msg_360_fish_2=get_msg('/usb_cam2/image_raw', Image, '/home/ashish/data/360-fish-2/')
			msg_down_fish=get_msg('/camera/image_color', Image, '/home/ashish/data/down-fish/')
			data_list=[count,msg_hebi.header.stamp,msg_header.pose.orientation.x,msg_header.pose.orientation.y,msg_real_sense[0],msg_real_sense[1],msg_decawave.header.stamp,msg_decawave.range,msg_360_fish_0[0],msg_360_fish_0[1],msg_360_fish_1[0],msg_360_fish_1[1],msg_360_fish_2[0],msg_360_fish_2[1],msg_down_fish[0],msg_down_fish[1]]'''

			msg_real_sense=get_msg('/camera/color/image_raw', Image, '/home/ashish/data/real-sense/',count)
			write_csv(data_list,file_1)
			print("bye")
	    		continue
    
    	rospy.spin()


if __name__ == '__main__':
    try:
        sensor_capture()
    except rospy.ROSInterruptException:
        pass
