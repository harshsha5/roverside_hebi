#!/usr/bin/env python

"""
This script is used to capture sensor data from all the 7 sensors (or actuators)- Decawave UWM, 360 Fish-eye (3 cameras), Downward facing fish-eye, Intel Real Sense, Hebi Motor angle using joystick button
"""

__version__ = "0.0.2"
__author__ = "Harsh Sharma"
__email__ = "harshsha@cs.cmu.edu"
__website__ = "http://mrsdprojects.ri.cmu.edu/2019teami/"
__copyright__ = "Copyright (C) 2019, the H#SH Robotics. All rights reserved."

import rospy
import tf
import sys
import os
from geometry_msgs.msg import PoseStamped
import hebi
import time
import numpy as np
from sensor_msgs.msg import Joy,Image,Range
from std_msgs.msg import String
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import csv


def open_csv(filio):
    with open(filio,'w') as file_1:
	pass
    
    file_1.close()

def write_csv(data_list,wr):
    '''for entries in data_list:
            wr.write(entries)
            wr.write(",")
    wr.write(';')'''
    wr.writerow(data_list)
    return

def get_msg_twoargs(topic_name,data_type):
    msg=rospy.wait_for_message(topic_name, data_type)
    return msg

def get_msg(topic_name,data_type,name_folder,count):
    msg=rospy.wait_for_message(topic_name, data_type)
    try:
        # Convert your ROS Image message to OpenCV2
	bridge=CvBridge()
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = str(msg.header.stamp)
	image_name='img_'+str(count)+'.jpeg'
        cv2.imwrite(name_folder+image_name, cv2_img)
        #rospy.sleep(1)

    return [time,image_name]

def make_folders(my_argv):
	n=my_argv[1]
	if('RS' in my_argv):
		print("Real Sense detected")
		path = "/home/ashish/data/real-sense{0}".format(n)
		create_directory('RS',path)
		RS_FOLDER=path
	if('PG' in my_argv):
		print("Point Grey detected")
		path = "/home/ashish/data/PG{0}".format(n)
		print(path)
		create_directory('PG',path)
		PG_FOLDER=path

def create_directory(stro,path):
		try:  
    			os.mkdir(path)
		except OSError:  
    			print (stro," directory already exists")	
			exit()	

	
    
def sensor_capture(my_argv):
    rospy.init_node('sensor_capture', anonymous=True)
    open_csv('sensor_data.csv')
    count=0
    make_folders(my_argv)
    with open('sensor_data.csv','a') as file_1:
	wr = csv.writer(file_1)
    

    	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("ak1/joy", Joy)
		print("Sensor Capture file running..... ", int(msg.axes[7]))
		if(int(msg.axes[7])==1):
			data_list=[]
			count+=1
			data_list.append(count)
			print("Button Snapped")
			if('DW' in my_argv):
				msg_decawave=get_msg_twoargs('/range', Range)
				data_list.append(msg_decawave.header.stamp)
				data_list.append(msg_decawave.range)
				print("Decawave captured")
			
			if('HEBI' in my_argv):
				msg_hebi=get_msgtwoargs('motor_para', PoseStamped)
				data_list.append(msg_hebi.header.stamp)
				data_list.append(msg_header.pose.orientation.x)
				data_list.append(msg_header.pose.orientation.y)

			if('RS' in my_argv):
				msg_real_sense=get_msg('/camera/color/image_raw', Image, RS_FOLDER,count)
				data_list.append(msg_real_sense[0])
				data_list.append(msg_real_sense[1])
			
			if('PG' in my_argv):
				msg_down_fish=get_msg('/pg_camera/image_color', Image, PG_FOLDER,count)
				data_list.append(msg_down_fish[0])
				data_list.append(msg_down_fish[1])

			write_csv(data_list,wr)
	    		continue
    
    	rospy.spin()


if __name__ == '__main__':
    PG_FOLDER=''
    RS_FOLDER=''
    my_argv = rospy.myargv(argv=sys.argv)
    print(my_argv)
    try:
        sensor_capture(my_argv)
    except rospy.ROSInterruptException:
        pass
