#!/usr/bin/env python

"""
hebi_rov.py
Automatically pointing the claw manipulator towards the companion rover ring,
using kinematic control approach.
"""

__version__ = "0.0.1"
__author__ = "Harsh Sharma"
__email__ = "harshsha@cs.cmu.edu"
__website__ = "http://mrsdprojects.ri.cmu.edu/2019teami/"
__copyright__ = "Copyright (C) 2019, the H#SH Robotics. All rights reserved."

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import hebi
import time
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def auto_pointing():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'auto_pointing' node so that multiple nodes can run
    # simultaneously.
    rospy.init_node('auto_pointing', anonymous=True)
    
    hebi_modules = [('arm', 'joint_0'), ('arm', 'joint_1')]

    theta_bounds = [[-.7, 1.4], [-0.8, 0.8]]
    #theta_bounds = [-.1.32, 0.8]


    # discover HEBI modules
    count_found = 0
    lookup = hebi.Lookup()
    time.sleep(2)  # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
        print('{0} | {1}'.format(entry.family, entry.name))
        if (entry.family, entry.name) in hebi_modules:
            count_found = count_found + 1

    if len(hebi_modules) != count_found:
        print('Some HEBI module(s) is missing (%d/%d)...' % (count_found, len(hebi_modules)))
        exit()

    # create a HEBI control group from a set of names
    hebi_group = lookup.get_group_from_names([m[0] for m in hebi_modules], [m[1] for m in hebi_modules])
    print('HEBI control group created (size=%d)' % (hebi_group.size))
    print(hebi_group)

    group_command = hebi.GroupCommand(hebi_group.size)

    #PUBLISH ON TOPIC
    pub = rospy.Publisher('motor_para', PoseStamped, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    goal = PoseStamped()

    # create TF listener
    tf_listener = tf.TransformListener()

    # initialize HEBI actuator position control parameters
    # update_alpha = 0.5
    theta = np.array([0.0, 0.0])
    stepo = 0.8
    rate = rospy.Rate(20)
    #rospy.Subscriber("joy", Joy, callback)
    while not rospy.is_shutdown():

    	#stepo+=-0.01	#positive means anti-clockwise rotation
	msg = rospy.wait_for_message("ak2/joy", Joy)
	stepo=stepo+.02*msg.axes[6]
    	theta=np.array([stepo, 0.0])

	# Fill in feedback
	group_feedback = hebi.GroupFeedback(hebi_group.size)
	group_feedback = hebi_group.get_next_feedback(reuse_fbk=group_feedback)

	# Retrieve positions:
	positions = group_feedback.position

	goal.header.stamp=rospy.Time.now()
	goal.pose.orientation.x=positions[0] #This is the pan of the hebi motor
	goal.pose.orientation.y=positions[1] #This is the tilt of the hebi motor
	pub.publish(goal)


    # send command to HEBI control group
        group_command.position = theta
        hebi_group.send_command(group_command)
	time.sleep(.2)	
	print('Position Feedback:\nmotor_actual_position: {0}\tstepo: {1}\t'.format(positions,stepo))
        rate.sleep()

    #rospy.Subscriber("ak2/joy", Joy, callback)


    # rospy.spin() # use spin only in pure event handling programming model

if __name__ == '__main__':
    try:
        auto_pointing()
    except rospy.ROSInterruptException:
        pass
