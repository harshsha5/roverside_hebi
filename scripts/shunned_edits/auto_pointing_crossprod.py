#!/usr/bin/env python

"""
auto_pointing_crossprod.py
Automatically pointing the claw manipulator towards the companion rover ring,
using cross product metric.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2018, the Moon Wreckers. All rights reserved."

import rospy
import tf
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

import hebi
import time
import numpy as np

MOCK_TEST = False
TWO_POINT_TEST = False


def sin(x):
    return np.sin(x)


def cos(x):
    return np.cos(x)


def conj(x):
    return np.conj(x)


def calc_d_theta1(theta1, theta2, x_c_x, x_c_y, x_c_z):
    d_theta1 = - (((31*sin(conj(theta2))*(x_c_x - (3*sin(conj(theta1)))/40))/100 - (31*cos(conj(theta1))*cos(conj(theta2))*(x_c_z - 9/100))/100)*((93*cos(theta1)*sin(theta2))/4000 - (31*cos(theta2)*sin(theta1)*(conj(x_c_z) - 9/100))/100))/2 - (((31*sin(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 - (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_z - 9/100))/100)*((93*sin(theta1)*sin(theta2))/4000 + (31*cos(theta1)*cos(theta2)*(conj(x_c_z) - 9/100))/100))/2 - (((93*cos(conj(theta1))*sin(conj(theta2)))/4000 - (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_z - 9/100))/100)*((31*sin(theta2)*(conj(x_c_x) - (3*sin(theta1))/40))/100 - (31*cos(theta1)*cos(theta2)*(conj(x_c_z) - 9/100))/100))/2 - (((93*sin(conj(theta1))*sin(conj(theta2)))/4000 + (31*cos(conj(theta1))*cos(conj(theta2))*(x_c_z - 9/100))/100)*((31*sin(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 - (31*cos(theta2)*sin(theta1)*(conj(x_c_z) - 9/100))/100))/2 - (((31*cos(conj(theta1))*cos(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 - (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_x - (3*sin(conj(theta1)))/40))/100)*((31*cos(theta1)*cos(theta2)*(conj(x_c_x) - (3*sin(theta1))/40))/100 + (31*cos(theta2)*sin(theta1)*((3*cos(theta1))/40 + conj(x_c_y)))/100))/2 - (((31*cos(conj(theta1))*cos(conj(theta2))*(x_c_x - (3*sin(conj(theta1)))/40))/100 + (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_y + (3*cos(conj(theta1)))/40))/100)*((31*cos(theta1)*cos(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 - (31*cos(theta2)*sin(theta1)*(conj(x_c_x) - (3*sin(theta1))/40))/100))/2

    return d_theta1


def calc_d_theta2(theta1, theta2, x_c_x, x_c_y, x_c_z):
    d_theta2 = (((31*cos(conj(theta2))*(x_c_x - (3*sin(conj(theta1)))/40))/100 + (31*cos(conj(theta1))*sin(conj(theta2))*(x_c_z - 9/100))/100)*((31*sin(theta2)*(conj(x_c_x) - (3*sin(theta1))/40))/100 - (31*cos(theta1)*cos(theta2)*(conj(x_c_z) - 9/100))/100))/2 + (((31*sin(conj(theta2))*(x_c_x - (3*sin(conj(theta1)))/40))/100 - (31*cos(conj(theta1))*cos(conj(theta2))*(x_c_z - 9/100))/100)*((31*cos(theta2)*(conj(x_c_x) - (3*sin(theta1))/40))/100 + (31*cos(theta1)*sin(theta2)*(conj(x_c_z) - 9/100))/100))/2 + (((31*cos(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 + (31*sin(conj(theta1))*sin(conj(theta2))*(x_c_z - 9/100))/100)*((31*sin(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 - (31*cos(theta2)*sin(theta1)*(conj(x_c_z) - 9/100))/100))/2 + (((31*sin(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 - (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_z - 9/100))/100)*((31*cos(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 + (31*sin(theta1)*sin(theta2)*(conj(x_c_z) - 9/100))/100))/2 - (((31*cos(conj(theta1))*cos(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 - (31*cos(conj(theta2))*sin(conj(theta1))*(x_c_x - (3*sin(conj(theta1)))/40))/100)*((31*cos(theta1)*sin(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 - (31*sin(theta1)*sin(theta2)*(conj(x_c_x) - (3*sin(theta1))/40))/100))/2 - (((31*cos(conj(theta1))*sin(conj(theta2))*(x_c_y + (3*cos(conj(theta1)))/40))/100 - (31*sin(conj(theta1))*sin(conj(theta2))*(x_c_x - (3*sin(conj(theta1)))/40))/100)*((31*cos(theta1)*cos(theta2)*((3*cos(theta1))/40 + conj(x_c_y)))/100 - (31*cos(theta2)*sin(theta1)*(conj(x_c_x) - (3*sin(theta1))/40))/100))/2

    return d_theta2


def auto_pointing():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'auto_pointing' node so that multiple nodes can run
    # simultaneously.
    rospy.init_node('auto_pointing', anonymous=True)

    hebi_modules = [('arm', 'joint_0'), ('arm', 'joint_1')]

    theta_bounds = [[-1.5, 1.5], [-0.15, 0.6]]
    motion_bound = 0.05

    # discover HEBI modules
    count_found = 0
    lookup = hebi.Lookup()
    time.sleep(2) # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
        print('{0} | {1}'.format(entry.family, entry.name))
        if (entry.family, entry.name) in hebi_modules:
            count_found = count_found + 1

    if len(hebi_modules) != count_found:
        print('Some HEBI module(s) is missing (%d/%d)...' % (count_found, len(hebi_modules)))
        if not MOCK_TEST:
            exit()

    # create a HEBI control group from a set of names
    if not MOCK_TEST:
        hebi_group = lookup.get_group_from_names([m[0] for m in hebi_modules], [m[1] for m in hebi_modules])
        print('HEBI control group created (size=%d)' % (hebi_group.size))
        print(hebi_group)

        group_command = hebi.GroupCommand(hebi_group.size)

    # create TF listener
    tf_listener = tf.TransformListener()

    # initialize HEBI actuator position control parameters
    update_alpha = 0.5
    theta = np.array([0, 0])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # observe the current ring location
        if not MOCK_TEST:
            if not TWO_POINT_TEST:
                try:
                    (T_ak1_ring, R_ak1_ring) = tf_listener.lookupTransform('/ak2_claw', '/ak1_ring', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                x_c = [T_ak1_ring[0], T_ak1_ring[1], T_ak1_ring[2]]
            else:
                if time.time() % 20 > 10:
                    x_c = [2,2,1]
                else:
                    x_c = [2,-2,0.5]
        else:
            if time.time() % 20 > 10:
                x_c = [2,2,1]
            else:
                x_c = [2,-2,0.5]

        # loop for gradient descent
        for t_gd in range(20):
            # find gradient
            d_theta = np.array([calc_d_theta1(theta[0], theta[1], x_c[0], x_c[1], x_c[2]),
                                calc_d_theta2(theta[0], theta[1], x_c[0], x_c[1], x_c[2])])
            print(d_theta)
            # update desired theta
            for i_dim in range(len(d_theta)):
                d_theta[i_dim] = max(-motion_bound, min(motion_bound, d_theta[i_dim]))
            theta = theta - d_theta * update_alpha
            for i_dim in range(len(theta)):
                theta[i_dim] = max(theta_bounds[i_dim][0], min(theta_bounds[i_dim][1], theta[i_dim]))

        # send command to HEBI control group
        if not MOCK_TEST:
            group_command.position = theta
            hebi_group.send_command(group_command)
        print('theta1 = %.4f, theta2 = %.4f, x_c = (%.2f, %.2f, %.2f)' % (
            theta[0], theta[1], x_c[0], x_c[1], x_c[2]));

        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        auto_pointing()
    except rospy.ROSInterruptException:
        pass
