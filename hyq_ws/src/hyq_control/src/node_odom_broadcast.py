#! /usr/bin/env python

import rospy
from transform import my_transform

if __name__=="__main__":
    rospy.init_node("odom_tf_broadcast")
    rate = rospy.Rate(50)
    tran = my_transform()
    while not rospy.is_shutdown():
        tran.odom_broadcast()