#! /urs/bin/env python

import rospy
from sensor_msgs.msg import Imu
from imu import my_imu

if __name__=='__main__':
    my_imu = my_imu()
    rospy.init_node("imu_listener", anonymous=True)
    rospy.Subscriber("/hyq/imu/data",Imu,my_imu.self_balance(),queue_size=1)
    rospy.spin()