#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class HyqState_sub(object):

    def __init__(self):
        super(HyqState_sub, self).__init__()

        rospy.Subscriber("/ground_truth", Odometry, self.p3d_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/hyq/lf_foot_bumper", ContactsState, self.contact_callback)
        rospy.Subscriber("/hyq/lh_foot_bumper", ContactsState, self.contact_callback)
        rospy.Subscriber("/hyq/rf_foot_bumper", ContactsState, self.contact_callback)
        rospy.Subscriber("/hyq/rh_foot_bumper", ContactsState, self.contact_callback)

        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/hyq/joint_states", JointState, self.joints_state_callback)
