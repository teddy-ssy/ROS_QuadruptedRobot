#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64

"""
position is [-1.57, 1.57, -1.57, -1.57, 1.57, -1.57, -1.57, -1.57, 1.57, -1.57, -1.57, 1.57]

all haa             [-1.57, 1.57]

lf_hfe and rf_hfe   [0, 1.57]

lf_kfe and rf_kfe   [-1.57, 0]

lh_hfe and rh_hfe   [-1.57, 0]

rh_kfe and rh_kfe   [0, 1.57]  
"""


class JointPub(object):
    def __init__(self):

        self.publishers_array = []

        self._lf_haa_joint_pub = rospy.Publisher('/hyq/lf_haa_joint_position_controller/command', Float64, queue_size=1)
        self._lf_hfe_joint_pub = rospy.Publisher('/hyq/lf_hfe_joint_position_controller/command', Float64, queue_size=1)
        self._lf_kfe_joint_pub = rospy.Publisher('/hyq/lf_kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._lf_haa_joint_pub)
        self.publishers_array.append(self._lf_hfe_joint_pub)
        self.publishers_array.append(self._lf_kfe_joint_pub)

        self._rf_haa_joint_pub = rospy.Publisher('/hyq/rf_haa_joint_position_controller/command', Float64, queue_size=1)
        self._rf_hfe_joint_pub = rospy.Publisher('/hyq/rf_hfe_joint_position_controller/command', Float64, queue_size=1)
        self._rf_kfe_joint_pub = rospy.Publisher('/hyq/rf_kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._rf_haa_joint_pub)
        self.publishers_array.append(self._rf_hfe_joint_pub)
        self.publishers_array.append(self._rf_kfe_joint_pub)

        self._lh_haa_joint_pub = rospy.Publisher('/hyq/lh_haa_joint_position_controller/command', Float64, queue_size=1)
        self._lh_hfe_joint_pub = rospy.Publisher('/hyq/lh_hfe_joint_position_controller/command', Float64, queue_size=1)
        self._lh_kfe_joint_pub = rospy.Publisher('/hyq/lh_kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._lh_haa_joint_pub)
        self.publishers_array.append(self._lh_hfe_joint_pub)
        self.publishers_array.append(self._lh_kfe_joint_pub)

        self._rh_haa_joint_pub = rospy.Publisher('/hyq/rh_haa_joint_position_controller/command', Float64, queue_size=1)
        self._rh_hfe_joint_pub = rospy.Publisher('/hyq/rh_hfe_joint_position_controller/command', Float64, queue_size=1)
        self._rh_kfe_joint_pub = rospy.Publisher('/hyq/rh_kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._rh_haa_joint_pub)
        self.publishers_array.append(self._rh_hfe_joint_pub)
        self.publishers_array.append(self._rh_kfe_joint_pub)

        '''
        self.init_pos =[0.0, 1.57, -1.57,
                        0.0, 1.57, -1.57, 
                        0.0, -1.57, 1.57, 
                        0.0, -1.57, 1.57]
        '''
        self.init_pos =[0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0]

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._lf_haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lf_haa_joint_pub Publisher Connected")

        while (self._lf_hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lf_hfe_joint_pub Publisher Connected")

        while (self._lf_kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lf_kfe_joint_pub Publisher Connected")

        while (self._rf_haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rf_haa_joint_pub Publisher Connected")

        while (self._rf_hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rf_hfe_joint_pub Publisher Connected")

        while (self._rf_kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rf_kfe_joint_pub Publisher Connected")

        while (self._lh_haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lh_haa_joint_pub Publisher Connected")

        while (self._lh_hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lh_hfe_joint_pub Publisher Connected")

        while (self._lh_kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_lh_kfe_joint_pub Publisher Connected")

        while (self._rh_haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rh_haa_joint_pub Publisher Connected")

        while (self._rh_hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rh_hfe_joint_pub Publisher Connected")

        while (self._rh_kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_rh_kfe_joint_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")
    
    def joint_hyq_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            #rospy.loginfo(str(joint_value))
            rospy.logdebug("JointsPos>>" + str(joint_value))
            publisher_object.publish(joint_value)
            i += 1
        #rospy.spin()

    def start_loop(self, rate_value=2.0):
        rospy.loginfo("Start Loop")
        pos6 = [0.0, 1.6, -1.6,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 1.6, -1.6]
        pos7 = [0.0, 0.0, 0.0, 
                0.0, 1.6, -1.6, 
                0.0, 1.6, 0.0, 
                0.0, 0.0, -1.6]
        position = "pos0"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            if position == "pos6":
                self.move_joints(pos6)
                position = "pos7"
            else:
                self.move_joints(pos7)
                position = "pos6"
            rate.sleep()

    



if __name__ == "__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 1.0
    joint_publisher.start_loop(rate_value)
    #joint_publisher.test_jump()