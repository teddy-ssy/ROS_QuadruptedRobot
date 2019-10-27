#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import hyq_constants

class JointPub(object):

    def __init__(self):
        super(JointPub, self).__init__()

        self.publishers_array = []
        self.lf_haa_joint_pub = rospy.Publisher("/hyq/lf_haa_joint_position_controller/command", Float64, queue_size=1)
        self.lf_hfe_joint_pub = rospy.Publisher("/hyq/lf_hfe_joint_position_controller/command", Float64, queue_size=1)
        self.lf_kfe_joint_pub = rospy.Publisher("/hyq/lf_kfe_joint_position_controller/command", Float64, queue_size=1)

        self.lh_haa_joint_pub = rospy.Publisher("/hyq/lh_haa_joint_position_controller/command", Float64, queue_size=1)
        self.lh_hfe_joint_pub = rospy.Publisher("/hyq/lh_hfe_joint_position_controller/command", Float64, queue_size=1)
        self.lh_kfe_joint_pub = rospy.Publisher("/hyq/lh_kfe_joint_position_controller/command", Float64, queue_size=1)

        self.rf_haa_joint_pub = rospy.Publisher("/hyq/rf_haa_joint_position_controller/command", Float64, queue_size=1)
        self.rf_hfe_joint_pub = rospy.Publisher("/hyq/rf_hfe_joint_position_controller/command", Float64, queue_size=1)
        self.rf_kfe_joint_pub = rospy.Publisher("/hyq/rf_kfe_joint_position_controller/command", Float64, queue_size=1)

        self.rh_haa_joint_pub = rospy.Publisher("/hyq/rh_haa_joint_position_controller/command", Float64, queue_size=1)
        self.rh_hfe_joint_pub = rospy.Publisher("/hyq/rh_hfe_joint_position_controller/command", Float64, queue_size=1)
        self.rh_kfe_joint_pub = rospy.Publisher("/hyq/rh_kfe_joint_position_controller/command", Float64, queue_size=1)

        self.publishers_array.append(self.lf_haa_joint_pub)
        self.publishers_array.append(self.lf_hfe_joint_pub)
        self.publishers_array.append(self.lf_kfe_joint_pub)

        self.publishers_array.append(self.lh_haa_joint_pub)
        self.publishers_array.append(self.lh_hfe_joint_pub)
        self.publishers_array.append(self.lh_kfe_joint_pub)

        self.publishers_array.append(self.rf_haa_joint_pub)
        self.publishers_array.append(self.rf_hfe_joint_pub)
        self.publishers_array.append(self.rf_kfe_joint_pub)

        self.publishers_array.append(self.rh_haa_joint_pub)
        self.publishers_array.append(self.rh_hfe_joint_pub)
        self.publishers_array.append(self.rh_kfe_joint_pub)

    def move_joint(self, joints_array,speed):
        i = 0
        for publisher_object in self.publishers_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            #rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1
        if speed !=0:
            rate = rospy.Rate(1/speed)
            rate.sleep()

    def start_loop(self):
        rospy.loginfo('start_loop')
        pos1 = [hyq_constants.test_joint_lf[0], hyq_constants.test_joint_lf[1], hyq_constants.test_joint_lf[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pos2 = [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0]
        position = "pos1"
        while not rospy.is_shutdown():
            if position == "pos1":
                self.move_joint(pos1)
                position = "pos2"
            else:
                self.move_joint(pos2)
                position = "pos1"
            rospy.sleep(0.1)

    def move_one_step(self, joints,rate):
        rospy.loginfo('move_one_step')
        num=0
        while num<5:
            self.move_joint(joints)
            num+=1
            rospy.sleep(rate)


