#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from motion_plan import my_Motion_plan

class my_imu(object):

    def __init__(self):
        super(my_imu,self).__init__()

    def rpy(self,data):
        ox = data.orientation.x
        oy = data.orientation.y
        oz = data.orientation.z
        ow = data.orientation.w
        o = [ox,oy,oz,ow]

        wx = data.angular_velocity.x
        wy = data.angular_velocity.y
        wz = data.angular_velocity.z
        a_v = [wx,wy,wz]

        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z
        l_a = [ax,ay,az]

        rpy_angle = [0, 0, 0]
        #rpy_angle=quat_to_angle(data.orientation)
        rpy_angle[0] = -math.atan2(2 * (ow * ox + oy * oz), 1 - 2 * (ox ** 2 + oy ** 2))
        rpy_angle[1] = -math.asin(2 * (ow * oy - oz * ox))
        rpy_angle[2] = -math.atan2(2 * (ow * oz + ox * oy), 1 - 2 * (oy ** 2 + oz ** 2))
        #rpy_angle = quat_to_angle(data.orientation)
        return o,a_v,l_a,rpy_angle

    def self_balance(self,data):
        Motion_plan = my_Motion_plan()
        old_angle = rospy.get_param("hyq_angle")
        h = rospy.get_param("hyq_h")
        auto = rospy.get_param('hyq_auto')
        if auto:
            o,a_v,l_a,angle = self.rpy(data)
            if data.header.seq %100 ==0:
                if math.fabs(old_angle[0]-angle[0])>0.01 or math.fabs(old_angle[0]-angle[0])>0.01 or math.fabs(old_angle[0]-angle[0])>0.01:
                    Motion_plan.init_balance(angle, h)
                    rospy.set_param('hyq_angle', angle)




