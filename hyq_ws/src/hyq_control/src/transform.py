#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion,Twist,Vector3
from gazebo_msgs.srv import GetModelState,GetModelStateRequest
import tf

class my_transform(object):

    def __init__(self):
        super(my_transform,self).__init__()

    def odom_broadcast(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model = GetModelStateRequest()
        model.model_name = "hyq"
        result = get_model_srv(model)
        odom = Odometry()
        header = Header()
        header.frame_id = 'base_link'
        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        br = tf.TransformBroadcaster()
        br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                         (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                          odom.pose.pose.orientation.w),
                         rospy.Time.now(),
                         'base_link', 'map')