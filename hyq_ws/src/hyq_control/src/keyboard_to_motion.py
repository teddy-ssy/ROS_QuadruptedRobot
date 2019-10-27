#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import String,Float32
from motion_plan import my_Motion_plan

h=0
angle = [0.0,0.0,0.0]

class my_keyboard_to_motion(object):

    def __init__(self):
        super(my_keyboard_to_motion,self).__init__()

    def keys_cb(self,msg):
        if len(msg.data) == 0:
            return
        print msg.data[0]
        self.motion_commander(msg.data[0])


    def motion_commander(self,vels):
        angle = rospy.get_param('hyq_angle')
        rospy.set_param('hyq_auto',False)
        rate = rospy.Rate(50)

        motion_plan = my_Motion_plan()
        h = rospy.get_param('hyq_h')
        if vels == 'w':
            point = [1,0]
            motion_plan.walk_balance(point,angle,h)
        elif vels == 's':
            point = [-1, 0]
            motion_plan.walk_balance(point, angle, h)
        elif vels == 'a':
            point = [0, -1]
            motion_plan.walk_balance(point, angle, h)
        elif vels == 'd':
            point = [0, 1]
            motion_plan.walk_balance(point, angle, h)
        elif vels == 'q':
            degree = -math.pi/30
            motion_plan.rotation_balance(degree,angle,h)
        elif vels == 'e':
            degree = math.pi / 30
            motion_plan.rotation_balance(degree, angle, h)
        elif vels == 'i':
            angle[1] -=math.pi/30
            motion_plan.init_balance(angle,h)
            rate.sleep()
        elif vels == 'k':
            angle[1] += math.pi / 30
            motion_plan.init_balance(angle,h)
            rate.sleep()
        elif vels == 'j':
            angle[0] += math.pi / 30
            motion_plan.init_balance(angle,h)
            rate.sleep()
        elif vels == 'l':
            angle[0] -= math.pi / 30
            motion_plan.init_balance(angle,h)
            rate.sleep()
        elif vels == 'u':
            h += 0.1
            motion_plan.init_balance(angle,h)
        elif vels == 'o':
            h -= 0.1
            motion_plan.init_balance(angle,h)
        rospy.set_param('hyq_h',h)
        rospy.set_param('hyq_ange',angle)
        rospy.set_param('hyq_auto', True)


if __name__ == '__main__':
    rospy.init_node('keyboard_to_motion')
