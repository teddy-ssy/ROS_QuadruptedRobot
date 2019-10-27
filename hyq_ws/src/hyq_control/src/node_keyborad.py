#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import sys, select, tty, termios
from keyboard_to_motion import my_keyboard_to_motion

if __name__== "__main__":
    rospy.init_node("keyboard_driver")
    my_keyboard_to_motion = my_keyboard_to_motion()
    key_pub = rospy.Publisher('keys',String,queue_size=1)
    hyq_forward_pub = rospy.Publisher('hyq_forward', String, queue_size=1)
    rospy.Subscriber('keys', String, my_keyboard_to_motion.keys_cb)
    rate = rospy.Rate(200)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print "press Ctrl-C to exit"
    while not rospy.is_shutdown():
        if select.select([sys.stdin],[],[],0)[0] ==[sys.stdin]:
            print sys.stdin.read(1)
            key_pub.publish(sys.stdin.read(1))
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_attr)