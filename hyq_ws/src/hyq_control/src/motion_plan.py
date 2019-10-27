#! /usr/bin/env python

import math
import rospy
import hyq_constants
from joint_publisher import JointPub
from kinematic import my_Kinematic
from std_srvs.srv import Empty

class my_Motion_plan(object):

    def __init__(self):
        super(my_Motion_plan, self).__init__()
        self.my_Kinematic = my_Kinematic()

    def reset_gazebo(self):
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        reset_simulation()

    def move_path(self,positions,speed):
        joints =[]
        for position in positions:
            joint_lf= self.my_Kinematic.inverse_kinematics_lf(position[0], position[1], position[2])
            joint_lh = self.my_Kinematic.inverse_kinematics_lh(position[3], position[4], position[5])
            joint_rf = self.my_Kinematic.inverse_kinematics_rf(position[6], position[7], position[8])
            joint_rh = self.my_Kinematic.inverse_kinematics_rh(position[9], position[10], position[11])
            joint=[]
            for item in joint_lf:
                joint.append(item)
            for item in joint_lh:
                joint.append(item)
            for item in joint_rf:
                joint.append(item)
            for item in joint_rh:
                joint.append(item)
            joints.append(joint)
        joint_publisher = JointPub()
        for joint in joints:
            joint_publisher.move_joint(
                [joint[0],joint[1],joint[2],
                 joint[3], joint[4], joint[5],
                 joint[6], joint[7], joint[8],
                 joint[9], joint[10], joint[11],
             ],speed)
        return joints

    def translation_motion(self,h,alpha,step,rally,num):
        body_x = 0
        body_y = 0
        body_z = 0
        #points = []
        if num <=rally:
            x = body_x - ((step*num/rally) * math.cos(alpha))
            y = body_y + ((step*num/rally) * math.sin(alpha))
            z = body_z + 0
            #points.append([x,y,z])
        elif num <= 2*rally:
            num = num - rally
            x = body_x - step*math.cos(alpha) + ((step*num/rally) * math.cos(alpha))
            y = body_y + step*math.sin(alpha) - ((step*num/rally) * math.sin(alpha))
            z = body_z + h*num/rally
            #points.append([x, y, z])
        elif num <= 3*rally:
            num = num - (2*rally)
            x = body_x + ((step*num/rally) * math.cos(alpha))
            y = body_y - ((step*num/rally) * math.sin(alpha))
            z = body_z + h - (h*num/rally)
            #points.append([x, y, z])
        elif num <= 4*rally:
            num = num - (3*rally)
            x = body_x + step*math.cos(alpha) - ((step*num/rally) * math.cos(alpha))
            y = body_y - step*math.sin(alpha) + ((step*num/rally) * math.sin(alpha))
            z = body_z + 0
            #points.append([x, y, z])
        return x,y,z

    def tort_forward(self,init_lf,init_lh,init_rf,init_rh,h,alpha,step,rally,num):
        if num <=2 * rally:
            x, y, z = self.translation_motion(h, alpha, step, rally, num)
            lf_x = init_lf[0] + x
            lf_y = init_lf[1] + y
            lf_z = init_lf[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num+(2*rally))
            lh_x = init_lh[0] + x
            lh_y = init_lh[1] + y
            lh_z = init_lh[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num + (2 * rally))
            rf_x = init_rf[0] + x
            rf_y = init_rf[1] + y
            rf_z = init_rf[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num)
            rh_x = init_rh[0] + x
            rh_y = init_rh[1] + y
            rh_z = init_rh[2] + z
            return [lf_x, lf_y, lf_z, lh_x, lh_y, lh_z, rf_x, rf_y, rf_z, rh_x, rh_y, rh_z]
        elif num <= 4 * rally:
            x, y, z = self.translation_motion(h, alpha, step, rally, num)
            lf_x = init_lf[0] + x
            lf_y = init_lf[1] + y
            lf_z = init_lf[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num - (2 * rally))
            lh_x = init_lh[0] + x
            lh_y = init_lh[1] + y
            lh_z = init_lh[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num - (2 * rally))
            rf_x = init_rf[0] + x
            rf_y = init_rf[1] + y
            rf_z = init_rf[2] + z

            x, y, z = self.translation_motion(h, alpha, step, rally, num)
            rh_x = init_rh[0] + x
            rh_y = init_rh[1] + y
            rh_z = init_rh[2] + z
            return [lf_x, lf_y, lf_z, lh_x, lh_y, lh_z, rf_x, rf_y, rf_z, rh_x, rh_y, rh_z]

    def rotation_motion(self,h,alpha,rally,num):
        body_x = 0
        body_y = 0
        body_z = 0
        step = math.sqrt((hyq_constants.lf_init_position[0] * hyq_constants.lf_init_position[0]) + (
                hyq_constants.lf_init_position[1] * hyq_constants.lf_init_position[1]))
        base_alpha = math.atan(hyq_constants.lf_init_position[1] / hyq_constants.lf_init_position[0])
        if num <= rally:
            x = body_x + (step * math.cos(alpha* num / rally + base_alpha)) - hyq_constants.lf_init_position[0]
            y = body_y + (step * math.sin(alpha* num / rally + base_alpha)) - hyq_constants.lf_init_position[1]
            z = body_z + 0
            # points.append([x,y,z])
        elif num <= 2 * rally:
            num = num - rally
            x = body_x + (step * math.cos((alpha - (alpha * num / rally)) + base_alpha)) - hyq_constants.lf_init_position[0]
            y = body_y + (step * math.sin((alpha - (alpha * num / rally)) + base_alpha)) - hyq_constants.lf_init_position[1]
            z = body_z + h * num / rally
            # points.append([x, y, z])
        elif num <= 3 * rally:
            num = num - (2 * rally)
            x = body_x + (step * math.cos( base_alpha - (alpha * num / rally) )) - hyq_constants.lf_init_position[0]
            y = body_y + (step * math.sin( base_alpha - (alpha * num / rally) )) - hyq_constants.lf_init_position[1]
            z = body_z + h - (h * num / rally)
            # points.append([x, y, z])
        elif num <= 4 * rally:
            num = num - (3 * rally)
            x = body_x + (step * math.cos(base_alpha - (alpha - (alpha * num / rally)))) - hyq_constants.lf_init_position[0]
            y = body_y + (step * math.sin(base_alpha - (alpha - (alpha * num / rally)))) - hyq_constants.lf_init_position[1]
            z = body_z + 0
            # points.append([x, y, z])
        return x, y, z

    def tort_rotation(self,init_lf,init_lh,init_rf,init_rh,h,alpha,rally,num):
        if num <=2 * rally:
            x, y, z = self.rotation_motion(h,alpha,rally,num)
            lf_x = init_lf[0] + x
            lf_y = init_lf[1] + y
            lf_z = init_lf[2] + z

            x, y, z = self.rotation_motion(h,alpha,rally,num+(2*rally))
            lh_x = init_lh[0] + x
            lh_y = init_lh[1] + y
            lh_z = init_lh[2] + z

            x, y, z = self.rotation_motion(h, -alpha, rally, num + (2 * rally))
            rf_x = init_rf[0] + x
            rf_y = init_rf[1] + y
            rf_z = init_rf[2] + z

            x, y, z = self.rotation_motion(h, -alpha, rally, num)
            rh_x = init_rh[0] + x
            rh_y = init_rh[1] + y
            rh_z = init_rh[2] + z
            return [lf_x, lf_y, lf_z, lh_x, lh_y, lh_z, rf_x, rf_y, rf_z, rh_x, rh_y, rh_z]
        elif num <= 4 * rally:
            x, y, z = self.rotation_motion(h, alpha, rally, num)
            lf_x = init_lf[0] + x
            lf_y = init_lf[1] + y
            lf_z = init_lf[2] + z

            x, y, z = self.rotation_motion(h, alpha, rally, num - (2 * rally))
            lh_x = init_lh[0] + x
            lh_y = init_lh[1] + y
            lh_z = init_lh[2] + z

            x, y, z = self.rotation_motion(h, -alpha, rally, num - (2 * rally))
            rf_x = init_rf[0] + x
            rf_y = init_rf[1] + y
            rf_z = init_rf[2] + z

            x, y, z = self.rotation_motion(h, -alpha, rally, num )
            rh_x = init_rh[0] + x
            rh_y = init_rh[1] + y
            rh_z = init_rh[2] + z
            return [lf_x, lf_y, lf_z, lh_x, lh_y, lh_z, rf_x, rf_y, rf_z, rh_x, rh_y, rh_z]

    def init_balance(self,angle,h):

        lf_init_position, lh_init_position, rf_init_position, rh_init_position = self.my_Kinematic.calculate_shoulder(angle, h)
        position = []
        for item in lf_init_position:
            position.append(item)
        for item in lh_init_position:
            position.append(item)
        for item in rf_init_position:
            position.append(item)
        for item in rh_init_position:
            position.append(item)
        joints = self.move_path([position],0)
        return joints

    def walk_balance(self,point,angle,h):
        init_lf, init_lh, init_rf, init_rh = self.my_Kinematic.calculate_shoulder(angle, h)
        step = 0.03
        rally = 10
        rally_time = 0.05
        high = 0.1
        if point[0] ==0:
            point[0] = 1.0e-10
        degree = math.atan(point[1]/point[0])
        if point[0]<0:
            step = -0.03

        positions_forward = []
        for i in range(4 * rally):
            position_forward = self.tort_forward(init_lf, init_lh, init_rf, init_rh,
                                    high, degree, step, rally, i)
            positions_forward.append(position_forward)
        joint = self.move_path(positions_forward, rally_time / rally)
        self.init_balance(angle,h)

    def rotation_balance(self,degree,angle,h):
        init_lf, init_lh, init_rf, init_rh = self.my_Kinematic.calculate_shoulder(angle, h)
        rally = 10
        rally_time = 0.05
        high = 0.1
        alpha = math.pi/30
        if degree<0:
            alpha = -alpha

        positions_rotation = []
        for i in range(4 * rally):
            position_rotation = self.tort_rotation(init_lf, init_lh, init_rf, init_rh,
                                     high, alpha, rally, i)
            positions_rotation.append(position_rotation)
        joint = self.move_path(positions_rotation, rally_time / rally)

        self.init_balance(angle,h)


