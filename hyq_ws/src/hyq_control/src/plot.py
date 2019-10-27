#! /usr/bin/env python
import kinematic
import hyq_constants
import matplotlib.pyplot as plt

def draw_2D(points):
    x=[]
    y=[]
    z=[]
    for item in points:
        x.append(item[0])
        y.append(item[1])
        z.append(item[2])
    #plt.plot(x,z)
    plt.scatter(x,y)
    plt.show()

def print_scale_xy():
    range_num =100
    joint_haa = hyq_constants.lf_haa_bound
    joint_hfe = hyq_constants.lf_hfe_bound
    joint_kfe = hyq_constants.lf_kfe_bound
    poss =[]
    for i in range(range_num):
        for j in range(range_num):
            for k in range(range_num):
                joint_1 = (joint_haa[1] - joint_haa[0]) / range_num * i + joint_haa[0]
                joint_2 = (joint_hfe[1] - joint_hfe[0]) / range_num * j + joint_hfe[0]
                joint_3 = (joint_kfe[1] - joint_kfe[0]) / range_num * k + joint_kfe[0]
                #print joint_1,joint_2,joint_3
                pos = kinematic.forward_kinematics_lf(joint_1, joint_2, joint_3)
                #print pos
                ##
                z=-0.4
                ##
                if pos[2] >z-0.01 and pos[2] < z+0.01:
                    poss.append(pos)
    draw_2D(poss)

def print_four_foot(point1,point2,point3,point4):
    time =[]
    for i in range(len(point1)):
        time.append(i)

    plt.subplot(221)
    plt.plot(time, point1)
    plt.subplot(222)
    plt.plot(time, point2)
    plt.subplot(223)
    plt.plot(time, point3)
    plt.subplot(224)
    plt.plot(time, point4)

    plt.show()

