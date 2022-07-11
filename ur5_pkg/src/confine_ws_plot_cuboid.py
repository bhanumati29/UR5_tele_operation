#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randint
import numpy as np
import time
import math
import random
from itertools import count
import time
from mpl_toolkits import mplot3d

global x_dot
x_dot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global tool_pos_linear
tool_pos_linear = [0.0, 0.0, 0.0]

global tool_pos_angular
tool_pos_angular = [0.0, 0.0, 0.0, 0.0]

global alpha
alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def X_dot_callback(X_dot_msg):
    global x_dot
    x_dot[0] = X_dot_msg.linear.x
    x_dot[1] = X_dot_msg.linear.y
    x_dot[2] = X_dot_msg.linear.z
    x_dot[3] = X_dot_msg.angular.x
    x_dot[4] = X_dot_msg.angular.y
    x_dot[5] = X_dot_msg.angular.z
    #print("callback 1 -----")

global Cx, Cy, Cz, R
Cx = 0.10634792505818291
Cy = -0.5287466779966042
Cz = 0.5652483044300174
R  = 0.20

def quat2eul(q0,q1,q2,q3):
    phi = math.atan2(2*(q0*q1 + q2*q3),1-2*((q1)**2+(q2)**2))
    th  = math.asin(2*(q0*q2 - q3*q1))
    psi = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))
    return phi, th, psi

def tf_callback(msg):
    temp_list = msg.transforms
    #print(len(temp_list))
    
    if(len(temp_list)==1):
        msg_n = temp_list[0]
        #print('---')
        #print(msg_n)
        global tool_pos_linear
        global tool_pos_angular
        tool_pos_linear[0]  = msg_n.transform.translation.x
        tool_pos_linear[1]  = msg_n.transform.translation.y
        tool_pos_linear[2]  = msg_n.transform.translation.z
        tool_pos_angular[0] = msg_n.transform.rotation.w
        tool_pos_angular[1] = msg_n.transform.rotation.x
        tool_pos_angular[2] = msg_n.transform.rotation.y
        tool_pos_angular[3] = msg_n.transform.rotation.z

        global alpha
        for i in range(0,3):
            alpha[i] = tool_pos_linear[i]
        
        q0_ = tool_pos_angular[0]
        q1_ = tool_pos_angular[1]
        q2_ = tool_pos_angular[2]
        q3_ = tool_pos_angular[3]

        phi, th, psi = quat2eul(q0_,q1_,q2_,q3_)
        
        alpha[3] = phi
        alpha[4] = th
        alpha[5] = psi

        #print(alpha)

global display_val
display_val = 0

def display_plot(Cx, Cy, Cz, R, Px, Py, Pz, vx_0, vy_0, vz_0, vx, vy, vz):
    print(time.time())
    plt.plot(Px*100.0,Py*100.0,'ro')
    plt.draw()
    plt.pause(1e-17)
  
def tv_callback(tv_msg):
    global Cx, Cy, Cz, R
    global tool_pos_linear

    Px = tool_pos_linear[0]
    Py = tool_pos_linear[1]
    Pz = tool_pos_linear[2]

    ##############################

    global x_dot
    vx_0 = x_dot[0]
    vy_0 = x_dot[1]
    vz_0 = x_dot[2]

    vx = tv_msg.twist.linear.x
    vy = tv_msg.twist.linear.y
    vz = tv_msg.twist.linear.z

    global display_val
    display_val += 1
    if(display_val==5):
        global display_val
        display_val = 0

    if(display_val==1):
        display_plot(Cx, Cy, Cz, R, Px, Py, Pz, vx_0, vy_0, vz_0, vx, vy, vz)

    # print('---')
    # print(tv_msg)

def main():
    rospy.init_node('confine_ws_plot_sphere', anonymous=False)
    rospy.Subscriber('/tool_velocity', TwistStamped, tv_callback)
    rospy.Subscriber('/X_dot', Twist, X_dot_callback)
    rospy.Subscriber('/tf', TFMessage , tf_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

