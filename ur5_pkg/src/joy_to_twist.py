#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

global des_twist
des_twist = Twist()
des_twist.linear.x  = 0
des_twist.linear.y  = 0
des_twist.linear.z  = 0
des_twist.angular.x = 0
des_twist.angular.y = 0
des_twist.angular.z = 0

pub = rospy.Publisher('/X_dot', Twist, queue_size=100)

def joy_interpret(axes,buttons):
    trig_button = buttons[0]
    #print(trig_button, type(trig_button))

    ctrl_type = trig_button # ang_ctrl:0, pos_ctrl:1

    vx = 0
    vy = 0
    vz = 0
    wx = 0
    wy = 0
    wz = 0

    if(ctrl_type==0):
        # ang_ctrl
        k1 = 0.3
        vx = 0
        vy = 0
        vz = 0
        wx = axes[0]*k1
        wy = -axes[1]*k1
        wz = axes[2]*k1

    else: #(ctrl_type==1)
        #pos_ctrl
        k2 = 0.05
        vx = -axes[1]*k2
        vy = -axes[0]*k2
        vz = axes[5]*k2/2.0
        wx = 0
        wy = 0
        wz = 0

    return vx, vy, vz, wx, wy, wz

def joy_callback(msg):
    axes = msg.axes
    buttons = msg.buttons
    #print(axes, buttons)

    vx,vy,vz,wx,wy,wz = joy_interpret(axes,buttons)
    #print(vx,vy,vz,wx,wy,wz)

    global des_twist
    des_twist.linear.x  = vx
    des_twist.linear.y  = vy
    des_twist.linear.z  = vz
    des_twist.angular.x = wx
    des_twist.angular.y = wy
    des_twist.angular.z = wz
    print(des_twist)
    print("---")
    pub.publish(des_twist)

def main():
    rospy.init_node('joy_to_des_twist', anonymous=False)
    rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
