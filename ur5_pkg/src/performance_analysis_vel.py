#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

global x_dot
x_dot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def X_dot_callback(X_dot_msg):
    global x_dot
    x_dot[0] = X_dot_msg.linear.x
    x_dot[1] = X_dot_msg.linear.y
    x_dot[2] = X_dot_msg.linear.z
    x_dot[3] = X_dot_msg.angular.x
    x_dot[4] = X_dot_msg.angular.y
    x_dot[5] = X_dot_msg.angular.z
    print("callback 1 -----", x_dot)

def main():
    rospy.init_node('performance_analysis', anonymous=False)
    rospy.Subscriber('/X_dot', Twist, X_dot_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

