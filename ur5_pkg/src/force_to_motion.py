#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

global t_v
t_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def t_vel_callback(msg):
    global t_v
    t_v[0] = msg.twist.linear.x
    t_v[1] = msg.twist.linear.y
    t_v[2] = msg.twist.linear.z
    t_v[3] = msg.twist.angular.x
    t_v[4] = msg.twist.angular.y
    t_v[5] = msg.twist.angular.z

    print(t_v)

def X_callback(msg):
    

    
def main():
    rospy.init_node('performance_eval', anonymous=True)
    rospy.Subscriber('/tool_velocity', TwistStamped, t_vel_callback)
    rospy.Subscriber('/X_dot', Twist , X_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

