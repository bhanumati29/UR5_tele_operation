#!/usr/bin/env python

import rospy
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped

global omni_feedback_msg 
omni_feedback_msg = OmniFeedback()

pub =rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=10 )

def force_feedback(h_x_, h_y_, h_z_, fx, fy, fz):
    omni_feedback_msg.force.x = fx
    omni_feedback_msg.force.y = fy
    omni_feedback_msg.force.z = fz
    omni_feedback_msg.position.x = h_x_
    omni_feedback_msg.position.y = h_y_
    omni_feedback_msg.position.z = h_z_

    pub.publish(omni_feedback_msg)

def hp_callback(msg2): 
    h_x = msg2.pose.position.x
    h_y = msg2.pose.position.y
    h_z = msg2.pose.position.z
    fx = 0.0
    fy = 0.0
    fz = 0.0

    force_feedback(h_x, h_y, h_z, fx, fy, fz)

def main():

    rospy.init_node('force_feedback_publisher', anonymous=True)
    rospy.Subscriber('/phantom/pose', PoseStamped, hp_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    