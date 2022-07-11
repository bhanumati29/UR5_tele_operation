#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped

global f_t
f_t = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def callback(msg):
    global f_t
    f_t[0] = msg.wrench.force.x
    f_t[1] = msg.wrench.force.y
    f_t[2] = msg.wrench.force.z
    f_t[3] = msg.wrench.torque.x
    f_t[4] = msg.wrench.torque.y
    f_t[5] = msg.wrench.torque.z

    #print(f_t)
    
def main():
    rospy.init_node('sub_wrench', anonymous=True)
    rospy.Subscriber('wrench', WrenchStamped, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

