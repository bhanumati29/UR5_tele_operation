#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def callback(js):
    print("Position::",  js.position[0])
    
def subscribers():
    rospy.init_node('subjs', anonymous=True)
    rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        subscribers()
    except rospy.ROSInterruptException:
        pass

