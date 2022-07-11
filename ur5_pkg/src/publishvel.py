#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
global msg
global vel

def pubvel():
    global msg
    global vel
    rospy.init_node('publish', anonymous=True)
    pub=rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=100)
    r=rospy.Rate(100)
    i=0
    vel=[0.02,0.03,0.03,0.0,0.05,0.1]
    msg=JointTrajectory()
    while not rospy.is_shutdown():
        vel[5]=-0.3
        msg.points=[JointTrajectoryPoint(velocities=vel)]
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__=='__main__':
    try:
        pubvel()
    except rospy.ROSInterruptException:
        pass
