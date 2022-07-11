#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
global msg
global vel
global js

def callbackjs(js):
    global vel
    global msg
    print("Vel_J5=", js.velocity[5])
    pub=rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
    r=rospy.Rate(100)
    vel=[0.0,0.0,0.0,0.0,0.0,0.0]
    msg=JointTrajectory()
    if (js.position[5]<0):
        vel[5]=0.1
        msg.points=[JointTrajectoryPoint(velocities=vel)]
        rospy.loginfo(msg)
        pub.publish(msg)
            
    
def subscribers():
    rospy.init_node('subandpub', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, callbackjs)
    rospy.spin()

if __name__=='__main__':
    try:
        subscribers()
    except rospy.ROSInterruptException:
        pass

