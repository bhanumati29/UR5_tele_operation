#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

global cur_joint_pos
global cur_joint_vel
global cur_joint_acc
global cur_joint_eff

cur_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cur_joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cur_joint_acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cur_joint_eff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global des_joint_pos
global des_joint_vel
global des_joint_acc
global des_joint_eff

#des_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
des_joint_pos = [2.3968450477696024e-05, -5.4661427633106996e-05, -1.5707700888263147, -0.7854984442340296, -1.57084829012026, 1.5708271265029907] # this is the home location
des_joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
des_joint_acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
des_joint_eff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global joint_traj_msg
joint_traj_msg = JointTrajectory()

def limit(x,x_min,x_max):
    if(x<x_min):
        return x_min
    elif(x>x_max):
        return x_max
    else:
        return x

def control_angle(th0,th):
    e = [0,0,0,0,0,0]
    kp = [0.15,0.15,0.15,0.15,0.15,0.15]
    w = [0,0,0,0,0,0]

    for i in range(0,6):
        e[i] = th0[i] - th[i]
        w[i] = kp[i]*e[i]

    return w

pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)

def js_callback(joint_states):
    # save current joint states
    print("subscriber running")
    global cur_joint_pos
    global cur_joint_vel
    global cur_joint_acc
    global cur_joint_eff

    cur_joint_pos = joint_states.position
    cur_joint_vel = joint_states.velocity
    cur_joint_acc = [0.0,0.0,0.0,0.0,0.0,0.0]
    cur_joint_eff = joint_states.effort
    #print(cur_joint_eff)

    # calculate desired joint states
    global des_joint_pos
    global des_joint_vel
    global des_joint_acc
    global des_joint_eff

    #pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
    des_joint_vel = [0,0,0,0,0,0]
    des_joint_vel = control_angle(des_joint_pos,cur_joint_pos)

    w_max = 0.3
    for i in range(0,6):
        des_joint_vel[i] = limit(des_joint_vel[i],-w_max,w_max)
    
    #rospy.loginfo(joint_traj_msg)
    print(des_joint_vel)
    
    velocity_scale = 2.0
    for i in range(0,6):
        des_joint_vel[i] = des_joint_vel[i]*velocity_scale
    
    joint_traj_msg.points = [JointTrajectoryPoint(velocities=des_joint_vel)]
    pub.publish(joint_traj_msg)


def main():
    rospy.init_node('pub_ang', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, js_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


#######################################################################################################################

###################################################################################################################
'''
name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,
  wrist_3_joint]
position: [1.0529385805130005, 0.023707032203674316, -1.5739391485797327, -2.0011566321002405, 0.008474345318973064, 0.008632761426270008]
velocity: [0.0, 0.0, -0.0, 0.0037630468141287565, 0.0, 0.0]
effort: [-0.008967242203652859, -3.6048314571380615, -0.2421155422925949, 0.6298419833183289, -0.22418105602264404, -0.06405173242092133]
'''
###################################################################################################################
