#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import numpy as np
import math

global x_dot
x_dot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global tool_pos_linear
tool_pos_linear = [0.0, 0.0, 0.0]

global tool_pos_angular
tool_pos_angular = [0.0, 0.0, 0.0, 0.0]

global alpha
alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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

def calc_jacob(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    jacobian = np.array([[ (2183*np.cos(q1))/20000 + (823*np.cos(q1)*np.cos(q5))/10000 + (17*np.cos(q2)*np.sin(q1))/40 - (1569*np.sin(q1)*np.sin(q2)*np.sin(q3))/4000 + (823*np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5))/10000 - (591*np.cos(q2 + q3)*np.sin(q1)*np.sin(q4))/6250 - (591*np.sin(q2 + q3)*np.cos(q4)*np.sin(q1))/6250 + (1569*np.cos(q2)*np.cos(q3)*np.sin(q1))/4000, np.cos(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.cos(q2)*np.cos(q5)*np.sin(q3)*np.sin(q4))/10000 - (823*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))/10000 - (823*np.sin(q1)*np.sin(q5))/10000 + (823*np.cos(q1)*np.cos(q3)*np.cos(q5)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q1)*np.cos(q4)*np.cos(q5)*np.sin(q2)*np.sin(q3))/10000, 0],
	[ (2183*np.sin(q1))/20000 - (17*np.cos(q1)*np.cos(q2))/40 + (823*np.cos(q5)*np.sin(q1))/10000 - (823*np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5))/10000 + (591*np.cos(q2 + q3)*np.cos(q1)*np.sin(q4))/6250 + (591*np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))/6250 - (1569*np.cos(q1)*np.cos(q2)*np.cos(q3))/4000 + (1569*np.cos(q1)*np.sin(q2)*np.sin(q3))/4000, np.sin(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.sin(q5))/10000 - (823*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q1))/10000 + (823*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3)*np.sin(q4))/10000 + (823*np.cos(q3)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q4)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q3))/10000,                      0],
	[                                                                                                                                                                                                                                                                                            0,                                                      (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 - (17*np.cos(q2))/40 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 + (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                                                                                                                                           - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                     0],
	[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                  np.sin(q1),                                                                                                                           np.sin(q1),                                                                                                np.sin(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.cos(q1),   np.cos(q5)*np.sin(q1) - np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5)],
	[0,                                                                                                                                                                                                 -np.cos(q1),                                                                                                                          -np.cos(q1),                                                                                               -np.cos(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.sin(q1), - np.cos(q1)*np.cos(q5) - np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5)],
	[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        1,                                                                                                                                                                                                        0,                                                                                                                                 0,                                                                                                      0,                                                                                                                                                                                                                                  -np.cos(q2 + q3 + q4),                            -np.sin(q2 + q3 + q4)*np.sin(q5)]])
    return jacobian

def manipulability(J):
    m = np.linalg.det(np.matmul(J,np.transpose(J)))
    return m

def calc_q_dot(q,x_dot):
    J = calc_jacob(q)
    #print(manipulability(J))
    q_dot = np.linalg.pinv(J).dot(x_dot)
    return q_dot

pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)

def X_dot_callback(X_dot_msg):
    global x_dot
    x_dot[0] = X_dot_msg.linear.x
    x_dot[1] = X_dot_msg.linear.y
    x_dot[2] = X_dot_msg.linear.z
    x_dot[3] = X_dot_msg.angular.x
    x_dot[4] = X_dot_msg.angular.y
    x_dot[5] = X_dot_msg.angular.z
    #print("callback 1 -----")

global EE_region_flag, EE_last_region_flag
EE_region_flag = 'reg_1'
EE_last_region_flag = 'reg_1'

global Cx, Cy, Cz
Cx = 0.10634792505818291
Cy = -0.5287466779966042
Cz = 0.5652483044300174

def EE_region(Px,Py,Pz):
    global Cx, Cy, Cz

    R_in = 0.10
    R_out = 0.15

    S_in = (Px-Cx)**2 + (Py-Cy)**2 + (Pz-Cz)**2 - R_in**2
    S_out = (Px-Cx)**2 + (Py-Cy)**2 + (Pz-Cz)**2 - R_out**2

    current_reg = 'reg_1'
    if(S_in<=0):
        current_reg = 'reg_1'
    elif((S_in>0) and (S_out<=0)):
        current_reg = 'reg_2'
    else:
        current_reg = 'reg_3'
    return current_reg

global entry_reg_2
entry_reg_2 = True
global reg_2_V0
reg_2_V0 = 0.0

def js_callback(joint_states):
    # save current joint states
    global x_dot
    #print("callback 2", x_dot)
    global cur_joint_pos
    global cur_joint_vel
    global cur_joint_acc
    global cur_joint_eff

    cur_joint_pos = joint_states.position
    cur_joint_vel = joint_states.velocity
    cur_joint_acc = [0.0,0.0,0.0,0.0,0.0,0.0]
    cur_joint_eff = joint_states.effort
    #print(cur_joint_pos)

    # calculate desired joint states
    global des_joint_pos
    global des_joint_vel
    global des_joint_acc
    global des_joint_eff

    global alpha
    EE_region_flag = EE_region(alpha[0],alpha[1],alpha[2])
    # print(EE_inside_ws)

    des_joint_vel = [0,0,0,0,0,0] 

    if(EE_region_flag == 'reg_1'):
        global alpha
        global Cx, Cy, Cz
        global x_dot
        #---------------------------- Save last radial velocity in region 1 -----------------------
        Vxyz = np.array([x_dot[0], x_dot[1], x_dot[2]])
        v1 = np.array([alpha[0]-Cx, alpha[1]-Cy, alpha[2]-Cz])
        v2 = np.array([x_dot[0], x_dot[1], x_dot[2]])
        OE_vect = v1 / np.linalg.norm(v1)
        v1_v2_dot = np.dot(v1,v2)
        v1_v2_th = math.acos(v1_v2_dot/(np.linalg.norm(v1)*np.linalg.norm(v2)))
        Vr = np.linalg.norm(Vxyz)*math.cos(v1_v2_th)*OE_vect
        global reg_2_V0
        reg_2_V0 = Vr
        #----------------------------------------------------------------------------------------------
    elif(EE_region_flag == 'reg_2'):
        global alpha
        global Cx, Cy, Cz
        global x_dot
        v1 = np.array([alpha[0]-Cx, alpha[1]-Cy, alpha[2]-Cz])
        v2 = np.array([x_dot[0], x_dot[1], x_dot[2]])
        
        if(np.linalg.norm(v2)<=0.001):
           pass
        else:
            v1_v2_dot = np.dot(v1,v2)
            v1_v2_th = math.acos(v1_v2_dot/(np.linalg.norm(v1)*np.linalg.norm(v2)))

            if(v1_v2_dot>=0):
                #EE going outwards
                OE_vect = v1 / np.linalg.norm(v1)
                Vxyz = np.array([x_dot[0], x_dot[1], x_dot[2]])

                Vr = np.linalg.norm(Vxyz)*math.cos(v1_v2_th)*OE_vect
                Vt = Vxyz - Vr

                Vr_n = np.array([0.0, 0.0, 0.0])

                R = np.linalg.norm(v1)
                global reg_2_V0
                V_in = reg_2_V0
                R_in = 0.10
                R_out = 0.15
                Vr_i = np.linalg.norm(V_in)*(R-R_out)/(R_in - R_out)
                if ((np.linalg.norm(Vr))<=Vr_i):
                    pass
                else:
                    Vr_n = Vr_i
                    Vxyz_n = Vr_n + Vt
                    global x_dot
                    x_dot[0] = Vxyz_n[0]
                    x_dot[1] = Vxyz_n[1]
                    x_dot[2] = Vxyz_n[2]
            else:
                #EE moving inwards
                pass

    else:
        global alpha
        global Cx, Cy, Cz
        global x_dot
        v1 = np.array([alpha[0]-Cx, alpha[1]-Cy, alpha[2]-Cz])
        v2 = np.array([x_dot[0], x_dot[1], x_dot[2]])

        if(np.linalg.norm(v2)<=0.001):
           pass
        else:
            v1_v2_dot = np.dot(v1,v2)
            v1_v2_th = math.acos(v1_v2_dot/(np.linalg.norm(v1)*np.linalg.norm(v2)))
            #print(v1_v2_dot, v1_v2_th*180.0/math.pi)
            
            if(v1_v2_dot>=0):
                global x_dot
                x_dot[0] = 0
                x_dot[1] = 0
                x_dot[2] = 0
            else:
                v2 = np.linalg.norm(v2)*math.cos(v1_v2_th)*(v1/np.linalg.norm(v1))
                global x_dot
                x_dot[0] = v2[0]
                x_dot[1] = v2[1]
                x_dot[2] = v2[2]
        x_dot[3] = 0.0
        x_dot[4] = 0.0
        x_dot[5] = 0.0
    print(x_dot)

    des_joint_vel = calc_q_dot(cur_joint_pos,x_dot)

    w_max = 0.3
    for i in range(0,6):
        des_joint_vel[i] = limit(des_joint_vel[i],-w_max,w_max)
    
    #rospy.loginfo(joint_traj_msg)
    #print("des_joint_vel", des_joint_vel)
    
    velocity_scale = 2.0
    for i in range(0,6):
        des_joint_vel[i] = des_joint_vel[i]*velocity_scale
    
    joint_traj_msg.points = [JointTrajectoryPoint(velocities=des_joint_vel)] #_________________
    pub.publish(joint_traj_msg)

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


def main():
    rospy.init_node('X_dot_to_q_dot', anonymous=False)
    rospy.Subscriber('/joint_states', JointState, js_callback)
    rospy.Subscriber('/X_dot', Twist, X_dot_callback)
    rospy.Subscriber('/tf', TFMessage , tf_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

