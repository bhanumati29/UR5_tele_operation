#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

global h_x, h_y, h_z
h_x, h_y, h_z = 0.0, 0.0, 0.0

global x_dot
x_dot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global tool_pos_linear
tool_pos_linear = [0.0, 0.0, 0.0]

global tool_pos_angular
tool_pos_angular = [0.0, 0.0, 0.0, 0.0]

global alpha
alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global diff_alpha
diff_alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

def cal_K_alpha(k_p):
    K_alpha = np.array([[k_p[0], 0.0, 0.0, 0.0, 0.0, 0.0], 
                        [0.0, k_p[1], 0.0, 0.0, 0.0, 0.0,],
                        [0.0, 0.0, k_p[2], 0.0, 0.0, 0.0,], 
                        [0.0, 0.0, 0.0, k_p[3], 0.0, 0.0], 
                        [0.0, 0.0, 0.0, 0.0, k_p[4], 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, k_p[5]]])
    return K_alpha

def calc_M(phi, th, psi):
    m11 = 1.0
    m12 = np.sin(phi)*np.tan(th)
    m13 = np.cos(phi)*np.tan(th)
    m21 = 0.0
    m22 = np.cos(phi)
    m23 = -np.sin(phi)
    m31 = 0.0
    m32 = np.sin(phi)/np.cos(th)
    m33 = np.cos(phi)/np.cos(th)

    M = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 1.0, 0.0, 0.0, 0.0, 0.0,],
                 [0.0, 0.0, 1.0, 0.0, 0.0, 0.0,], 
                 [0.0, 0.0, 0.0, m11, m12, m13], 
                 [0.0, 0.0, 0.0, m21, m22, m23],
                 [0.0, 0.0, 0.0, m31, m32, m33]])
    return M

def manipulability(J):
    m = math.sqrt(np.linalg.det(np.matmul(J,np.transpose(J))))
    return m

def calc_q_dot(J,M,K_alpha,alpha_0,alpha):
    #print(alpha)
    #print(alpha_0)
    #print(alpha[3]*180.0/math.pi)
    #print(manipulability(J))
    global diff_alpha
    for i in range(0,6):
        diff_alpha[i] = alpha_0[i] - alpha[i]
    #print(diff_alpha)

    q_dot = np.matmul(np.matmul(np.linalg.pinv(J),np.linalg.pinv(M)),K_alpha).dot(np.array(diff_alpha))
    #print(q_dot)
    return q_dot

pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)

def get_des_pos_haptic():
    xc, yc, zc, phic, thc, psic = -0.5997612101616936, -0.06073770506450917, 0.43478579811645157, -3.100452033292313, -0.04909078830447712, 1.3444715525329196

    global h_x, h_y, h_z

    x0   = xc + h_x*0
    y0   = yc + h_y*0
    z0   = zc + h_z*0
    phi0 = phic
    th0  = thc
    psi0 = psic
    
    alpha_0_ = [x0, y0, z0, phi0, th0, psi0]
    # print(alpha_0_)
    return alpha_0_

def quat2eul(q0,q1,q2,q3):
    phi = math.atan2(2*(q0*q1 + q2*q3),1-2*((q1)**2+(q2)**2))
    th  = math.asin(2*(q0*q2 - q3*q1))
    psi = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))
    return phi, th, psi

global js_start
global t_init
js_start = True
t_init = 0

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
    #print(alpha)

    global js_start
    global t_init

    if(js_start==True):
        global js_start
        global t_init
        js_start = False
        t_init = time.time()
    
    t_now = time.time() - t_init
    #print(t_now)
    
    alpha_0 = get_des_pos_haptic()
    # print(t_now, alpha_0)
    # print(t_now, alpha)
    
    k_xyz = 2.5
    k_pts = 2.5

    k_x   = k_xyz
    k_y   = k_xyz
    k_z   = k_xyz
    k_phi = k_pts
    k_th  = k_pts
    k_psi = k_pts
    k_p = [k_x, k_y, k_z, k_phi, k_th, k_psi]
    K_alpha = cal_K_alpha(k_p)
    #print(K_alpha)
    #print("---")

    M = calc_M(alpha[3], alpha[4], alpha[5])
    #print(M)
    #print("---")

    J = calc_jacob(cur_joint_pos)
    #print(J)
    #print("---")

    des_joint_vel = [0,0,0,0,0,0]
    des_joint_vel = calc_q_dot(J,M,K_alpha,alpha_0,alpha)
    print(t_now, des_joint_vel)
    des_joint_vel = [0,0,0,0,0,0]
    #print(des_joint_vel)
    
    w_max = 0.3 # rotor load estimation

    for i in range(0,6):
        des_joint_vel[i] = limit(des_joint_vel[i],-w_max,w_max)
    
    #rospy.loginfo(joint_traj_msg)
    #print("des_joint_vel", des_joint_vel)
    
    velocity_scale = 2.0
    for i in range(0,6):
        des_joint_vel[i] = des_joint_vel[i]*velocity_scale
    
    joint_traj_msg.points = [JointTrajectoryPoint(velocities=des_joint_vel)] 
    pub.publish(joint_traj_msg)

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
        # print(alpha)

def hp_callback(msg2): 
    global h_x, h_y, h_z

    h_x = msg2.pose.position.x
    h_y = msg2.pose.position.y
    h_z = msg2.pose.position.z

def main():
    rospy.init_node('task_space_traj_tracking', anonymous=False)
    rospy.Subscriber('/joint_states', JointState, js_callback)
    rospy.Subscriber('/tf', TFMessage , tf_callback)
    rospy.Subscriber('/phantom/pose', PoseStamped, hp_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
