#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import os.path
import time

global file, com_file_name
com_file_name = "xyz"

global h_x, h_y, h_z, h_phi, h_th, h_psi
h_x, h_y, h_z, h_phi, h_th, h_psi = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h
th1_h, th2_h, th3_h, th4_h, th5_h, th6_h = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

global roll_h, pitch_h, yaw_h
roll_h, pitch_h, yaw_h = 0.0, 0.0, 0.0

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

def Qx(th):
    qx = np.matrix([[1, 0, 0],[0, math.cos(th), -math.sin(th)],[0, math.sin(th), math.cos(th)]])
    return qx

def Qy(th):
    qy = np.matrix([[math.cos(th), 0, math.sin(th)],[0, 1, 0],[-math.sin(th), 0, math.cos(th)]])
    return qy

def Qz(th):
    qz = np.matrix([[math.cos(th), -math.sin(th), 0],[math.sin(th), math.cos(th), 0], [0, 0, 1]])
    return qz

def deg2rad(th):
    return th*(math.pi/180.0)

def rad2deg(th):
    return th*(180.0/math.pi)

def Q(th1, th2, th3, th4, th5, th6):
    q1 = Qz(th1)
    q2 = Qx(th2)
    q3 = Qx(th3)
    q4 = Qy(th4)
    q5 = Qx(th5)
    q5_1 = Qy(deg2rad(90.0*0))
    q5_2 = Qz(deg2rad(180.0*0))
    q6 = Qy(th6)
    q6_1 = Qy(deg2rad(180.0*0))
    q6_2 = Qx(deg2rad(180.0*0))
    q6_3 = Qz(deg2rad(90.0))
    q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q5_1),q5_2),q6),q6_1),q6_2)
    q = np.matmul(q, q6_3)
    return q

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
    # print('0 ',alpha)
    # print('c ', alpha_0)
    #print(alpha[3]*180.0/math.pi)
    #print(manipulability(J))
    global diff_alpha
    for i in range(0,6):
        diff_alpha[i] = alpha_0[i] - alpha[i]
    # print(diff_alpha)

    v_des = np.matmul(np.linalg.pinv(M),K_alpha.dot(np.array(diff_alpha)))
    # print(v_des[0],v_des[1],v_des[2],v_des[3],v_des[4],v_des[5])
    q_dot = np.matmul(np.matmul(np.linalg.pinv(J),np.linalg.pinv(M)),K_alpha).dot(np.array(diff_alpha))
    # print(q_dot)
    return q_dot

pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)

def quat2eul(q0,q1,q2,q3):
    phi = math.atan2(2*(q0*q1 + q2*q3),1-2*((q1)**2+(q2)**2))
    th  = math.asin(2*(q0*q2 - q3*q1))
    psi = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))
    return phi, th, psi

global js_start
global t_init, t_now
js_start = True
t_init = 0
t_now = 0

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
    global h_x, h_y, h_z, h_phi, h_th, h_psi

    h_x = msg2.pose.position.x
    h_y = msg2.pose.position.y
    h_z = msg2.pose.position.z

    h_q0 = msg2.pose.orientation.w
    h_q1 = msg2.pose.orientation.x
    h_q2 = msg2.pose.orientation.y
    h_q3 = msg2.pose.orientation.z

    h_phi, h_th, h_psi = quat2eul(h_q0, h_q1, h_q2, h_q3)
    # print(h_phi, h_th, h_psi)

def h_js_callback(msg3):
    global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h

    th1_h_off = deg2rad(0.0)
    th2_h_off = deg2rad(-14.0)
    th3_h_off = deg2rad(25.0 + 90.0)
    th4_h_off = deg2rad(182.5)
    th5_h_off = deg2rad(-210)
    th6_h_off = deg2rad(-183)

    th1_h = -msg3.position[0] - th1_h_off
    th2_h = msg3.position[1] - th2_h_off
    th3_h = msg3.position[2] - th3_h_off
    th4_h = -msg3.position[5] - th4_h_off ############ !!!
    th5_h = msg3.position[4] - th5_h_off
    th6_h = -msg3.position[3] - th6_h_off ############ !!!
    th6_h = -th6_h
    # print(rad2deg(th1_h), rad2deg(th2_h), rad2deg(th3_h), rad2deg(th4_h), rad2deg(th5_h), rad2deg(th6_h))

    q = Q(th1_h, th2_h, th3_h, th4_h, th5_h, th6_h)
    q0_h = math.sqrt(1 + q[0,0] + q[1,1] + q[2,2]) /2
    q1_h = (q[2,1] - q[1,2])/( 4 *q0_h)
    q2_h = (q[0,2] - q[2,0])/( 4 *q0_h)
    q3_h = (q[1,0] - q[0,1])/( 4 *q0_h)
    # print(q0_h, q1_h, q2_h, q3_h)
    
    # r = R.from_matrix(q)
    # q = r.as_quat() # last term is scalar

    global roll_h, pitch_h, yaw_h
    roll_h, pitch_h, yaw_h = quat2eul(q0_h, q1_h, q2_h, q3_h)
    yaw_h = yaw_h - np.pi/2
    roll_h  = limit(roll_h,-deg2rad(30),deg2rad(30))
    pitch_h = limit(pitch_h,-deg2rad(30),deg2rad(30))
    yaw_h   = limit(yaw_h,-deg2rad(30),deg2rad(30))
    # print(rad2deg(roll_h),rad2deg(pitch_h),rad2deg(yaw_h))

def get_des_pos_haptic():
    # current end-effector pose, change it by printing the alpha value when you start to implement the code.
    alpha_c = [-0.4311814584656362, -0.1904948842722761, 0.4677669435133598, 8.655360175626746e-06, 6.34089473934378e-05, 1.12014001956086*0.0]
    xc   = alpha_c[0]
    yc   = alpha_c[1]
    zc   = alpha_c[2]
    phic = alpha_c[3]
    thc  = alpha_c[4]
    psic = alpha_c[5]

    # haptic device's stylus's x, y and z when it is is the inkwell. keep it in inkwell before running this code.
    global h_x, h_y, h_z, h_phi, h_th, h_psi
    hx_off = 0
    hy_off = 0.085595458984375
    hz_off = -0.09794529724121094
 
    # print(h_x, h_y, h_z)
    # print(h_phi*rad2deg, h_th*rad2deg, h_psi*rad2deg)
    # print(h_z-hz_off)

    dx_h = (h_x-hx_off)
    dy_h = - (h_y-hy_off)
    dz_h = (h_z-hz_off)

    dx_h, dy_h = dy_h, -dx_h

    global t_now
    ccc = 0.0
    if(t_now>17.0):
        ccc = 0.0

    x0   = xc + dx_h + 0.1*ccc
    y0   = yc + dy_h + 0.1*ccc
    z0   = zc + dz_h + 0.1*ccc

    global roll_h, pitch_h, yaw_h
    phi0 = phic*0 + 0*h_phi + deg2rad(15.0)*ccc + roll_h
    th0  = thc*0  + 0*h_th + deg2rad(15.0)*ccc + pitch_h
    psi0 = psic*0 + 0*h_psi + deg2rad(15.0)*ccc +  yaw_h
    
    alpha_0_ = [x0, y0, z0, phi0, th0, psi0]
    # print('alpha_0->', alpha_0_)
    return alpha_0_

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
    # print('alpha_c->',alpha)

    global js_start
    global t_init, t_now

    if(js_start==True):
        global js_start
        global t_init
        js_start = False
        t_init = time.time()
    
    t_now = time.time() - t_init
    
    alpha_0 = get_des_pos_haptic()
    # print(t_now, alpha_0)
    # print(t_now, alpha)
    
    vvvvvv = 2.0
    k_xyz = vvvvvv #2.5*0.2
    k_pts = vvvvvv #2.5*0.2

    print(t_now)
    #_________________data collection_____________________________________________
    if(t_now>5.0):
        data_0 = str(alpha_0[0]) + ',' + str(alpha_0[1]) + ',' + str(alpha_0[2]) + ','  + str(alpha_0[3]) + ',' + str(alpha_0[4]) + ',' + str(alpha_0[5]) 
        data_c = str(alpha[0]) + ',' + str(alpha[1]) + ',' + str(alpha[2]) + ','  + str(alpha[3]) + ',' + str(alpha[4]) + ',' + str(alpha[5]) 
        data = str(t_now) + ',' + data_0 + ',' + data_c + ',' + str(k_xyz) + ',' + str(k_pts) + '\n'
        global com_file_name
        file = open(com_file_name,'a')
        file.write(data)
        file.close()

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
    # print(t_now, des_joint_vel)
    # des_joint_vel = [0,0,0,0,0,0]

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

def main():
    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)
    global com_file_name
    file_name = 'data_log_ur5_haptic '  + str(t) + '.csv'
    save_path = '/home/bhanu/catkin_ws/src/ur5_pkg/src/data_log_files_ur5_haptic/'
    com_file_name = os.path.join(save_path, file_name)
    file = open(com_file_name,'w')
    file.close()
    time.sleep(2)

    rospy.init_node('task_space_traj_tracking', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, js_callback)
    rospy.Subscriber('/tf', TFMessage , tf_callback)
    rospy.Subscriber('/phantom/pose', PoseStamped, hp_callback)
    rospy.Subscriber('/phantom/joint_states', JointState, h_js_callback)
    rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
