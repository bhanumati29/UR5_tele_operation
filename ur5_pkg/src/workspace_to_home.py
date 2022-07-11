#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import numpy as np
import math
import time
from sensor_msgs.msg import Joy

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
    m = np.linalg.det(np.matmul(J,np.transpose(J)))
    return m

def calc_q_dot_1(J,M,K_alpha,alpha_0,alpha):
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

def calc_q_dot(q,x_dot):
    J = calc_jacob(q)
    #print(manipulability(J))
    q_dot = np.linalg.pinv(J).dot(x_dot)
    return q_dot

pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)

def get_des_traj(t_):
    # rotor load estimation
    '''x0   = 0.33
    y0   = 0.11
    z0   = 0.564

    A = 31.8*math.pi/180.0
    freq = 0.1
    w = 2.0*math.pi*freq
    phi0 = A*math.sin(w*t_)

    #phi0 = 0.0
    th0  = 0.0
    psi0 = 0.0'''

    # # infinity
    # freq = 0.1
    # w = 2.0*math.pi*freq
    
    # Ax = 0.15
    # Ay = 0.1

    # x0   = 0.33 + Ax*math.sin(w*t_)
    # y0   = 0.11 + Ay*math.sin(2.0*w*t_)
    # z0   = 0.564
    # phi0 = 0.0
    # th0  = 0.0
    # psi0 = 0.0

    # st line
    x1 = 0.10634792505818291
    y1 = -0.5287466779966042
    z1 = 0.5652483044300174

    x2 = 0.656674405254
    y2 = -0.523855639958
    z2 = 0.407785352766

    u = 0.015 #0.01154700538

    global P1
    P1 = np.array([x1, y1, z1])
    P2 = np.array([x2, y2, z2])

    T = np.linalg.norm(P2-P1) / u

    if(t_>T):
        t_ = T
    #print(t_)

    P = P1 + u*t_*(P2-P1)/np.linalg.norm(P2-P1)
    #print(t_,' ---> ' ,(P - P1)*100.0)

    x0   = P[0]
    y0   = P[1]
    z0   = P[2]
    phi0 = 0.0
    th0  = 0.0
    psi0 = 0.0

    
    alpha_0_ = [x0, y0, z0, phi0, th0, psi0]
    #print(alpha_0_)
    return alpha_0_

global js_start
global t_init
js_start = True
t_init = 0

def X_dot_callback(X_dot_msg):
    global x_dot
    x_dot[0] = X_dot_msg.linear.x
    x_dot[1] = X_dot_msg.linear.y
    x_dot[2] = X_dot_msg.linear.z
    x_dot[3] = X_dot_msg.angular.x
    x_dot[4] = X_dot_msg.angular.y
    x_dot[5] = X_dot_msg.angular.z
    #print("callback 1 -----")

EE_inside_ws = True

global Cx, Cy, Cz
Cx = 0.10634792505818291
Cy = -0.5287466779966042
Cz = 0.5652483044300174

def EE_inside_sphere(Px,Py,Pz):
    global Cx, Cy, Cz

    R = 0.10

    S = (Px-Cx)**2 + (Py-Cy)**2 + (Pz-Cz)**2 - R**2

    if(S<=0):
        return True
    else:
        return False

global opration_mode
operation_mode = True # True: js, False: Traj

def js_callback(joint_states):
    ########################### ws starts
    global opration_mode
    if(operation_mode==True):
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
        EE_inside_ws = EE_inside_sphere(alpha[0],alpha[1],alpha[2])
        # print(EE_inside_ws)

        des_joint_vel = [0,0,0,0,0,0] 

        if(EE_inside_ws==True):
            pass
        else:
            global alpha
            global Cx, Cy, Cz
            global x_dot
            v1 = np.array([Cx-alpha[0], Cy-alpha[1], Cz-alpha[2]])
            v2 = np.array([x_dot[0], x_dot[1], x_dot[2]])

            if(np.linalg.norm(v2)<=0):
                pass
            else:
                v1_v2_dot = np.dot(v1,v2)
                v1_v2_th = math.acos(v1_v2_dot/(np.linalg.norm(v1)*np.linalg.norm(v2)))
                #print(v1_v2_dot, v1_v2_th*180.0/math.pi)
                
                if(v1_v2_dot<=0):
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
        #print(x_dot)

        des_joint_vel = calc_q_dot(cur_joint_pos,x_dot)

        global js_start
        global t_init
        js_start = True
        t_init = time.time()
    ########################### ws ends

    ########################### traj starts
    if(operation_mode==False):
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
        
        alpha_0 = get_des_traj(t_now)
        #print(alpha_0)
        
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
        des_joint_vel = calc_q_dot_1(J,M,K_alpha,alpha_0,alpha)
        #print(des_joint_vel)
        #des_joint_vel = [0,0,0,0,0,0]
    ########################### traj ends

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

def joy_callback(msg):
    axes = msg.axes
    buttons = msg.buttons
    # print(axes, buttons)
    # print(axes[3])
    mode_axes = axes[3]
    global opration_mode
    
    if(mode_axes<0.0):
        global operation_mode
        operation_mode = False # go to home using trajectory
    else:
        operation_mode = True # joystick mode in workspace
    print(operation_mode)

def main():
    rospy.init_node('X_dot_to_q_dot', anonymous=False)
    rospy.Subscriber('/joint_states', JointState, js_callback)
    rospy.Subscriber('/X_dot', Twist, X_dot_callback)
    rospy.Subscriber('/tf', TFMessage , tf_callback)
    rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
