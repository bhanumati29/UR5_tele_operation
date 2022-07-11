from scipy.spatial.transform import Rotation as R
import numpy as np
import math

def quat2eul(q0,q1,q2,q3):
    phi = math.atan2(2*(q0*q1 + q2*q3),1-2*((q1)**2+(q2)**2))
    th  = math.asin(2*(q0*q2 - q3*q1))
    psi = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))
    return phi, th, psi

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
    q5_1 = Qy(deg2rad(90.0))
    q5_2 = Qz(deg2rad(180.0))
    q6 = Qy(th6)
    q6_1 = Qy(deg2rad(180.0))
    q6_2 = Qx(deg2rad(180.0))

    q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q5_1),q5_2),q6),q6_1),q6_2)
    return q

th1, th2, th3, th4, th5, th6 = deg2rad(25), deg2rad(57), 0.0, 0.0, 0.0, deg2rad(0)
q = Q(th1, th2, th3, th4, th5, th6)
print(q)
print('---')
qw = math.sqrt(1 + q[0,0] + q[1,1] + q[2,2]) /2
qx = (q[2,1] - q[1,2])/( 4 *qw)
qy = (q[0,2] - q[2,0])/( 4 *qw)
qz = (q[1,0] - q[0,1])/( 4 *qw)
# print(qx,qy,qz,qw)
r = R.from_matrix(q)
q = r.as_quat()
# print(q)

roll_h, pitch_h, yaw_h = quat2eul(q[3], q[0], q[1], q[2])
print(rad2deg(roll_h),rad2deg(pitch_h),rad2deg(yaw_h))
