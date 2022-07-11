import numpy as np 

# cuboidal workspace definition
global O, P1, P2, P3, P4, P5, P6, P7 # these are not vectors, these are points
# O = np.array([-0.0847295430773, -0.617645733183, 0.2250026551])
O = np.array([1,2,3])

# edge vectors
OP1 = np.array([0.2,0,0])
OP2 = np.array([0,0.2,0])
OP3 = np.array([0,0,0.2])

P1 = O + OP1
P2 = O + OP2
P3 = O + OP3
P4 = O + OP1 + OP2
P5 = O + OP2 + OP3
P6 = O + OP1 + OP3
P7 = O + OP1 + OP2 + OP3

global C
C = (O + P7)/2.0

# print('O',O)
# print('P1',P1)
# print('P2',P2)
# print('P3',P3)
# print('P4',P4)
# print('P5',P5)
# print('P6',P6)
# print('P7',P7)
# print('C',C)

global Vol_cuboid
Vol_cuboid = np.absolute( np.dot(np.cross(OP1, OP2), OP3) )

# print (Vol_cuboid)

def vol_pyramid(p0, p1, p2, p3):         # p3 is the apex of pyramid
    p01 = p1 - p0
    p02 = p2 - p0
    p03 = p3 - p0
    
    vol_py = np.absolute(np.dot(np.cross(p01, p02), p03)/3.0)
    return vol_py

'''x = np.array([0,0,0])
y = np.array([1.0,0,0])
z = np.array([0,2.0,0])
w = np.array([0,0,5.0])

print(vol_pyramid(x, y, z, w))'''

def EE_inside_cuboid(P):
    inside = True
    vol_py1 = vol_pyramid(O, P1, P6, P)
    vol_py2 = vol_pyramid(O, P2, P5, P)
    vol_py3 = vol_pyramid(O, P1, P4, P)
    vol_py4 = vol_pyramid(P2, P5, P7, P)
    vol_py5 = vol_pyramid(P1, P6, P7, P)
    vol_py6 = vol_pyramid(P3, P5, P7, P)

    #print(vol_py1, vol_py2, vol_py3, vol_py4, vol_py5, vol_py6)
    vol_py_total = vol_py1 + vol_py2 + vol_py3 + vol_py4 + vol_py5 + vol_py6
    print(vol_py_total, Vol_cuboid)

    tol = 0.5
    vol_py_total = vol_py1 + vol_py2 + vol_py3 + vol_py4 + vol_py5 + vol_py6
    vol_error = np.absolute(( vol_py_total - Vol_cuboid)/ Vol_cuboid)*100.0

    if (vol_error < tol):
        inside = True
    else:
        inside = False
    print(vol_error)
    return inside

print(EE_inside_cuboid([1, 2.0 - 0.002, 3]))

