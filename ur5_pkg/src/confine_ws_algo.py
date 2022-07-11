EE_inside_ws = True

global Cx, Cy, Cz
Cx = 0.10634792505818291
Cy = -0.5287466779966042
Cz = 0.5652483044300174

def EE_inside_sphere(Px,Py,Pz):
    global Cx, Cy, Cz

    R = 0.20

    S = (Px-Cx)**2 + (Py-Cy)**2 + (Pz-Cz)**2 - R**2

    if(S<=0):
        return True
    else:
        return False

