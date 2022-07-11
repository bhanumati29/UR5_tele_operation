import csv
import matplotlib.pyplot as plt

t = []

x_0   = []
y_0   = []
z_0   = []
phi_0 = []
th_0  = []
psi_0 = []

x   = []
y   = []
z   = []
phi = []
th  = []
psi = []

Kp_xyz = []
Kp_pts = []

with open('data_log_ur5_haptic Mon Jun 27 04:44:07 2022.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        t.append(float(row[0]))
        x_0.append(float(row[1]))
        y_0.append(float(row[2]))
        z_0.append(float(row[3]))
        phi_0.append(float(row[4]))
        th_0.append(float(row[5]))
        psi_0.append(float(row[6]))

        x.append(float(row[7]))
        y.append(float(row[8]))
        z.append(float(row[9]))
        phi.append(float(row[10]))
        th.append(float(row[11]))
        psi.append(float(row[12]))

        Kp_xyz.append(float(row[13]))
        Kp_pts.append(float(row[14]))

plt.plot(t, x, color = 'k', linestyle = 'solid',label = 'Kp_xyz: '+str(Kp_xyz[0]))
plt.plot(t, x_0, color = 'r', linestyle = 'solid')

plt.xlabel('t (s)')
plt.ylabel('x (m)')
plt.title('X vs t', fontsize = 20)
plt.grid()
plt.legend()
plt.show()
