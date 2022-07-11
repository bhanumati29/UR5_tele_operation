import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy import signal as sg

def decode_csv(file_name):
	t = np.array([])

	x_0   = np.array([])
	y_0   = np.array([])
	z_0   = np.array([])
	phi_0 = np.array([])
	th_0  = np.array([])
	psi_0 = np.array([])

	x   = np.array([])
	y   = np.array([])
	z   = np.array([])
	phi = np.array([])
	th  = np.array([])
	psi = np.array([])

	Kp_xyz = np.array([])
	Kp_pts = np.array([])

	with open(file_name,'r') as csvfile:
		lines = csv.reader(csvfile, delimiter=',')
		for row in lines:
			t = np.append(t, float(row[0]))
			x_0 = np.append(x_0, float(row[1]))
			y_0 = np.append(y_0, float(row[2]))
			z_0 = np.append(z_0, float(row[3]))
			phi_0 = np.append(phi_0, float(row[4]))
			th_0 = np.append(th_0, float(row[5]))
			psi_0 = np.append(psi_0, float(row[6]))

			x = np.append(x, float(row[7]))
			y = np.append(y, float(row[8]))
			z = np.append(z, float(row[9]))
			phi = np.append(phi, float(row[10]))
			th = np.append(th, float(row[11]))
			psi = np.append(psi, float(row[12]))

			Kp_xyz = np.append(Kp_xyz, float(row[13]))
			Kp_pts = np.append(Kp_pts, float(row[14]))

			t = np.transpose(t)
			x_0   = np.transpose(x_0)
			y_0   = np.transpose(y_0)
			z_0   = np.transpose(z_0)
			phi_0 = np.transpose(phi_0)
			th_0  = np.transpose(th_0)
			psi_0 = np.transpose(psi_0)

			x   = np.transpose(x)
			y   = np.transpose(y)
			z   = np.transpose(z)
			phi = np.transpose(phi)
			th  = np.transpose(th)
			psi = np.transpose(psi)

	return t, x_0, y_0, z_0, phi_0, th_0, psi_0, x, y, z, phi, th, psi, Kp_xyz[-1], Kp_pts[-1]

def disp_data(t, data_0, data, data_name, unit):
	plt.figure(1)
	plt.plot(t, data_0, color = 'r', linestyle = 'solid')
	plt.plot(t, data, color = 'b', linestyle = 'solid')
		
	plt.xlabel('t (s)')
	plt.ylabel(data_name + '(' + unit + ')')
	plt.title(data_name + ' vs t', fontsize = 20)
	plt.grid()
	plt.legend(["Input from haptic device", "Output from UR5"], loc ="upper right")
	plt.show()

input_file_name = 'data_log_ur5_haptic Mon Jun 27 05:40:12 2022.csv'
t, x_0, y_0, z_0, phi_0, th_0, psi_0, x, y, z, phi, th, psi, Kp_xyz, Kp_pts = decode_csv(input_file_name)

disp_data(t, x_0, x, 'x', 'm')

# using scipy -------------------------------------------------------
# corr = sg.correlate(x_0, x)
# corr /= np.max(corr)
# lags = sg.correlation_lags(len(x), len(x_0))

# plt.figure(2)
# plt.plot(corr)
# plt.show()

# print(len(x_0),len(x),len(corr))

# corr_max = max(corr)
# corr_max_index = np.where(corr == 1.0)
# print(corr_max, corr_max_index)

# using numpy ----------------------------------------------------------
x_0 = x_0[1000:2563] - np.mean(x_0[1000:2563])
x = x[1000:2563] - np.mean(x[1000:2563])
autocorr_xdm = np.correlate(x_0, x, mode='full')
nx = len(x)
# autocorr_xdm /= autocorr_xdm[nx - 1]
lags = np.arange(-nx + 1, nx)

plt.figure(3)
plt.plot(lags, autocorr_xdm, 'r')
plt.show()
