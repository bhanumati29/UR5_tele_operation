import csv
import matplotlib.pyplot as plt
from labellines import labelLines
from scipy import signal as sg
import numpy as np

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


def decode_multiple_csv(file_list):
	n_data = 8400
	n_obs = len(file_list)

	t = np.empty([n_data, n_obs])

	x_0   = np.empty([n_data, n_obs])
	y_0   = np.empty([n_data, n_obs])
	z_0   = np.empty([n_data, n_obs])
	phi_0 = np.empty([n_data, n_obs])
	th_0  = np.empty([n_data, n_obs])
	psi_0 = np.empty([n_data, n_obs])

	x   = np.empty([n_data, n_obs])
	y   = np.empty([n_data, n_obs])
	z   = np.empty([n_data, n_obs])
	phi = np.empty([n_data, n_obs])
	th  = np.empty([n_data, n_obs])
	psi = np.empty([n_data, n_obs])

	Kp_xyz = np.empty([1, n_obs])
	Kp_pts = np.empty([1, n_obs])

	i = 0
	for file_i in file_list:
		t_, x_0_, y_0_, z_0_, phi_0_, th_0_, psi_0_, x_, y_, z_, phi_, th_, psi_, Kp_xyz_, Kp_pts_ = decode_csv(file_i)
		t[:,i], x_0[:,i], y_0[:,i], z_0[:,i], phi_0[:,i], th_0[:,i], psi_0[:,i], x[:,i], y[:,i], z[:,i], phi[:,i], th[:,i], psi[:,i], Kp_xyz[:,i], Kp_pts[:,i] = t_[0:n_data], x_0_[0:n_data], y_0_[0:n_data], z_0_[0:n_data], phi_0_[0:n_data], th_0_[0:n_data], psi_0_[0:n_data], x_[0:n_data], y_[0:n_data], z_[0:n_data], phi_[0:n_data], th_[0:n_data], psi_[0:n_data], Kp_xyz_, Kp_pts_
		i += 1
	
	return t[:,0], x_0, y_0, z_0, phi_0, th_0, psi_0, x, y, z, phi, th, psi, Kp_xyz, Kp_pts

# -----------------------------------------------------------------------------------------------------------------

file_array = [
			#   'data_log_ur5_haptic Mon Jun 27 04:44:07 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 04:54:05 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 04:58:49 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 04:51:43 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 04:48:55 2022.csv', 
   			#   'data_log_ur5_haptic Mon Jun 27 04:57:12 2022.csv', 
   			#   'data_log_ur5_haptic Mon Jun 27 04:53:10 2022.csv', 
			  #---------------Kp_xyz !=0,Kp_pts =0 for above files--------------------------------
			
			#   'data_log_ur5_haptic Mon Jun 27 04:50:50 2022.csv', -------------------
			#   'data_log_ur5_haptic Mon Jun 27 04:55:57 2022.csv', -------------------
			#   'data_log_ur5_haptic Mon Jun 27 04:55:00 2022.csv', -------------------
			#   'data_log_ur5_haptic Mon Jun 27 04:49:47 2022.csv',  ------------------ 
			#  ----------- above files are not empty -------------------------------------------------

			#   'data_log_ur5_haptic Mon Jun 27 05:07:12 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:14:06 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:05:48 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:08:02 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:11:35 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:09:48 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:18:11 2022.csv'
			  #---------------Kp_xyz =0,Kp_pts !=0 for above files--------------------------------

			#   'data_log_ur5_haptic Mon Jun 27 05:12:29 2022.csv', ------------
			#   'data_log_ur5_haptic Mon Jun 27 05:08:57 2022.csv', -------------
			#   'data_log_ur5_haptic Mon Jun 27 05:10:42 2022.csv', -------------
			#  ----------- above files are not empty -------------------------------------------------

			#   'data_log_ur5_haptic Mon Jun 27 05:29:24 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:27:07 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:23:36 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:28:09 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:22:47 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:25:20 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:21:57 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:20:58 2022.csv', 
			  #---------------Kp_xyz !=0,Kp_pts !=0 for above files--------------------------------

			#   'data_log_ur5_haptic Mon Jun 27 05:26:08 2022.csv', ------------------------
			#   'data_log_ur5_haptic Mon Jun 27 05:24:24 2022.csv', ---------------------------
			#  ----------- above files are not empty -------------------------------------------------
			  
			#   'data_log_ur5_haptic Mon Jun 27 05:43:39 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:33:46 2022.csv'
			#   'data_log_ur5_haptic Mon Jun 27 05:35:22 2022.csv', 
			#   'data_log_ur5_haptic Mon Jun 27 05:38:22 2022.csv'
			  'data_log_ur5_haptic Mon Jun 27 05:40:12 2022.csv', 
			]

t, x_0, y_0, z_0, phi_0, th_0, psi_0, x, y, z, phi, th, psi, Kp_xyz, Kp_pts = decode_multiple_csv(file_array)

def disp_data(t, data_0, data, data_name, unit):
	dimension = np.shape(data)
	n_col = dimension[1]
	# print(dimension)
	
	plt.figure()
	plt.plot(t, data_0[:,0], color = 'r', linestyle = 'solid')
	# col = ['k', 'c', 'g', 'b', 'm', 'y']

	for i in range(0,n_col):
		# plt.plot(t, data[:,i], linestyle='solid', label='Kp_xyz: ' + str(Kp_xyz[0,i]) + ', '+'Kp_pts: ' + str(Kp_pts[0,i]))
		plt.plot(t, data[:,i], linestyle='solid', label=str(Kp_xyz[0,i]) + ', '+str(Kp_pts[0,i]))
		# plt.plot(t, data[:,i], linestyle='solid', label=str(i))

	labelLines(plt.gca().get_lines(), ha='left', va='bottom', align=True, zorder=2.5)

	plt.xlabel('t (s)')
	plt.ylabel(data_name + '(' + unit + ')')
	plt.title(data_name + ' vs t', fontsize = 20)
	plt.grid()
	plt.legend()
	# plt.show()

t   = t[1350:4000]
x_0 = x_0[1350:4000]
x   = x[1350:4000]

x_0 = x_0 - np.mean(x_0) 
x   = x   - np.mean(x) 

disp_data(t, x_0, x, 'x', 'm')
plt.show()

corr = sg.correlate(x_0, x, mode = 'full', method = 'auto')
corr = corr/max(corr)
corr_l = corr.tolist()
max_corr_index = corr_l.index(max(corr_l))
print(max_corr_index)


lag = np.array(np.linspace(-np.size(corr)/2.0, np.size(corr)/2.0, num = np.size(corr)))/125.0
delay = abs(lag[max_corr_index])
print(f'Delay is: {delay} sec')
plt.figure(2)
plt.plot(lag, corr)
plt.xlabel('2N')
plt.ylabel('cross-correlation')
plt.title('cross-correlation of x_0 and x', fontsize = 20)
plt.grid()
plt.legend()
plt.show()


























# disp_data(t, y_0, y, 'y', 'm')
# disp_data(t, z_0, z, 'z', 'm')
# disp_data(t, phi_0, phi, 'phi', 'rad')
# disp_data(t, th_0, th, 'th', 'rad')
# disp_data(t, psi_0, psi, 'psi', 'rad')

