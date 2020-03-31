###########################################################################
########## EXAMPLE 3D INTERPOLATION #################
'''
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D


# 3D example
total_rad = 10
z_factor = 3
noise = 0.1

num_true_pts = 200
s_true = np.linspace(0, total_rad, num_true_pts)
x_true = np.cos(s_true)
y_true = np.sin(s_true)
z_true = s_true/z_factor

num_sample_pts = 80
s_sample = np.linspace(0, total_rad, num_sample_pts)
x_sample = np.cos(s_sample) + noise * np.random.randn(num_sample_pts)
y_sample = np.sin(s_sample) + noise * np.random.randn(num_sample_pts)
z_sample = s_sample/z_factor + noise * np.random.randn(num_sample_pts)

tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s=2)
x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
u_fine = np.linspace(0,1,num_true_pts)
x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

fig2 = plt.figure(2)
ax3d = fig2.add_subplot(111, projection='3d')
ax3d.plot(x_true, y_true, z_true, 'b')
ax3d.plot(x_sample, y_sample, z_sample, 'r*')
ax3d.plot(x_knots, y_knots, z_knots, 'go')
ax3d.plot(x_fine, y_fine, z_fine, 'g')
fig2.show()
plt.show()
'''
############# MODIFIED FOR ACTUAL POINTS #################

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D


def getTransformedPoints(camera_point):
	# Loading points from the numpy file instead
	#camera_point = np.load("../../../npy files/trial1.npy")
	print(camera_point)
	camera_point = np.concatenate((camera_point, np.ones((camera_point.shape[0],1))), axis=1)

	# Transforming them into robot frame:
	cam_base_transform = np.array([[ 0.11393721, -0.74960065,  0.65201011, -0.04074974 + (-0.094)],
	                               [-0.99337023, -0.07585431,  0.08638116,  0.03243686 + ( 0.0  )],
	                               [-0.0152936,  -0.65752946, -0.7532736,   0.44402816 + ( 0.066)],
	                               [ 0.,          0.,          0.,          1.        			]])

	#Possible alternative transform (If this one doesn't work well):
	'''
	cam_base_transform = np.array([[ 0., 0.73727734,  0.67559021, -0.0419618 + (-0.094)],
	                               [-1., 0.,  0.,  0.015 + ( 0.0  )],
	                               [0.,  -0.67559021, 0.73727734,   0.45126645 + ( 0.066)],
	                               [ 0.,          0.,          0.,          1.        ]])
	'''
	base_points = np.dot(cam_base_transform, camera_point.T).T[:,:-1]

	print("Transformed Points:")
	print(base_points)

	return list(base_points)


def plotPolyTrajectory(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_fine, y_fine, z_fine):

	fig = plt.figure(1)

	ax3d = fig.add_subplot(111, projection='3d')

	ax3d.plot(x_sample, y_sample, z_sample, 'r*')
	ax3d.plot(x_knots, y_knots, z_knots, 'go')
	ax3d.plot(x_fine, y_fine, z_fine, 'g')

	fig.show()
	plt.show()

	return

def getPolyTrajectory():

	#For testing
	test_points = np.array([[0.1, 0.5, 0.2], [0.4, 0.6, 0.3], [0.7, 0.7, 0.4], [0.9, 0.6, 0.5], [1.2, 0.4, 0.6], [1.1, 0.2, 0.7]])
	test_points = np.array([[0.3, 0.2, 0.1], [0.3, -0.2, 0.1], [0.3, -0.2, 0.3], [0.25, -0.1, 0.4]])

	# For transformed AR Tag Points
	test_pointsx = getTransformedPoints()

	num_intermediate_points = 20

	x_sample = test_points[:,0]
	y_sample = test_points[:,1]
	z_sample = test_points[:,2]

	tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s=2)
	x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
	u_fine = np.linspace(0,1,num_intermediate_points)
	x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

	final_points = np.concatenate(([x_fine], [y_fine], [z_fine]), axis=0).T
	#print("Polyfit Trajectory:")
	#print(final_points)

	np.save("../../../npy files/final_points.npy", final_points)

	plotPolyTrajectory(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_fine, y_fine, z_fine)

if __name__ == '__main__':
	getPolyTrajectory()