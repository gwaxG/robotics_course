#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import tf

# Definition of class
class Estimation_Node:
    def __init__(self, node_name):
        self.nname = node_name 	#Giving a name for the ROS node

        rospy.init_node(self.nname, anonymous=True) #ROS node initialization

	self.num_of_plane_points = 100 # This is the total number of points used to estimate a 3D plane

	self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

	self.plane_points = {"red":[], "green":[], "blue":[]}

	self.feature_pose = Transform() # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

	self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations
	
	point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback, queue_size=None) # ROS topic subscription

	self.br = tf.TransformBroadcaster()

	rospy.spin() # Initiate the ROS loop

    def empty_points(self):
	self.plane_points["red"] = []
	self.plane_points["green"] = []
	self.plane_points["blue"] = []
	return

    def estimate_pose_callback(self, pointcloud_msg):
	#rospy.loginfo("Received PointCloud2 message. Reading data...")
	point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))

	for point in point_list:
	    rgb = struct.unpack('BBBB', struct.pack('f', point[3]))

	    if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If redish point, concatenate it
		self.plane_points["red"] += [[point[0], point[1], point[2]]]
	    elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If greenish point, concatenate it
		self.plane_points["green"] += [[point[0], point[1], point[2]]]
	    elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If blueish point, concatenate it
		self.plane_points["blue"] += [[point[0], point[1], point[2]]]

	# key will be one of "red", "green" and "blue". val will be the corresponding points' list
	for key, val in self.plane_points.items():
	    if len(val) < self.num_of_plane_points:
		rospy.loginfo("Not enough of points for the %s plane. Will not estimate 6DOF pose.", key)
		return

	# key will be one of "red", "green" and "blue". val will be the corresponding points' list
	for key, val in self.plane_points.items():
	    # Randomly select self.num_of_plane_points of points per plane. Attention: random selection implies that a single point might be selected multiple times
	    p = [val[int(random.random()*(len(val)-1))] for i in range(0, self.num_of_plane_points)]

	    # Least-squares plane estimation
	    # Each row of the H matrix should have the form [x, z, 1]
	    h_mat = [[point[0], point[2], 1] for point in p]

	    y_observ = [[point[1]] for point in p]

	    self.plane_params[key] = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(h_mat), h_mat)), np.transpose(h_mat)), y_observ)
	    normal_vector_norm = np.sqrt(self.plane_params[key][0][0]**2 + 1 + self.plane_params[key][1][0]**2)
	    self.plane_params[key] = [float(self.plane_params[key][0]/normal_vector_norm), -1.0/normal_vector_norm, float(self.plane_params[key][1]/normal_vector_norm), float(self.plane_params[key][2]/normal_vector_norm)]

	# Verify orthogonality of 3D planes
	if abs(np.dot(self.plane_params["green"][:3], self.plane_params["red"][:3])) >= 0.01 or abs(np.dot(self.plane_params["blue"][:3], self.plane_params["red"][:3])) >= 0.01 or abs(np.dot(self.plane_params["green"][:3], self.plane_params["blue"][:3])) >= 0.01:

	    rospy.loginfo("Not all planes are orthogonal to each other. Will not estimate 6DOF pose.")
	    # Empty points
            self.empty_points()
	    return

	# Feature detection
	# Solve 3x3 linear system of equations, given by the three intersecting planes
	self.linear_solution = np.linalg.solve([self.plane_params["red"][:3], self.plane_params["green"][:3], self.plane_params["blue"][:3]], [-self.plane_params["red"][3], -self.plane_params["green"][3], -self.plane_params["blue"][3]])

	# Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
	corrected_blue = np.cross(self.plane_params["red"][:3], self.plane_params["green"][:3])
	corrected_blue /= np.linalg.norm(corrected_blue)

	# Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis) 
	corrected_green = np.cross(corrected_blue, self.plane_params["red"][:3])
	corrected_green /= np.linalg.norm(corrected_green)
	
	# Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
	# rotation_matrix = np.transpose([self.plane_params["red"][:3], corrected_green, corrected_blue])
	rotation_matrix = np.transpose([self.plane_params["red"][:3], self.plane_params["green"][:3], self.plane_params["blue"][:3]])
	roll, pitch, yaw =  tf.transformations.euler_from_matrix(rotation_matrix)

	self.feature_pose.translation = Vector3(self.linear_solution[0], self.linear_solution[1], self.linear_solution[2])

	self.feature_pose.rotation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	
	rospy.loginfo("6DOF feature has been calculated and published as a TF named corner_6dof_pose")
	self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame") 

	# Empty points
        self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')
