#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
        self.nname = node_name         #Giving a name for the ROS node

        rospy.init_node(self.nname, anonymous=True) #ROS node initialization

        self.num_of_plane_points = 100 # This sets a minimum number of points used to estimate a 3D plane

        self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

        self.plane_points = {"red":[], "green":[], "blue":[]}

        self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

        self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations
        
        point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback) # ROS topic subscription

        self.br = tf.TransformBroadcaster()

        rospy.spin() # Initiate the ROS loop

    def empty_points(self):
        self.plane_points["red"] = []
        self.plane_points["green"] = []
        self.plane_points["blue"] = []

    def estimate_pose_callback(self, pointcloud_msg):
        #print 'Received PointCloud2 message. Reading data...'
        point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))

        #print 'Retrieving coordinates and colors...'
        for point in point_list:
            rgb = struct.unpack('BBBB', struct.pack('f', point[3]))

            if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If dominant red point, concatenate it
                self.plane_points["red"] += [[point[0], point[1], point[2]]]
            elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If dominant green point, concatenate it                
                self.plane_points["green"] += [[point[0], point[1], point[2]]]
            elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If dominant blue point, concatenate it
                self.plane_points["blue"] += [[point[0], point[1], point[2]]]

        # Test if there are sufficient points for each plane
        ### Enter your code ###
            # print(len(self.plane_points['red']), len(self.plane_points['blue']), len(self.plane_points['green']))
        for key, val in self.plane_points.items():
            if len(val) < self.num_of_plane_points:
                rospy.loginfo("Not enough of points for the %s plane. Will not estimate 6DOF pose.", key)
                return
        # Estimate the plane equation for each colored point set using Least Squares algorithm
        sub_set = lambda val :[val[np.random.choice(len(val))] for i in range(int(len(val)*0.1))]
        for key, val in self.plane_points.items():
            self.plane_points = {k: sub_set(v) for k, v in self.plane_points.items()}
        ### Enter your code ###
        print("#####")
        for key, val in self.plane_points.items():
            # Randomly select self.num_of_plane_points of points per plane. Attention: random selection implies that a single point might be selected multiple times
            p = [val[int(random.random()*(len(val)-1))] for i in range(0, self.num_of_plane_points)]

            # Least-squares plane estimation
            # Each row of the H matrix should have the form [x, z, 1]
            h_mat = [[point[0], point[2], 1] for point in p]
            y_observ = [[point[1]] for point in p]
            self.plane_params[key] = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(h_mat), h_mat)), np.transpose(h_mat)), y_observ)
            normal_vector_norm = np.sqrt(self.plane_params[key][0][0]**2 + 1 + self.plane_params[key][1][0]**2)
            
            print(key, self.plane_params[key][0], -1, self.plane_params[key][1], self.plane_params[key][2])
            self.plane_params[key] = [
                float(self.plane_params[key][0]/normal_vector_norm), 
                -1.0/normal_vector_norm, 
                float(self.plane_params[key][1]/normal_vector_norm), 
                float(self.plane_params[key][2]/normal_vector_norm)
                ]

        # Verify that each pair of 3D planes are approximately orthogonal to each other
        ### Enter your code ###
        r = self.plane_params['red']
        b = self.plane_params['blue']
        g = self.plane_params['green']
        print('----')
        print('r normal', r[0], r[1], r[2])
        print('g normal', g[0], g[1], g[2])
        print('b normal', b[0], b[1], b[2])
        rg = r[0]*g[0] + r[1]*g[1]+ r[2]*g[2]
        rb = r[0]*b[0] + r[1]*b[1]+ r[2]*b[2]
        bg = b[0]*g[0] + b[1]*g[1]+ b[2]*g[2]
        if rg > 0.1 and rb > 0.1 and bg > 0.1:
            rospy.loginfo('Planes are not orthogonal')
            self.empty_points()
            return None
        # Feature detection
        # Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection
        ### Enter your code ###
        self.linear_solution = np.linalg.solve([self.plane_params["red"][:3], self.plane_params["green"][:3], self.plane_params["blue"][:3]], [-self.plane_params["red"][3], -self.plane_params["green"][3], -self.plane_params["blue"][3]])
        # Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
        corrected_blue = self.plane_params["blue"][:3]# np.cross(self.plane_params["red"][:3], self.plane_params["green"][:3])
        corrected_blue /= np.linalg.norm(corrected_blue)

        # Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis) 
        corrected_green = np.cross(corrected_blue, self.plane_params["red"][:3])
        corrected_green /= np.linalg.norm(corrected_green)

        # Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
	
        R = np.transpose([self.plane_params["red"][:3], corrected_green, corrected_blue])

        # Obtain the corresponding euler angles from the previous 3x3 rotation matrix
        ### Enter your code ###
        roll, pitch, yaw =  tf.transformations.euler_from_matrix(R)
        # Set the translation part of the 6DOF pose 'self.feature_pose'
        ### Enter your code ###
        self.feature_pose.translation.x = self.linear_solution[0]
        self.feature_pose.translation.y = self.linear_solution[1]
        self.feature_pose.translation.z = self.linear_solution[2]
        # self.feature_pose.translation = Vector3(self.linear_solution[0], self.linear_solution[1], self.linear_solution[2])
        # Set the rotation part of the 6DOF pose 'self.feature_pose'
        ### Enter your code ###
        self.feature_pose.rotation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # Publish the transform using the data stored in the 'self.feature_pose'
        rospy.loginfo("6DOF feature has been calculated and published")
        self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame") 

        # Empty points
        self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')
