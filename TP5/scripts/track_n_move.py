#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv2 import aruco
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

'''
histogram backprojected image (initial target location is needed)
https://docs.opencv.org/3.4/dc/df6/tutorial_py_histogram_backprojection.html
meanshift
https://docs.opencv.org/3.4/d7/d00/tutorial_meanshift.html
'''

class Node:

    def __init__(self):
        rospy.init_node('track_n_move')
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.msg_vel = Twist()
        self.listener = tf.TransformListener()
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        rospy.spin()
        
    def callback_image(self, data):
        '''
        This callback transforms ROS image into OpenCV image, call a processing method and publishes a ROS image.
        :param data: ROS image
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return None
        cv_image = self.process_image(cv_image)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def process_image(self, img):
        '''
        Put your code here in this method.  
        You have to define where the aruco marker is located.
        Then, you have to track it without additional detections.                
        Draw a cross over the marker, rotate the robot to its center 
        and come closer up to one meter.
        Hint: marker's height and width are 1 m.
        :param img: opencv image
        :reutrn img: processed opencv image
        '''
        # 1 Detect rectangles (Hough transform)
        # 2 Filter detected rectangles by color so that they have only white and black colors inside
        # 3 Perform histogram backpropagation
        # 4 Track a needed rectangle using meanshift
        # 5 Calculate robot velocities and publish it
        # Aruco
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
        # Commands
        detected = False
        try:
            (trans, rot) = self.listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
            norm = np.sqrt(trans[0]**2+trans[1]**2)
            n = [trans[0]/norm, trans[1]/norm]
            detected = True
        except Exception as e:
            trans = [0,0,0]
            n = [0, 0]    
        angle = np.arcsin(n[1]) # if abs(np.arcsin(n[1])) < 0.1 else 0
        print(trans)
        if detected:
            cmds = {'linear': 0.05, 'angular': angle/2}
        else:
            cmds = {'linear': 0.0, 'angular': 0}
        self.send_commands(cmds)
        return frame_markers
        
    def send_commands(self, cmds):
        '''
        :param: dictionary of velocity commands
        '''
        self.msg_vel.linear.x = cmds['linear']
        self.msg_vel.angular.z = cmds['angular']    
        self.pub_vel.publish(self.msg_vel)
        

if __name__ == '__main__':
    Node()
