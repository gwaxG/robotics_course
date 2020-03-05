#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

'''
histogram backprojected image and initial target location, then meanshift
'''

class Node:

    def __init__(self):
        rospy.init_node('track_n_move')
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.msg_vel = Twist()
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
        # D something here on image
        #Calculate velocities and publish it
        cmds = {'linear': 0, 'angular': 0.5}
        self.send_commands(cmds)
        return img
        
    def send_commands(self, cmds):
        '''
        :param: dictionary of velocity commands.
        '''
        self.msg_vel.linear.x = cmds['linear']
        self.msg_vel.angular.z = cmds['angular']    
        self.pub_vel.publish(self.msg_vel)
        

if __name__ == '__main__':
    Node()
