#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

'''
Hints:
    Aruco markers:
        package://scripts/detect.py
    Histogram Backprojection
        https://docs.opencv.org/3.4/dc/df6/tutorial_py_histogram_backprojection.html
    Meanshift & Camshift:
        https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_meanshift/py_meanshift.html
'''

class Node:

    def __init__(self):
        '''
        Initialization of synchronized subscription to topics.
        '''
        self.mode = rospy.get_param("mode")
        rospy.init_node('track_n_move')
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
        ts.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.msg_vel = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        # image processing variables
        self.roi_hist = None
        self.term_crit = None
        self.track_window = None
        self.first_time = True
        self.N = 10
        self.cnt = self.N
        # creating aruco_dict with 5x5 bits with max 250 ids..so ids ranges from 0-249
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        rospy.spin()

    def detect_aruco(self, img):
        '''

        :param img: cv image
        :return: list(dict(id: list(corners),...))
        '''
        aruco_list = {}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # lists of ids and the corners beloning to each id
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(corners):
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
            return aruco_list

    def callback(self, rgb, depth):
        '''
        This callback transforms ROS image into OpenCV image, call a processing method and publishes a ROS image.
        :param data: ROS image
        '''
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)
            return None
        if self.mode == "circle":
            # Part I: red circle
            cv_image = self.process_image_circle(cv_rgb_image, cv_depth_image)
        elif self.mode == "joker":
            # Part II: a random image
            cv_image = self.process_random_image(cv_rgb_image, cv_depth_image)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def calc_hist(self, img):
        '''
        Calculates histogram backprojection.
        :param img: cv image
        :return:
        '''
        aruco_list = self.detect_aruco(img)
        dist2origin = {int((v[0] ** 2 + v[1] ** 2) ** .5): list(v) for v in list(aruco_list[0]) + list(aruco_list[34])}
        sort = sorted(dist2origin.keys())
        roi = [dist2origin[sort[3]], dist2origin[sort[4]]]
        x, y, w, h = (
            int(roi[0][0]),
            int(roi[0][1]),
            int(roi[1][0] - roi[0][0]),
            int(roi[1][1] - roi[0][1])
        )
        self.track_window = (x, y, w, h)
        hsv_roi = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

    def process_random_image(self, img, depth):
        '''
        Processing of image with a random picture

        Hint: you will need to add a source of light in Gazebo. It is on the top panel. Just add it.
        :param img: opencv image
        :reutrn img: processed opencv image
        '''
        # Detect corners of aruco markers 0 and 34 and create histogram backpropagation
        height, width, n_ch = img.shape
        if self.first_time:
            try:
                self.calc_hist(img)
            except Exception as e:
                print(e)
                return img
            self.first_time = False

        if self.cnt == 0:
            self.first_time = True
            self.cnt = self.N
        self.cnt -= 1
        # Perform tracking with meanshift or camshift
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)
        ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)

        img2 = cv2.polylines(img, [pts], True, 255, 2)
        center_x = (pts[0][0]+pts[1][0]+pts[2][0]+pts[3][0])/4
        center_y = (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) / 4

        self.calc_commands(depth, height, width, center_x, center_y)
        return img

    def calc_commands(self, depth, height, width, center_w, center_h):
        '''
        Calculates commands to the robot accordingly to detection.
        :param depth: depth image
        :param height: height of image
        :param width: width of image
        :param center_w: center of roi along X-axis
        :param center_h: cetner of roi along Y-axis
        :return:
        '''
        angle = center_w - width / 2.0
        distance = depth[center_h][center_w]
        print(distance)
        if angle == -1:
            cmds = {'linear': 0.0, 'angular': 0}
        elif angle != -1:
            ang = -angle / (width * 0.5)
            angular = ang if abs(ang) < 0.5 else ang / abs(ang) * 0.5
            linear = 0.05 if distance > 1.0 else 0.0
            cmds = {'linear': linear, 'angular':  angular}

        self.send_commands(cmds)

    def process_image_circle(self, img, depth):
        '''
        Processing of an image with a circle.
        :param img: opencv image
        :reutrn img: processed opencv image
        '''
        height, width, n_ch = img.shape
        heading_w = width / 2.0
        heading_h = height / 2.0
        # ----Circle detection----
        # 1 Detect circle using Hough transform
        img = cv2.medianBlur(img, 5)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=50, param2=30, minRadius=0, maxRadius=0)
        angle = -1
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # draw the outer circle
                cv2.circle(gray, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(gray, (i[0], i[1]), 2, (0, 0, 255), 3)
            # Estimated angle in pixels between the robot heading angle and the circle center
            circle_w = circles[0][0][0]
            circle_h = circles[0][0][1]
            angle = circle_w-heading_w
            distance = depth[circle_h][circle_w]
            self.calc_commands(depth, height, width, circle_w, circle_h)
        # Detect circle
        # Calculate angle and distance
        # Publish commands
        # self.calc_commands(depth, height, width, circle_w, circle_h)
        return gray

        
    def send_commands(self, cmds):
        '''
        :param: dictionary of velocity commands
        '''
        self.msg_vel.linear.x = cmds['linear']
        self.msg_vel.angular.z = cmds['angular']    
        self.pub_vel.publish(self.msg_vel)


if __name__ == '__main__':
    Node()
