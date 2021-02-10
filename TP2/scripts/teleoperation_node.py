#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

# Definition of class


class Teleoperation_Node:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        msg = Twist()
        moves = {
            'forward': rospy.get_param('forward'),
            'backward': rospy.get_param('backward'),
            'left': rospy.get_param('left'),
            'right': rospy.get_param('right'),
            'increase': rospy.get_param('increase'),
            'decrease': rospy.get_param('decrease'),
            'exit': rospy.get_param('exit'),
        }
        vel = 0.2
        DELTA = 0.1
        while True:
            k = self.getKey()
            if k == moves['forward']:
                msg.linear.x = vel
                msg.angular.z = 0
            elif k == moves['backward']:
                msg.linear.x = -vel
                msg.angular.z = 0
            elif k == moves['left']:
                msg.linear.x = 0
                msg.angular.z = vel*3
            elif k == moves['right']:
                msg.linear.x = 0
                msg.angular.z = -vel*3
            elif k == moves['increase']:
                if vel <= 2.0:
                    vel += vel*0.1
            elif k == moves['decrease']:
                if vel > 0.0:
                    vel -= vel*0.1
            elif k == moves['exit']:
                exit()
            publisher.publish(msg)

    # This function reads a single keyboard character from the terminal and returns this character
    def getKey(self):
    # Back-up default terminal settings
        settings = termios.tcgetattr(sys.stdin)

        tty.setraw(sys.stdin.fileno()) # Setting stdio terminal to raw (no need for pressing enter)
        key = sys.stdin.read(1) # Read 1 character 
    
    # Restore default terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
if __name__ == '__main__':
    Teleoperation_Node('tele_node_2')

