#!/usr/bin/env python

"""Demo of flying the ARDrone.  Altitude is adjusted based on ambient
light levels.

Author: Nathan Sprague
Version: 4/4/2014

"""
import rospy
import math

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class FlyNode(object):
    
    def __init__(self):

        rospy.init_node('fly')

        # CREATE SUBSCRIBERS
        rospy.Subscriber('/ardrone/image_raw', 
                         Image, self.image_callback)
        rospy.Subscriber('/ardrone/navdata', Navdata, self.nav_callback)

        # FLATTEN THE TRIM
        print "Waiting for flattrim service..."
        rospy.wait_for_service('/ardrone/flattrim')
        try:
            trim_proxy = rospy.ServiceProxy('/ardrone/flattrim', EmptySrv)
            trim_proxy()
            print "Flattened."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # CREATE PUBLISHERS
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty,
                                           queue_size=1) 
        self.land_pub = rospy.Publisher('/ardrone/land', Empty,queue_size=1) 

        self.cv_bridge = CvBridge()

        self.min_height = 500 # .5 meters
        self.max_height = 3000 # 3 meters

        self.nav_msg = None
        self.img_msg = None

        while (not rospy.is_shutdown() and 
               (self.nav_msg is None or self.img_msg is None)):
            rospy.sleep(.1)

        # It seems that the first takeoff message is ignored... 
        self.takeoff_pub.publish(Empty())
        rospy.sleep(.2)
        self.takeoff_pub.publish(Empty())

        self.main_loop()
        
        self.land_pub.publish(Empty()) # doesn't work.
        
    def main_loop(self):
        while not rospy.is_shutdown():
            # Convert the image message to a cv image object
            img = self.cv_bridge.imgmsg_to_cv2(self.img_msg)
        
            # Convert the image object to a Numpy array for
            # compatibility with cv2.
            img = np.array(img, dtype=np.uint8)

            img_mean = np.mean(img)
            height_range = self.max_height - self.min_height
            
            target_height = self.min_height + img_mean /255.0 * height_range
            print target_height, self.nav_msg.altd
            twist = Twist()

            if self.nav_msg.altd < target_height:
                twist.linear.z = .3
            else:
                twist.linear.z = -.3

            print twist.linear.z
            self.vel_pub.publish(twist)
            rospy.sleep(.01)
        
    def nav_callback(self, nav_msg):
        self.nav_msg = nav_msg

    def image_callback(self, image_msg):
        self.img_msg = image_msg
        
if __name__ == "__main__":
    FlyNode()
