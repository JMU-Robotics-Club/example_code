#!/usr/bin/env python

"""
Simple example of subscribing to sensor messages and publishing
twist messages to the turtlebot. Object-oriented version.

Author: Nathan Sprague
Version: 2/29/2016

"""
import rospy
import math

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ApproachNode(object):
    """ Class that attempts to maintain a fixed distance from
    the obstacle ahead.

    Subscribes to: /scan
    Publishes to: /cmd_vel_mux/input/navi
    """

    TARGET = .75 # Desired distance to the wall

    def __init__(self):
        """ Set up the node, publishers and subscribers. """
        rospy.init_node('approach')

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
                                       queue_size=10)
        self.scan = None


    def scan_callback(self, scan_msg):
        """ Store the latest scan message in an instance variable. """
        self.scan = scan_msg


    def run(self):
        """ Move forward or backward to achieve the target distance. """

        twist = Twist()
        rate = rospy.Rate(10)

        # Wait until the first scan message is available
        while self.scan is None and not rospy.is_shutdown():
            rospy.sleep(.1)

        # This is the main loop.
        while not rospy.is_shutdown():

            if (math.isnan(self.scan.ranges[320]) or
                self.scan.ranges[320] < self.TARGET):
                twist.linear.x = -.1
                twist.angular.z = 0
            else:
                twist.linear.x = .1
                twist.angular.z = 0

            rospy.loginfo("Range: {:.3f}".format(self.scan.ranges[320]))

            self.vel_pub.publish(twist)
            rate.sleep()


if __name__ == "__main__":
    approach = ApproachNode()
    approach.run()
