#!/usr/bin/env python

import json
import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Point
import tf
import time
import numpy
from math import pi

class ur_tcp_pose_publisher():

    def __init__(self):
        
        rospy.init_node('ur_tcp_pose_publisher', anonymous=False)
        self.scene_master_ur_tcp_publisher = rospy.Publisher('ur_tcp_pose', Point , queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.tcp_rate = rospy.Rate(125)

        self.get_actual_tcp_pose()

        rospy.spin()
    
    def get_actual_tcp_pose(self):
        while not rospy.is_shutdown():
            try:
                #(self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/ee_link', rospy.Time(0))
                (self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/ee_link', rospy.Time(0))

                self.tcp_x = self.trans[0]
                self.tcp_y = self.trans[1]
                self.tcp_z = self.trans[2]

                self.scene_master_ur_tcp_publisher.publish(self.tcp_x, self.tcp_y, self.tcp_z)

                self.tcp_rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
if __name__ == '__main__':
    try:
        ur_tcp_pose_publisher()
    except rospy.ROSInterruptException:
        pass
