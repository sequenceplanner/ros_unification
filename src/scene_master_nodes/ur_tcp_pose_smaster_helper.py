#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR TCP Pose Scene Master Helper
    # V.0.2.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point
from unification_roscontrol.msg import URTCP
import numpy
import tf
import time

class ur_tcp_pose_smaster_helper():

    def __init__(self):
        
        rospy.init_node('ur_tcp_pose_smaster_helper', anonymous=False)
        self.ur_tcp_publisher = rospy.Publisher('unification_roscontrol/ur_tcp_pose_helper_to_smaster', URTCP, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.tcp_rate = rospy.Rate(100)

        self.x = 0
        self.y = 0
        self.z = 0
        self.r = 0
        self.p = 0
        self.w = 0

        self.main()


    def main(self):

        self.tcp = URTCP()

        while not rospy.is_shutdown():
         
            try:
                (self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/tool0', rospy.Time(0))

                quaternion = (
                    self.rot[0],
                    self.rot[1],
                    self.rot[2],
                    self.rot[3])
                euler = tf.transformations.euler_from_quaternion(quaternion)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]

                self.x = self.trans[0]
                self.y = self.trans[1]
                self.z = self.trans[2]
                self.r = roll
                self.p = pitch
                self.w = yaw

                URTCP.x = float(self.x)
                URTCP.y = float(self.y)
                URTCP.z = float(self.z)
                URTCP.r = float(self.r)
                URTCP.p = float(self.p)
                URTCP.w = float(self.w)

                self.ur_tcp_publisher.publish(self.tcp)
                self.tcp_rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        rospy.spin()


if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster_helper()
    except rospy.ROSInterruptException:
        pass
