#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Carlos Gil Camacho
    # UR TCP Pose Scene Master Helper
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point
from emats_ur_driver.msg import TCPState
import threading
from unification_roscontrol.msg import URTCP
import tf
import time
import math
import numpy as np


class ur_tcp_pose_smaster_helper():
        
    def __init__(self):
   
        rospy.init_node('ur_tcp_pose_smaster_helper', anonymous=False)
        self.ur_tcp_publisher = rospy.Publisher('unification_roscontrol/ur_tcp_pose_helper_to_smaster', URTCP, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.tcp_rate = rospy.Rate(100)

        self.x = 0
        self.y = 0
        self.z = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0

        self.main()


    def main(self):

        self.tcp = URTCP()

        while not rospy.is_shutdown():
            
            try:
                (self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

                #print self.trans
                #print self.rot

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

                yawMatrix = np.matrix([
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1]
                ])

                pitchMatrix = np.matrix([
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)]
                ])

                rollMatrix = np.matrix([
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)]
                ])

                R = yawMatrix * pitchMatrix * rollMatrix

                theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
                multi = 1 / (2 * math.sin(theta))

                rx = multi * (R[2, 1] - R[1, 2]) * theta
                ry = multi * (R[0, 2] - R[2, 0]) * theta
                rz = multi * (R[1, 0] - R[0, 1]) * theta

                R=[rx, ry, rz]

                #print R

                URTCP.x = self.x
                URTCP.y = self.y
                URTCP.z = self.z
                URTCP.rx = rx
                URTCP.ry = ry
                URTCP.rz = rz

                self.ur_tcp_publisher.publish(self.tcp)
                self.tcp_rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        rospy.spin()


    def radToRotVec(self, r, p, y):

        
        return R


if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster_helper()
    except rospy.ROSInterruptException:
        pass
