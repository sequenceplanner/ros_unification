#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR TCP Pose Scene Master
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy
import tf
import time

HOST = "0.0.0.0"
RTE_PORT = 30003

class ur_tcp_pose_smaster():

    def __init__(self):
        
        rospy.init_node('ur_tcp_pose_smaster', anonymous=False)
        self.ur_tcp_pose_to_unidriver_publisher = rospy.Publisher('unification_roscontrol/ur_pose_smaster_to_unidriver', String, queue_size=10)
        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_tcp_pose_smaster", String, self.ur_pose_unidriver_to_ur_tcp_pose_smaster_callback)
        self.urScriptPublisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.URTestPose1TCP = [-0.348724384005, -0.4957338458, 0.741348536432, 0.0, 0.0, 4.7115]

        self.tcp_rate = rospy.Rate(10)

        self.main()

    
    def moveL(self, tcpPose, a=1.5, v=0.5, t=0):
        if len(tcpPose) == 6:
            script_str = "movel(p" + str(tcpPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            print "The tcp pose size is not correct."


    def main(self):

        while not rospy.is_shutdown():
         

            try:
                #rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_tcp_pose_smaster", String, self.ur_pose_unidriver_to_ur_tcp_pose_smaster_callback)
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


                self.tcp_x = self.trans[0]
                self.tcp_y = self.trans[1]
                self.tcp_z = self.trans[2]
                self.tcp_roll = roll
                self.tcp_pitch = pitch
                self.tcp_yaw = yaw


                if numpy.isclose(self.tcp_x, self.URTestPose1TCP[0], self.isclose_tolerance) and\
                    numpy.isclose(self.tcp_y, self.URTestPose1TCP[1], self.isclose_tolerance) and\
                    numpy.isclose(self.tcp_z, self.URTestPose1TCP[2], self.isclose_tolerance) and\
                    numpy.isclose(self.tcp_roll, self.URTestPose1TCP[3], self.isclose_tolerance) and\
                    numpy.isclose(self.tcp_pitch, self.URTestPose1TCPp[4], self.isclose_tolerance) and\
                    numpy.isclose(self.tcp_yaw, self.URTestPose1TCPp[5], self.isclose_tolerance):
                    self.ur_tcp_pose_state = 'URTestPose1TCP'
                
                else:
                    self.ur_tcp_pose_state = 'unknown'


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.tcp_rate.sleep()
        rospy.spin()


    def ur_pose_unidriver_to_ur_tcp_pose_smaster_callback(self, tcp_cmd):
        self.go_to_tcp = tcp_cmd.data
        print "asdf1"

        if self.go_to_tcp == "URTestPose1TCP":
            print "asdf2"
            self.URTestPose1TCPPose()


    def URTestPose1TCPPose(self):
        self.moveL(self.URTestPose1TCP, a=1.5, v=5, t=3)
        rospy.sleep(3.5)

            
        
if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster()
    except rospy.ROSInterruptException:
        pass
