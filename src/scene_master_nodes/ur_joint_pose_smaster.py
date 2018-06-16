#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Joint Pose Scene Master
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy
import time



class ur_joint_pose_smaster():

    def __init__(self):
        
        rospy.init_node('ur_joint_pose_smaster', anonymous=False)

        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_joint_pose_smaster", String, self.ur_pose_unidriver_to_ur_joint_pose_smaster_callback)
        rospy.Subscriber("/joint_states", JointState, self.jointCallback)

        self.ur_joint_pose_to_unidriver_publisher = rospy.Publisher('unification_roscontrol/ur_joint_pose_smaster_to_unidriver', String, queue_size=10)
        self.urScriptPublisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
    
        self.isclose_tolerance = 0.005

        # Unification JOINT Poses
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.HomeJOINTPose = [0.0, 0.0, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.0]

        self.joint_rate = rospy.Rate(10)

        self.main()



    def moveJ(self, jointPose, a=1.5, v=0.5, t=0, r=0):

        if len(jointPose) == 6:
            script_str = "movej(" + str(jointPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) +  ", r=" + str(r) + ")"
            self.sendURScript(script_str)
        else:
            print "The joint pose size is not correct."



    def main(self):

        while not rospy.is_shutdown():

        self.joint_rate.sleep()
        rospy.spin()


    
    def ur_pose_unidriver_to_ur_joint_pose_smaster_callback(self, joint_cmd):
        self.go_to_joint_pose = joint_cmd.data

        if self.go_to_joint_pose == "HomeJOINT":
            self.HomeJOINT()

        elif self.go_to_joint_pose == "PreAttachAtlasFarJOINT":
            self.PreAttachAtlasFarJOINT()

        elif self.go_to_joint_pose == "PreAttachLFToolFarJOINT":
            self.PreAttachLFToolFarJOINT()

        elif self.go_to_joint_pose == "PreAttachOFToolFarJOINT":
            self.PreAttachOFToolFarJOINT()
        
        elif self.go_to_joint_pose == "PreFindEngineJOINT":
            self.PreFindEngineJOINT()

        elif self.go_to_joint_pose == "FindEngineRightJOINT":
            self.FindEngineRightJOINT()

        elif self.go_to_joint_pose == "FindEngineLeftJOINT":
            self.FindEngineLeftJOINT()

        elif self.go_to_joint_pose == "FindEngineMidJOINT":
            self.FindEngineMidJOINT()

        else:
            pass


    def HomeJOINT(self):
        self.moveJ(self.HomeJOINTPose, a=1.5, v=5, t=3, r=0)

# in progress...
    