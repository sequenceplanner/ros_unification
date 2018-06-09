#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Sim Test Pose Unification Driver for the Universal Robots UR10 
    # V.0.1.0.
#----------------------------------------------------------------------------------------

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
from geometry_msgs.msg import PoseStamped
from moveit_python import *
import rospkg
import tf
import time
import numpy
from math import pi

HOST = "0.0.0.0"
RTE_PORT = 30003


class ur_pose_unidriver():

    def __init__(self):

        #Settings the options for the socket connection
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 10)
        self.socket_client.connect((HOST, RTE_PORT))
        
        rospy.init_node('ur_pose_unidriver', anonymous=False)

        self.p = PlanningSceneInterface("base")
        self.g = MoveGroupInterface("manipulator", "base")

        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.tcp_names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        
        self.ur_pose_state = '_'
        self.isclose_tolerance = 0.005

        #------------------------------------------------------------------------------------------------------------
        # Sample Hardcoded Poses
        #------------------------------------------------------------------------------------------------------------

        self.tcp_x_first = -0.20011
        self.tcp_x_inc = 0.084
        self.tcp_y_const = -0.16394
        self.tcp_z_at = 0.56053
        self.tcp_z_above = 0.46053

        self.tcp_roll = 2.4184
        self.tcp_pitch = 2.4184
        self.tcp_yaw = - 2.4184

        self.URHomeJoint = [0.0, 0.0, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.0]
        self.URAboveFirstPairJoint = [1.0034697339733611e-07, -2.3164940516101282, 2.7339940071105957, 2.724092721939087, -1.570796314870016, 0.0]


        #------------------------------------------------------------------------------------------------------------
        # Oil Filter Poses
        #------------------------------------------------------------------------------------------------------------
        self.URAboveFilter1Joint = [0.6838626265525818, -1.318165127431051, 1.6211071014404297, 1.2678542137145996, -1.570796314870016, -3.8275070826159876]
        self.URAboveFilter1Tcp = [-0.348724384005, -0.4957338458, 0.641348536432, 0.0, 0.0, 4.7115]
        self.URAtFilter1Joint = [0.6838626265525818, -1.3534911314593714, 1.4862562417984009, 1.4380313158035278, -1.570796314870016, -3.8263068834887903]
        self.URAtFilter1Tcp = [-0.348724384005, -0.4957338458, 0.741348536432, 0.0, 0.0, 4.7115]
        self.URAtFilter1TightenedJoint = [0.6838626265525818, -1.3534892241107386, 1.4862691164016724, 1.4380165338516235, -1.570796314870016, 2.464404821395874]
        self.URAtFilter1TightenedTcp = [-0.348724384005, -0.4957338458, 0.741348536432, 0.0, 0.0, 4.7115]
        self.URAboveFilter1TightenedTcp = [-0.348724384005, -0.4957338458, 0.641348536432, 0.0, 0.0, 4.7191]

        self.URAboveFilter2Joint = [0.5697967410087585, -1.1931489149676722, 1.452964425086975, 1.3109804391860962, -1.570796314870016, -3.8275070826159876]
        self.URAboveFilter2Tcp = [-0.469861872741, -0.49573377701, 0.641349997496, 0.0, 0.0, 4.5963]
        self.URAtFilter2Joint = [0.5697967410087585, -1.2223637739764612, 1.3157622814178467, 1.4773974418640137, -1.570796314870016, -3.8275070826159876]
        self.URAtFilter2Tcp = [-0.469861872741, -0.49573377701, 0.741349997496, 0.0, 0.0, 4.5963]
        self.URAtFilter2TightenedJoint = [0.5697967410087585, -1.2223637739764612, 1.3157622814178467, 1.4773974418640137, -1.570796314870016, 2.464404821395874]
        self.URAtFilter2TightenedTcp = [-0.469861884392, -0.495733784474, 0.741350022904, 0.0, 0.0, 4.6050]
        self.URAboveFilter2TightenedTcp = [-0.469861884392, -0.495733784474, 0.641350022904, 0.0, 0.0, 4.6050]

        self.URAboveFilter3Joint = [0.48505112528800964, -1.047391716633932, 1.2376282215118408, 1.3805595636367798, -1.570796314870016, -3.8275070826159876]
        self.URAboveFilter3Tcp = [-0.588972194854, -0.495733734846, 0.641350121505, 0.0, 0.0, 4.5115]
        self.URAtFilter3Joint = [0.48505112528800964, -1.0664451758014124, 1.090820550918579, 1.5464208126068115, -1.570796314870016, -3.8275070826159876]
        self.URAtFilter3Tcp = [-0.588972194854, -0.495733734846, 0.741350121505, 0.0, 0.0, 4.5115]
        self.URAtFilter3TightenedJoint = [0.48505112528800964, -1.0664451758014124, 1.0908207893371582, 1.5464204549789429, -1.570796314870016, 2.464404821395874]
        self.URAtFilter3TightenedTcp = [-0.588972182188, -0.495733728171, 0.741349971306, 0.0, 0.0, 4.5203]
        self.URAboveFilter3TightenedTcp = [-0.588972182188, -0.495733728171, 0.641349971306, 0.0, 0.0, 4.5203]


        #------------------------------------------------------------------------------------------------------------
        # Tool Attaching Poses
        #------------------------------------------------------------------------------------------------------------
        self.URHomeTSToolTcp = [-0.9042, -0.10394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAttachTSToolTcp = [-1.0542, -0.10394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]

        self.URHomeOFToolTcp = [-0.7042, -0.50394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAttachOFToolTcp = [-0.8842, -0.50394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]

        self.URHomeLFToolTcp = [-0.7042, 0.30394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAttachLFToolTcp = [-0.8542, 0.30394, -0.3293, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]


        #------------------------------------------------------------------------------------------------------------
        # LF Moving Poses
        #------------------------------------------------------------------------------------------------------------
        self.URAboveLFMiRTcp = [0.446985870325, -0.175739280487, 0.49835611426, 0.0, 0.0, 0.0]
        self.URAboveLFMiRJoint = [2.418654441833496, 0.41698744893074036, -2.3895848433123987, 0.4018012583255768, 1.5707964897155762, -0.8478577772723597]
        self.URAtLFMiRTcp = [0.446985870325, -0.175739280487, 0.704666543397, 0.0, 0.0, 0.0]
        self.URAtLFMiRJoint = [2.418654441833496, 0.026409711688756943, -2.1005237738238733, 0.5033179521560669, 1.5707964897155762, -0.8478577772723597]
        self.URMidpointLFTcp = [-0.173405782431, -0.309330836372, 0.498356072846, 0.0, 0.0, 5.6648]
        self.URMidpointLFJoint = [0.5792755484580994, 0.21647852659225464, -2.5586586634265345, 0.7713837623596191, 1.5707961320877075, 0.3731209933757782]
        self.URAbovePlacingLFJoint = [0.0, 0.22762444615364075, -1.0287192503558558, -0.7697017828570765, 1.5707961320877075, 0.0]
        self.URAbovePlacingLFTcp = [-1.11018911749, -0.163941017953, 0.492372902973, 0.0, 0.0, 4.7124]
        self.URPlacingLFJoint = [0.0, 0.06413673609495163, -0.9000023047076624, -0.7349308172809046, 1.5707961320877075, 0.0]
        self.URPlacingLFTcp = [-1.11018929157, -0.163941212064, 0.604851541829, 0.0, 0.0, 4.7124]


        #------------------------------------------------------------------------------------------------------------
        # LF Bolting Poses
        #------------------------------------------------------------------------------------------------------------
        self.URAboveFirstPairTcp = [self.tcp_x_first, self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtFirstPairTcp = [self.tcp_x_first, self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveSecondPairTcp = [self.tcp_x_first -(1*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtSecondPairTcp = [self.tcp_x_first -(1*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveThirdPairTcp = [self.tcp_x_first -(2*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtThirdPairTcp = [self.tcp_x_first -(2*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveFourthPairTcp = [self.tcp_x_first -(3*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtFourthPairTcp = [self.tcp_x_first -(3*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveFifthPairTcp = [self.tcp_x_first -(4*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtFifthPairTcp = [self.tcp_x_first -(4*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveSixthPairTcp = [self.tcp_x_first -(5*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtSixthPairTcp = [self.tcp_x_first -(5*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveSeventhPairTcp = [self.tcp_x_first -(6*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtSeventhPairTcp = [self.tcp_x_first -(6*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveEighthPairTcp = [self.tcp_x_first -(7*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtEighthPairTcp = [self.tcp_x_first -(7*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveNinthPairTcp = [self.tcp_x_first -(8*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtNinthPairTcp = [self.tcp_x_first -(8*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveTenthPairTcp = [self.tcp_x_first -(9*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtTenthPairTcp = [self.tcp_x_first -(9*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveEleventhPairTcp = [self.tcp_x_first -(10*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtEleventhPairTcp = [self.tcp_x_first -(10*self.tcp_x_inc), self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAboveTwelvthPairTcp = [self.tcp_x_first -(11*self.tcp_x_inc) + 0.01, self.tcp_y_const, self.tcp_z_above, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]
        self.URAtTwelvthPairTcp = [self.tcp_x_first -(11*self.tcp_x_inc) + 0.01, self.tcp_y_const, self.tcp_z_at, self.tcp_roll, self.tcp_pitch, self.tcp_yaw]

        rospy.Subscriber("/joint_states", JointState, self.jointCallback)
        rospy.Subscriber("/bridge_to_driver", String, self.sp_to_driver_callback)
        rospy.Subscriber("/ur_tcp_pose", Point, self.tcpCallback)

        self.URScriptPublisher = rospy.Publisher("ur_driver/URScript", String, queue_size=200)
        self.ur_pose_state_publisher = rospy.Publisher('ur_pose_unistate', String, queue_size=10)
        self.message_ack_publisher = rospy.Publisher('driver_to_bridge', String, queue_size=10)
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()

    def moveL(self, tcpPose, a=1.5, v=0.5, t=0):
        if len(tcpPose) == 6:
            script_str = "movel(p" + str(tcpPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) + ")"
            self.sendURScript(script_str)
        else:
            print "The tcp pose size is not correct."
        
    def moveJ(self, jointPose, a=1.5, v=0.5, t=0, r=0):
        if len(jointPose) == 6:
            script_str = "movej(" + str(jointPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) +  ", r=" + str(r) + ")"
            self.sendURScript(script_str)
        else:
            print "The joint pose size is not correct."

    def sendURScript(self, script):
        self.socket_client.send(script + "\n")

    
    #------------------------------------------------------------------------------------------------------------
    # Moveit Scene Interface
    #------------------------------------------------------------------------------------------------------------
    def addWall(self):
        self.p.addBox("Wall", 2, 0.5, 2, -1, 0.5, 1)
    
    def addTwinSpinTool(self):
        self.p.addBox("TSTool", 0.1, 0.3, 0.4, -1.1042, -0.10394, -0.3293)
        self.p.addCylinder("atlas1", 0.1, 0.01, -1.1042, -0.10394 + 0.135, -0.3293 + 0.25)
        self.p.addCylinder("atlas2", 0.1, 0.01, -1.1042, -0.10394 - 0.135, -0.3293 + 0.25)
     
    def addOilFilterTool(self):
        self.p.addBox("OFTool", 0.07, 0.07, 0.07, -0.9042, -0.50394, -0.3293)
        rospy.sleep(2)

    def addLFTool(self):
        self.p.addBox("LFTool", 0.2, 0.07, 0.07, -0.9542, 0.30394, -0.3293)
        rospy.sleep(2)

    def attachLFTool(self):
        self.p.attachBox("LFTool", 0.07, 0.07, 0.2 , 0, 0, 0.1, "tool0")
        rospy.sleep(1)
        self.p.attachBox("LFTool", 0.07, 0.07, 0.2 , 0, 0, 0.1, "tool0")
        rospy.sleep(3)

    def attachTwinSpinTool(self):
        self.p.attachBox("TSTool", 0.3, 0.4, 0.1, 0, 0, 0.05, "tool0")
        self.p.attachBox("atlas1", 0.02, 0.1, 0.02, 0.135, -0.25, 0.05, "tool0")
        self.p.attachBox("atlas2", 0.02, 0.1, 0.02, -0.135, -0.25, 0.05, "tool0")
        rospy.sleep(3)

    def URAttachOFTool(self):
        self.p.attachBox("OFTool", 0.07, 0.07, 0.07, 0.0, 0.0, 0.035, "tool0")
        rospy.sleep(1)
        self.p.attachBox("OFTool", 0.07, 0.07, 0.07, 0.0, 0.0, 0.035, "tool0")
        rospy.sleep(3)

    def detachTwinSpinTool(self):
        self.p.removeAttachedObject("TSTool")
        self.p.removeAttachedObject("atlas1")
        self.p.removeAttachedObject("atlas2")
        rospy.sleep(2)

    def detachOFTool(self):
        self.p.removeAttachedObject("OFTool")

    def detachLFTool(self):
        self.p.removeAttachedObject("LFTool")

    #------------------------------------------------------------------------------------------------------------
    # Sample move methods LF tool
    #------------------------------------------------------------------------------------------------------------

    
    def URToAboveLFMiRL(self):
        self.moveL(self.URAboveLFMiRTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)
   
    def URToAboveLFMiRJ(self):
        self.moveJ(self.URAboveLFMiRJoint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    def URToLFMiRL(self):
        self.moveL(self.URAtLFMiRTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    #self.URAtLFMiRJoint 
    def URToMidpointLFL(self):
        self.moveL(self.URMidpointLFTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    #self.URMidpointLFJoint
    def URToAbovePlacingLFL(self):
        self.moveL(self.URAbovePlacingLFTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)
    
    #self.URPlacingLFJoint
    #self.URPlacingLFTcp 
    def URToPlacingLFL(self):
        self.moveL(self.URPlacingLFTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    def URToLFHomeL(self):
        self.moveL(self.URHomeLFToolTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    def URToAttachLFL(self):
        self.moveL(self.URAttachLFToolTcp,  a=1.5, v=5, t=2)
        rospy.sleep(2.5)



    #------------------------------------------------------------------------------------------------------------
    # Sample move methods OF tool
    #------------------------------------------------------------------------------------------------------------
    def URToAboveOF1J(self):
        self.moveJ(self.URAboveFilter1Joint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    def URToAtOF1L(self):
        self.moveL(self.URAtFilter1Tcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)

    def URToTightenOF1J(self):
        self.moveJ(self.URAtFilter1TightenedJoint, a=1.5, v=5, t=5, r=0)
        rospy.sleep(5.5)

    def URToAboveOF1L(self):
        self.moveL(self.URAboveFilter1TightenedTcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)

    
    def URToAboveOF2J(self):
        self.moveJ(self.URAboveFilter2Joint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    def URToAtOF2L(self):
        self.moveL(self.URAtFilter2Tcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)

    def URToTightenOF2J(self):
        self.moveJ(self.URAtFilter2TightenedJoint, a=1.5, v=5, t=5, r=0)
        rospy.sleep(5.5)

    def URToAboveOF2L(self):
        self.moveL(self.URAboveFilter2TightenedTcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)


    def URToAboveOF3J(self):
        self.moveJ(self.URAboveFilter3Joint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    def URToAtOF3L(self):
        self.moveL(self.URAtFilter3Tcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)

    def URToTightenOF3J(self):
        self.moveJ(self.URAtFilter3TightenedJoint, a=1.5, v=5, t=5, r=0)
        rospy.sleep(5.5)

    def URToAboveOF3L(self):
        self.moveL(self.URAboveFilter3TightenedTcp, a=1.5, v=5, t=3)
        rospy.sleep(3.5)



    #------------------------------------------------------------------------------------------------------------
    # Sample move methods Atlas LF
    #------------------------------------------------------------------------------------------------------------
    def URToHomeJ(self):
        self.moveJ(self.URHomeJoint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    def URToAttachTSL(self):
        self.moveL(self.URAttachTSToolTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    def URToAttachOFL(self):
        self.moveL(self.URAttachOFToolTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)
    
    def URToTSHomeL(self):
        self.moveL(self.URHomeTSToolTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)

    def URToOFHomeL(self):
        self.moveL(self.URHomeOFToolTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.5)
    
    def URToAboveFirstPairJ(self):
        self.moveJ(self.URAboveFirstPairJoint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)
    
    def URToAboveFirstPairL(self):
        self.moveL(self.URAboveFirstPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToFirstPair(self):
        self.moveL(self.URAtFirstPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveSecondPair(self):
        self.moveL(self.URAboveSecondPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToSecondPair(self):
        self.moveL(self.URAtSecondPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveThirdPair(self):
        self.moveL(self.URAboveThirdPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToThirdPair(self):
        self.moveL(self.URAtThirdPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveFourthPair(self):
        self.moveL(self.URAboveFourthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToFourthPair(self):
        self.moveL(self.URAtFourthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveFifthPair(self):
        self.moveL(self.URAboveFifthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToFifthPair(self):
        self.moveL(self.URAtFifthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveSixthPair(self):
        self.moveL(self.URAboveSixthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToSixthPair(self):
        self.moveL(self.URAtSixthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveSeventhPair(self):
        self.moveL(self.URAboveSeventhPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToSeventhPair(self):
        self.moveL(self.URAtSeventhPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveEighthPair(self):
        self.moveL(self.URAboveEighthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToEighthPair(self):
        self.moveL(self.URAtEighthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveNinthPair(self):
        self.moveL(self.URAboveNinthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToNinthPair(self):
        self.moveL(self.URAtNinthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveTenthPair(self):
        self.moveL(self.URAboveTenthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToTenthPair(self):
        self.moveL(self.URAtTenthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveEleventhPair(self):
        self.moveL(self.URAboveEleventhPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToEleventhPair(self):
        self.moveL(self.URAtEleventhPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)

    def URToAboveTwelvthPair(self):
        self.moveL(self.URAboveTwelvthPairTcp, a=1.5, v=5, t=1)
        rospy.sleep(1.2)

    def URToTwelvthPair(self):
        self.moveL(self.URAtTwelvthPairTcp, a=1.5, v=5, t=2)
        rospy.sleep(2.2)
    
    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        #self.p.clear()
        rospy.sleep(1)

        rospy.sleep(3)
        self.addLFTool()
        #self.addWall()
        self.addTwinSpinTool()
        self.addOilFilterTool()
        #self.addLF()
        self.g.moveToJointPosition(self.joint_names, self.URHomeJoint, tolerance = 0.01)
        self.URToLFHomeL()
        self.URToAttachLFL()
        self.attachLFTool()
        #rospy.sleep(2)
        self.URToLFHomeL()
        self.URToAboveLFMiRJ()
        self.URToLFMiRL()
        self.URToAboveLFMiRL()
        self.URToMidpointLFL()
        self.URToAbovePlacingLFL()
        self.URToPlacingLFL()
        self.URToAbovePlacingLFL()
        self.URToHomeJ()
        self.URToLFHomeL()
        self.URToAttachLFL()
        self.detachLFTool()
        self.URToLFHomeL()
        self.URToHomeJ()


        #self.URToHomeJ()
        self.URToAttachTSL()
        self.attachTwinSpinTool()
        self.URToTSHomeL()
        self.URToAboveFirstPairJ()
        self.URToFirstPair()
        self.URToAboveFirstPairL()
        self.URToAboveSecondPair()
        self.URToSecondPair()
        self.URToAboveSecondPair()
        self.URToAboveThirdPair()
        self.URToThirdPair()
        self.URToAboveThirdPair()
        self.URToAboveFourthPair()
        self.URToFourthPair()
        self.URToAboveFourthPair()
        self.URToAboveFifthPair()
        self.URToFifthPair()
        self.URToAboveFifthPair()
        self.URToAboveSixthPair()
        self.URToSixthPair()
        self.URToAboveSixthPair()
        self.URToAboveSeventhPair()
        self.URToSeventhPair()
        self.URToAboveSeventhPair()
        self.URToAboveEighthPair()
        self.URToEighthPair()
        self.URToAboveEighthPair()
        self.URToAboveNinthPair()
        self.URToNinthPair()
        self.URToAboveNinthPair()
        self.URToAboveTenthPair()
        self.URToTenthPair()
        self.URToAboveTenthPair()
        self.URToAboveEleventhPair()
        self.URToEleventhPair()
        self.URToAboveEleventhPair()
        self.URToAboveTwelvthPair()
        self.URToTwelvthPair()
        self.URToAboveTwelvthPair()
        self.URToHomeJ()
        self.URToAttachTSL()
        self.detachTwinSpinTool()
        self.URToTSHomeL()

        self.URToOFHomeL()
        self.URToAttachOFL()

        self.URAttachOFTool()
        self.URToOFHomeL()
        self.URToAboveOF1J()
        self.URToAtOF1L()
        self.URToTightenOF1J()
        self.URToAboveOF1L()

        self.URToAboveOF2J()
        self.URToAtOF2L()
        self.URToTightenOF2J()
        self.URToAboveOF2L()

        self.URToAboveOF3J()
        self.URToAtOF3L()
        self.URToTightenOF3J()
        self.URToAboveOF3L()

        self.URToHomeJ()
        self.URToOFHomeL()
        self.URToAttachOFL()
        self.detachOFTool()
        self.URToOFHomeL()
        self.URToHomeJ()



        def main_callback():
            while not rospy.is_shutdown():
                self.ur_pose_state_publisher.publish(self.ur_pose_state)
                self.main_rate.sleep()
        t = threading.Thread(target=main_callback)
        t.daemon = True
        t.start()
    
        rospy.spin()

    #----------------------------------------------------------------------------------------
    # Main callback for kafka communication
    #----------------------------------------------------------------------------------------
	
	# add method calls here based on topic from the kafka side /bridge_to_driver

    def sp_to_driver_callback(self, cmd_data):
        self.sp_to_driver = json.loads(cmd_data.data)
        if self.sp_to_driver['receiver'] == "ur_pose_unidriver":
            if self.sp_to_driver['refPos'] == "URHome":
                self.message_ack_publisher.publish("ur_pose_unidriver got msg: URHomePos")
                self.URToHomePos()
            elif self.sp_to_driver['refPos'] == "URPickPos":
                self.message_ack_publisher.publish("ur_pose_unidriver got msg: URPickPos")
                self.URToPickPos()
            elif self.sp_to_driver['refPos'] == "URRandomPos":
                self.message_ack_publisher.publish("ur_pose_unidriver got msg: URRandomPos")
                self.URToRandomPos()
            else:
                pass
        else:
            pass

    #----------------------------------------------------------------------------------------------------------------
    # jointCallback
    #----------------------------------------------------------------------------------------------------------------
    def jointCallback(self, joint):
        pass

    '''
        #if self.processing == False and\
        if numpy.isclose(joint.position[0], self.URHomePosJ[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.URHomePosJ[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.URHomePosJ[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.URHomePosJ[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.URHomePosJ[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.URHomePosJ[5], self.isclose_tolerance):
            self.ur_pose_state = 'URHomePos'

        #elif self.processing == False and\
        elif numpy.isclose(joint.position[0], self.URPickPosJ[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.URPickPosJ[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.URPickPosJ[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.URPickPosJ[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.URPickPosJ[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.URPickPosJ[5], self.isclose_tolerance):
            self.ur_pose_state = 'URPickPos'


        elif (joint.velocity[0] or joint.velocity[1] or joint.velocity[2] or joint.velocity[3] or joint.velocity[4] or joint.velocity[5] != 0):
            self.ur_pose_state = "URMoving"
            
        else:
            self.ur_pose_state = 'UR10StateERROR'

    '''

    #----------------------------------------------------------------------------------------------------------------
    # tcpCallback
    #----------------------------------------------------------------------------------------------------------------
    def tcpCallback(self, tcp):
        self.x = tcp.x
        self.y = tcp.y
        self.z = tcp.z

if __name__ == '__main__':
    try:
        ur_pose_unidriver()
    except rospy.ROSInterruptException:
        pass
