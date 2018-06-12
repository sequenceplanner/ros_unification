#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Pose Unification Driver for the Universal Robots UR10
    # Java-ROS plugin for SP removes need for Kafka
    # Robot state uploaded to SP in tcp state (x, y, z - Point) because after calibration we can know 
    # the needed tcp states but not the joint states
    # V.0.3.0.
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
# UR10 pose state names and descritions
#----------------------------------------------------------------------------------------
    # W - waiting
    # P - planned movement (MoveIt: OMPL or IPS)
    # J - joint linear movement (URScript)
    # L - tcp linear movement (URScript)
    # M - processing

    # URUnknownW		        # Beginning, no idea where the robot is
    # URUnknownToHomeP	        # Moving on a Moveit planned path 
    # URHomeW			        # Waiting in home position

    # URHomeToPreMeas1J         # Joint move from home position to a laser pre measuring pose 1
    # URPreMeas1W               # Waiting in the pre measuring pose 1
    # URPreMeas1M               # Turning on the laser and recording the readings
    # URPreMeas1ToPostMeas1L    # Linear move from pre measuring pose 1 to the post measuring pose 1 while laser is on
    # URPostMeas1W              # Waiting in the post measuring pose 1
    # URPostMeas1M              # Turning off the laser
    # URPostMeas1ToPreMeas2J    # Joint move from post measuring pose 1 to pre measuring pose 2

    # URPreMeas2W               # Waiting in the pre measuring pose 2
    # URPreMeas2M               # Turning on the laser and recording the readings
    # URPreMeas2ToPostMeas2L    # Linear move from pre measuring pose 2 to the post measuring pose 2 while laser is on
    # URPostMeas2W              # Waiting in the post measuring pose 2
    # URPostMeas2M              # Turning off the laser
    # URPostMeas2ToPreMeas3J    # Joint move from post measuring pose 2 to pre measuring pose 3

    # URPreMeas3W               # Waiting in the pre measuring pose 3
    # URPreMeas3M               # Turning on the laser and recording the readings
    # URPreMeas3ToPostMeas3L    # Linear move from pre measuring pose 3 to the post measuring pose 3 while laser is on
    # URPostMeas3W              # Waiting in the post measuring pose 3
    # URPostMeas3M              # Turning off the laser
    # URPostMeas3ToPreMeas4J    # Joint move from post measuring pose 3 to pre measuring pose 4

    # URPreMeas4W               # Waiting in the pre measuring pose 4
    # URPreMeas4M               # Turning on the laser and recording the readings
    # URPreMeas4ToPostMeas4L    # Linear move from pre measuring pose 4 to the post measuring pose 4 while laser is on
    # URPostMeas4W              # Waiting in the post measuring pose 4
    # URPostMeas4M              # Turning off the laser
    # URPostMeas4ToHomeJ        # Joint move from post measuring pose 4 to home position

    # URHomeToFrontLFToolJ	    # Joint move from home position to in front of the LF tool
    # URHomeToFrontATToolJ	    # Joint move from home position to in front of the Atlas tool
    # URHomeToFrontOFToolJ	    # Joint move from home position to in front of the Oil-Filter tool
    # URFrontLFToolW		    # Waiting in front of the LF tool, compressed air engages robot's RSP
    # URFrontATToolW		    # Waiting in front of the Atlas tool, compressed air engages robot's RSP
    # URFrontOFToolW		    # Waiting in front of the Oil-Filter tool, compressed air engages robot's RSP
    # URFrontLFToolM		    # Compressed air engages robot's RSP
    # URFrontATToolM		    # Compressed air engages robot's RSP
    # URFrontOFToolM		    # Compressed air engages robot's RSP
    # URFrontLFToolToLFToolL	# Linear move to join the robot's RSP part and the LF tool's RSP part
    # URFrontATToolToATToolL	# Linear move to join the robot's RSP part and the Atlas tool's RSP part
    # URFrontOFToolToOFToolL	# Linear move to join the robot's RSP part and the Oil-Filter tool's RSP part
    # URLFToolW		            # Waiting in the Robot-LFTool RSP joined pose
    # URATToolW		            # Waiting in the Robot-AtlasTool RSP joined pose
    # UROFToolW		            # Waiting in the Robot-OFTool RSP joined pose
    # URLFToolM		            # Joining/Leaving the Robot with the LFTool, disengaging/engaging compressed air
    # URATToolM		            # Joining/Leaving the Robot with the AtlasTool, disengaging/engaging compressed air
    # UROFToolM		            # Joining/Leaving the Robot with the OFTool, disengaging/engaging compressed air
    # URLFToolToFrontLFToolL	# Linear move to in front of the LF tool
    # URLFToolToFrontATToolL	# Linear move to in front of the Atlas tool
    # URLFToolToFrontOFToolL	# Linear move to in front of the OF tool
    # URFrontLFToolToMdp1J	    # Joint move from in fron of the LF tool to Midpoint 1, robot has the LF tool
    # URMdp1W			        # Waiting in Midpoint 1 with the LF tool
    # URMdp1ToAbLFJ		        # Joint move from Midpoint 1 to above the Ladderframe
    # URAboveLFW		        # Waiting above the Ladderframe
    # URAboveLFM		        # Compressed air opens the LF tool
    # URAboveLFToGrabLFL	    # Linear move to grab the Ladderframe
    # URGrabLFW		            # Waiting in the pose wherefrom the Ladderframe can be grabbed
    # URGrabLFM		            # Compressed air closes the LF tool, ladderframe grabbed
    # URTransportLFM		    # Robot in a state where the operator uses it to transport the Ladderframe
    			                # It is kind of a freedrive mode...
    			                # After this is done, the robot is in the URUnknownW state
    			                # URUnknownToHomeP Planned movement needed

    # URFrontATtoolToAbLFJ	    # Joint move with the atlas tool to above Ladderframe on the engine
    # URAbLFToAb1of3L           # Linear move to above the firts pair of bolts out of three
    # URAb1of3W		            # Waiting above the firts pair of bolts out of three
    # URAb1of3M		            # Atlas tool engages spinning
    # URAb1of3To1of3L		    # Linear move to tighten the first pair of bolts out of three
    # UR1of3M			        # Atlas tool disengages spinning
    # UR1of3W			        # Waiting in the 1of3 bolts pose
    # UR1of3ToAb1of3L		    # Linear move to above the first pair out of three
    # URAb1of3ToAb2of3L	        # Linear move from above the first pair of three to above the second pair of three
    
    # URAb2of3W		            # Waiting above the second pair of bolts out of three
    # URAb2of3M		            # Atlas tool engages spinning
    # URAb2of3To2of3L		    # Linear move to tighten the second pair of bolts out of three
    # UR2of3M			        # Atlas tool disengages spinning
    # UR2of3W			        # Waiting in the 2of3 bolts pose
    # UR2of3ToAb2of3L		    # Linear move to above the second pair out of three
    # URAb2of3ToAb3of3L	        # Linear move from above the second pair of three to above the third pair of three
    
    # URAb3of3W		            # Waiting above the third pair of bolts out of three
    # URAb3of3M		            # Atlas tool engages spinning
    # URAb3of3To3of3L		    # Linear move to tighten the third pair of bolts out of three
    # UR3of3M			        # Atlas tool disengages spinning
    # UR3of3W			        # Waiting in the 3of3 bolts pose
    # UR3of3ToAb3of3L		    # Linear move to above the third pair out of three
    # URAb3of3ToHomeJ		    # Joint move from above the third pair of three to Home position

    # URHomeToAb1of9J		    # Joint move from home pose to above the first pair of bolts out of nine
    # URAb1of9W		            # Waiting above the first pair of bolts out of nine
    # URAb1of9M		            # Atlas tool engages spinning
    # URAb1of9To1of9L		    # Linear move to tighten the first pair of bolts out of nine
    # UR1of9M			        # Atlas tool disengages spinning
    # UR1of9W			        # Waiting in the 1of9 bolts pose
    # UR1of9ToAb1of9L		    # Linear move to above the first pair out of nine
    # URAb1of9ToAb2of9L	        # Linear move from above the first pair of nine to above the second pair of nine

    # URAb2of9W		            # Waiting above the second pair of bolts out of nine
    # URAb2of9M		            # Atlas tool engages spinning
    # URAb2of9To2of9L		    # Linear move to tighten the second pair of bolts out of nine
    # UR2of9M			        # Atlas tool disengages spinning
    # UR2of9W			        # Waiting in the 2of9 bolts pose
    # UR2of9ToAb2of9L		    # Linear move to above the second pair out of nine
    # URAb2of9ToAb3of9L	        # Linear move from above the first pair of nine to above the second pair of nine
    # 
    # URAb3of9W		            # Waiting above the third pair of bolts out of nine
    # URAb3of9M		            # Atlas tool engages spinning
    # URAb3of9To3of9L		    # Linear move to tighten the third pair of bolts out of nine
    # UR3of9M			        # Atlas tool disengages spinning
    # UR3of9W			        # Waiting in the 3of9 bolts pose
    # UR3of9ToAb3of9L		    # Linear move to above the third pair out of nine
    # URAb3of9ToAb4of9L	        # Linear move from above the third pair of nine to above the fourth pair of nine
    # 
    # URAb4of9W		            # Waiting above the fourth pair of bolts out of nine
    # URAb4of9M		            # Atlas tool engages spinning
    # URAb4of9To4of9L		    # Linear move to tighten the fourth pair of bolts out of nine
    # UR4of9M			        # Atlas tool disengages spinning
    # UR4of9W			        # Waiting in the 4of9 bolts pose
    # UR4of9ToAb4of9L		    # Linear move to above the fourth pair out of nine
    # URAb4of9ToAb5of9L	        # Linear move from above the fourth pair of nine to above the fifth pair of nine
    # 
    # URAb5of9W		            # Waiting above the fifth pair of bolts out of nine
    # URAb5of9M		            # Atlas tool engages spinning
    # URAb5of9To5of9L		    # Linear move to tighten the fifth pair of bolts out of nine
    # UR5of9M			        # Atlas tool disengages spinning
    # UR5of9W			        # Waiting in the 5of9 bolts pose
    # UR5of9ToAb5of9L		    # Linear move to above the fifth pair out of nine
    # URAb5of9ToAb6of9L	        # Linear move from above the fifth pair of nine to above the sixth pair of nine

    # URAb6of9W		            # Waiting above the first pair of bolts out of nine
    # URAb6of9M		            # Atlas tool engages spinning
    # URAb6of9To6of9L		    # Linear move to tighten the sixth pair of bolts out of nine
    # UR6of9M			        # Atlas tool disengages spinning
    # UR6of9W			        # Waiting in the 6of9 bolts pose
    # UR6of9ToAb6of9L		    # Linear move to above the sixth pair out of nine
    # URAb6of9ToAb7of9L	        # Linear move from above the sixth pair of nine to above the seventh pair of nine

    # URAb7of9W		            # Waiting above the seventh pair of bolts out of nine
    # URAb7of9M		            # Atlas tool engages spinning
    # URAb7of9To7of9L		    # Linear move to tighten the seventh pair of bolts out of nine
    # UR7of9M			        # Atlas tool disengages spinning
    # UR7of9W			        # Waiting in the 7of9 bolts pose
    # UR7of9ToAb7of9L		    # Linear move to above the seventh pair out of nine
    # URAb7of9ToAb8of9L	        # Linear move from above the seventh pair of nine to above the eighth pair of nine

    # URAb8of9W		            # Waiting above the eighth pair of bolts out of nine
    # URAb8of9M		            # Atlas tool engages spinning
    # URAb8of9To8of9L		    # Linear move to tighten the eighth pair of bolts out of nine
    # UR8of9M			        # Atlas tool disengages spinning
    # UR8of9W			        # Waiting in the 8of9 bolts pose
    # UR8of9ToAb8of9L		    # Linear move to above the eighth pair out of nine
    # URAb8of9ToAb9of9L	        # Linear move from above the eighth pair of nine to above the ninth pair of nine

    # URAb9of9W		            # Waiting above the ninth pair of bolts out of nine
    # URAb9of9M		            # Atlas tool engages spinning
    # URAb9of9To9of9L		    # Linear move to tighten the ninth pair of bolts out of nine
    # UR9of9M			        # Atlas tool disengages spinning
    # UR9of9W			        # Waiting in the 9of9 bolts pose
    # UR9of9ToAb9of9L		    # Linear move to above the ninth pair out of nine
    # URAb9of9ToHomeJ		    # Joint move from above the ninth pair of nine to Home position

    # URFrontOFToolToAbOFJ	    # Joint move from in front of the OF tool to above the oil filters
    # URAbOFtoAbOF1L            # Linear move from above the oil filters to abovr the oil filter 1
    # URAbOF1W		            # Waiting above the first oil filter
    # URTightenOF1M		        # Engaging algorithm to tighten Oil Filter 1 (karen)
     			                # After tightening, robot should go above the Oil Fliter 1
    # URAbOF1ToAbOF2L		    # Linear move from above the Oil Filter 1 to above the Oil Filter 2

    # URAbOF2W		            # Waiting above the second oil filter
    # URTightenOF2M		        # Engaging algorithm to tighten Oil Filter 2 (karen)
    			                # After tightening, robot should go above the Oil Fliter 2
    # URAbOF2ToAbOF3L		    # Linear move from above the Oil Filter 2 to above the Oil Filter 3

    # URAbOF3W		            # Waiting above the third oil filter
    # URTightenOF3M		        # Engaging algorithm to tighten Oil Filter 3 (karen)
     			                # After tightening, robot should go above the Oil Fliter 3
    # URAbOF1ToHomeJ		    # Joint move from above the Oil Filter 3 to Home position
#----------------------------------------------------------------------------------------

import sys
import json
import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_commander import PlanningSceneInterface as PlanningSceneInterface2
from moveit_commander import roscpp_initialize, roscpp_shutdown
from unification_roscontrol.msg import URPose1
from unification_roscontrol.msg import URPose2
#from moveit_msgs.msg import RobotState, Grasp
import moveit_msgs.msg
#from pyassimp import pyassimp
import rospkg
import tf
import time
import numpy
from math import pi

HOST = "0.0.0.0"
RTE_PORT = 30003

class ur_pose_unidriver():

    def __init__(self):
        
        roscpp_initialize(sys.argv)

        # Initial values
        #self.ur_pose_state_shouldPlan = False
        self.ur_pose_state_refPos = "_"
        self.ur_pose_state_actPos = "_"
        self.ur_pose_refPos_prev_cmd = "_"
        self.ur_pose_state_executing = False
        #self.ur_pose_state_planning = False

        

        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 10)
        self.socket_client.connect((HOST, RTE_PORT))
        
        rospy.init_node('ur_pose_unidriver', anonymous=False)
        
        #self.p = PlanningSceneInterface("base")
        #self.m = PlanningSceneInterface("base_footprint")
        self.scene = PlanningSceneInterface2()
        self.g = MoveGroupInterface("manipulator", "base")

        self.URPosePublisher = rospy.Publisher("/unification/ur_pose_unidriver/state", URPose1, queue_size=10)
        self.mircall = rospy.Publisher("/sp_to_fake_mir_unidriver", String, queue_size=10)
        


        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.tcp_names = ['x', 'y', 'z', 'rx', 'ry', 'rz']

        #self.ur_pose_state = '_'
        self.isclose_tolerance = 0.1

        rospy.Subscriber("/joint_states", JointState, self.jointCallback)
        rospy.Subscriber("/unification/ur_pose_unidriver/cmd", URPose2, self.ur_pose_unistate_callback)
        #rospy.Subscriber("/ur_tcp_pose", Point, self.tcpCallback)

        self.URScriptPublisher = rospy.Publisher("ur_driver/URScript", String, queue_size=200)
        #self.ur_pose_state_publisher = rospy.Publisher('ur_pose_unistate', String, queue_size=10)
        #self.message_ack_publisher = rospy.Publisher('driver_to_bridge', String, queue_size=10)

        # Sample Hardcoded Poses for testing purposes
        self.URHomeJoint = [0.0, 0.0, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.0]
        self.URAboveFilter1Joint = [0.6838626265525818, -1.318165127431051, 1.6211071014404297, 1.2678542137145996, -1.570796314870016, -3.8275070826159876]
        self.URAboveFilter1Tcp = [-0.348724384005, -0.4957338458, 0.641348536432, 0.0, 0.0, 4.7115]
        self.URAtFilter1Joint = [0.6838626265525818, -1.3534911314593714, 1.4862562417984009, 1.4380313158035278, -1.570796314870016, -3.8263068834887903]
        self.URAtFilter1Tcp = [-0.348724384005, -0.4957338458, 0.741348536432, 0.0, 0.0, 4.7115]
        self.URAtFilter1TightenedJoint = [0.6838626265525818, -1.3534892241107386, 1.4862691164016724, 1.4380165338516235, -1.570796314870016, 2.464404821395874]
        self.URAtFilter1TightenedTcp = [-0.348724384005, -0.4957338458, 0.741348536432, 0.0, 0.0, 4.7115]
        self.URAboveFilter1TightenedTcp = [-0.348724384005, -0.4957338458, 0.641348536432, 0.0, 0.0, 4.7191]

        # Sample testing dummy values
        self.DummyJoint1 = [-1.4372690359698694, -1.5873220602618616, -1.5315807501422327, -1.6148999373065394, 1.59895920753479, 0.1324421614408493]
        self.DummyJoint2 = [-2.034539524708883, -1.3379505316363733, -1.8133795897113245, -1.5946019331561487, 1.582043170928955, -0.4647153059588831]
        self.DummyJoint3 = [-1.859448258076803, -1.1679099241839808, 0.7398859858512878, -1.5353425184832972, 1.5146180391311646, -0.6263755003558558]
        self.DummyJoint4 = [-2.388958994542257, -1.5455425421344202, 1.5693951845169067, -1.9648574034320276, 1.715777039527893, -0.1352313200580042]

        #------------------------------------------------------------------------------------------------------------
        # Sample Hardcoded Poses
        #------------------------------------------------------------------------------------------------------------

        self.tcp_x_first = -0.19511
        self.tcp_x_inc = 0.084
        self.tcp_y_const = -0.16394
        self.tcp_z_at = 0.56053
        self.tcp_z_above = 0.46053

        self.tcp_roll = 2.4184
        self.tcp_pitch = 2.4184
        self.tcp_yaw = - 2.4184

        self.URHomeJoint = [0.0, 0.0, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.0]
        self.URAboveFirstPairJoint = [0.0, -2.3164940516101282, 2.7339940071105957, 2.724092721939087, -1.570796314870016, 0.0]
        self.URAboveTwelvthPairJoint = [0.0, -0.6917160193072718, 0.9985163807868958, 2.834791421890259, -1.570796314870016, 0.0]

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
        self.URAtLFMiRTcp = [0.446985870325, -0.175739280487, 0.734666543397, 0.0, 0.0, 0.0]
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
                            

        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()

    # -------------------------------------------------------------------------------
    # transformations
    # -------------------------------------------------------------------------------
        # From Euler to Quaternion
        #quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        ##type(pose) = geometry_msgs.msg.Pose
        #pose.orientation.x = quaternion[0]
        #pose.orientation.y = quaternion[1]
        #pose.orientation.z = quaternion[2]
        #pose.orientation.w = quaternion[3]

        # From Quaternion to Euler
        ##type(pose) = geometry_msgs.msg.Pose
        #quaternion = (
        #    pose.orientation.x,
        #    pose.orientation.y,
        #    pose.orientation.z,
        #    pose.orientation.w)
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]


    #------------------------------------------------------------------------------------------------------------
    # Hardcoded moves via Socket
    #------------------------------------------------------------------------------------------------------------
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
    # MoveIt Move Group Interface Moves
    #------------------------------------------------------------------------------------------------------------
    def moveP(self, jointPose):
        self.g.moveToJointPosition(self.joint_names, jointPose, tolerance = 0.01)

    #def moveP(self, jointPose):
    #    self.g.go(jointPose)

    #------------------------------------------------------------------------------------------------------------
    # MoveIt Scene Interface for simulation purposes
    #------------------------------------------------------------------------------------------------------------
    def addWall(self):
        self.p.addBox("Wall", 2, 0.5, 2, -1, 0.5, 1)

    #------------------------------------------------------------------------------------------------------------
    # Atlas Tool add, attach, remove
    #------------------------------------------------------------------------------------------------------------
    def addAtlasTool(self):
        pos1 = PoseStamped()
    
        pos1.header.frame_id = "base"
        pos1.pose.position.x = -1.1042
        pos1.pose.position.y = -0.10394
        pos1.pose.position.z = -0.3293
        pos1.pose.orientation.x = 0
        pos1.pose.orientation.y = 0
        pos1.pose.orientation.z = 0
        pos1.pose.orientation.w = 1
        self.scene.add_box("TSTool", pos1, size = (0.1, 0.3, 0.4))
        rospy.sleep(1)

        pos2 = PoseStamped()
    
        pos2.header.frame_id = "base"
        pos2.pose.position.x = -1.1042
        pos2.pose.position.y = -0.10394 + 0.135
        pos2.pose.position.z = -0.3293 + 0.25
        pos2.pose.orientation.x = 0
        pos2.pose.orientation.y = 0
        pos2.pose.orientation.z = 0
        pos2.pose.orientation.w = 1
        self.scene.add_box("atlas1", pos2, size = (0.01, 0.01, 0.1))
        rospy.sleep(1)

        pos3 = PoseStamped()
    
        pos3.header.frame_id = "base"
        pos3.pose.position.x = -1.1042
        pos3.pose.position.y = -0.10394 - 0.135
        pos3.pose.position.z = -0.3293 + 0.25
        pos3.pose.orientation.x = 0
        pos3.pose.orientation.y = 0
        pos3.pose.orientation.z = 0
        pos3.pose.orientation.w = 1
        self.scene.add_box("atlas2", pos3, size = (0.01, 0.01, 0.1))
        rospy.sleep(1)

    def attachAtlasTool(self):
        self.scene.attach_box("tool0", "TSTool", size = (0.1, 0.3, 0.4))
        self.scene.attach_box("tool0", "atlas1", size = (0.01, 0.01, 0.1))
        self.scene.attach_box("tool0", "atlas2", size = (0.01, 0.01, 0.1))
        rospy.sleep(1)

    def detachAtlasTool(self):
        self.scene.remove_attached_object("tool0", "TSTool")
        self.scene.remove_attached_object("tool0", "atlas1")
        self.scene.remove_attached_object("tool0", "atlas2")

    #------------------------------------------------------------------------------------------------------------
    # Ladderframe tool add, attach, remove
    #------------------------------------------------------------------------------------------------------------
    def addLFTool(self):

        pos1 = PoseStamped()
    
        pos1.header.frame_id = "base"
        pos1.pose.position.x = -0.9542
        pos1.pose.position.y = 0.30394
        pos1.pose.position.z = -0.3293
        pos1.pose.orientation.x = 0
        pos1.pose.orientation.y = 0
        pos1.pose.orientation.z = 0
        pos1.pose.orientation.w = 1
        self.scene.add_box("LFTool", pos1, size = (0.2, 0.07, 0.07))
        rospy.sleep(1)

    def attachLFTool(self):
        self.scene.attach_box("tool0", "LFTool", size = (0.2, 0.07, 0.07))
        rospy.sleep(1)

    def detachLFTool(self):
        self.scene.remove_attached_object("tool0", "LFTool")

    #------------------------------------------------------------------------------------------------------------
    # Oilfilter tool add, attach, remove
    #------------------------------------------------------------------------------------------------------------
    def addOilFilterTool(self):

        pos1 = PoseStamped()
    
        pos1.header.frame_id = "base"
        pos1.pose.position.x = -0.9042
        pos1.pose.position.y = -0.50394
        pos1.pose.position.z = -0.3293
        pos1.pose.orientation.x = 0
        pos1.pose.orientation.y = 0
        pos1.pose.orientation.z = 0
        pos1.pose.orientation.w = 1
        self.scene.add_box("OFTool", pos1, size = (0.07, 0.07, 0.07))
        rospy.sleep(1)

    def attachOilFilterTool(self):
        self.scene.attach_box("tool0", "OFTool", size = (0.07, 0.07, 0.07))
        rospy.sleep(1)

    def detachOilFilterTool(self):
        self.scene.remove_attached_object("tool0", "OFTool")

    #------------------------------------------------------------------------------------------------------------
    # Laddeframe add, attach, remove
    #------------------------------------------------------------------------------------------------------------
    def addLFMesh(self):
        pos = PoseStamped()
        qpos = Pose()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.5707)
        #type(pose) = geometry_msgs.msg.Pose
        qpos.orientation.x = quaternion[0]
        qpos.orientation.y = quaternion[1]
        qpos.orientation.z = quaternion[2]
        qpos.orientation.w = quaternion[3]
    
        # mir <origin xyz="-0.5 0.2 0.5" rpy="0.0 0.0 0.0" />
        # lf to mir <origin xyz="-5.65 3.33 -0.572" rpy="0 0 -1.5707" />
        pos.header.frame_id = "world"
        pos.pose.position.x = -6.15 
        pos.pose.position.y = 3.53
        pos.pose.position.z = -0.1
        pos.pose.orientation.x = qpos.orientation.x
        pos.pose.orientation.y = qpos.orientation.y
        pos.pose.orientation.z = qpos.orientation.z
        pos.pose.orientation.w = qpos.orientation.w

        self.scene.add_mesh("LF", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/LF.stl", size = (0.01, 0.01, 0.01))

    def attachLF(self):
        self.scene.attach_mesh("tool0", "LF")
        rospy.sleep(3)

    def detachLF(self):
        self.scene.remove_attached_object("tool0", "LF")

    #------------------------------------------------------------------------------------------------------------
    # AGV add
    #------------------------------------------------------------------------------------------------------------
    def addAGVMesh(self):
        pos = PoseStamped()
    
        pos.header.frame_id = "world" #-2.7 -5.874 -0.1
        pos.pose.position.x = -2.7
        pos.pose.position.y = -5.874
        pos.pose.position.z = -0.1
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1

        self.scene.add_mesh("AGV", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/AGV.stl", size = (0.01, 0.01, 0.01))

    
    #------------------------------------------------------------------------------------------------------------
    # Engine add
    #------------------------------------------------------------------------------------------------------------
    def addReducedEngine(self):
        pos = PoseStamped()
        qpos = Pose()

        quaternion = tf.transformations.quaternion_from_euler(-1.5707, 0, 1.5707)
        #type(pose) = geometry_msgs.msg.Pose
        qpos.orientation.x = quaternion[0]
        qpos.orientation.y = quaternion[1]
        qpos.orientation.z = quaternion[2]
        qpos.orientation.w = quaternion[3]
    
        # agv to world <origin xyz="-2.7 -5.874 -0.1" 
        # engine to agv <origin xyz="3.42 5.65 0.8"
        pos.header.frame_id = "world"
        pos.pose.position.x = 3.42 - 2.7 + 0.647
        pos.pose.position.y = 5.65 - 5.874
        pos.pose.position.z = 0.8 + 0.05 + 0.132 # - 0.1
        pos.pose.orientation.x = qpos.orientation.x
        pos.pose.orientation.y = qpos.orientation.y
        pos.pose.orientation.z = qpos.orientation.z
        pos.pose.orientation.w = qpos.orientation.w
        self.scene.add_mesh("redengine", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/EngineReduced.stl", size = (0.01, 0.01, 0.01))
        rospy.sleep(1)

    #------------------------------------------------------------------------------------------------------------
    # Oilfilters to MiR add
    #------------------------------------------------------------------------------------------------------------
    def addCylinder1ToMiR(self):
        #def clbk():
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = - 0.7
        pos.pose.position.y = -0.122
        pos.pose.position.z = 0.98 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        rospy.sleep(2)
        #t = threading.Thread(target=clbk)
        #t.daemon = True
        #t.start()

    def addCylinder2ToMiR(self):
        #def clbk():
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = - 0.7
        pos.pose.position.y = -0.122 + 0.2
        pos.pose.position.z = 0.98 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        rospy.sleep(2)
        #t = threading.Thread(target=clbk)
        #t.daemon = True
        #t.start()

    def addCylinder3ToMiR(self):
        #def clbk():
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = - 0.7
        pos.pose.position.y = -0.122 + 0.2 + 0.2
        pos.pose.position.z = 0.98 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        #self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.006, 0.006, 0.0095))
        rospy.sleep(2)
        #t = threading.Thread(target=clbk)
        #t.daemon = True
        #t.start()

    #------------------------------------------------------------------------------------------------------------
    # Frame to scene add
    #------------------------------------------------------------------------------------------------------------
    def addFrame(self):
        pos = PoseStamped()
    
        pos.header.frame_id = "world" # 2.95 5.74 2.12
        pos.pose.position.x = -3
        pos.pose.position.y = -5.8
        pos.pose.position.z = -0.15
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("Frame", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Frame.stl", size = (0.01, 0.01, 0.01))
        rospy.sleep(1)


    #------------------------------------------------------------------------------------------------------------
    # Oilfilters to Engine add
    #------------------------------------------------------------------------------------------------------------
    def addCylinder1ToEngine(self):
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = 0.37
        pos.pose.position.y = -0.540
        pos.pose.position.z = 0.92 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF1", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        rospy.sleep(2)

    def addCylinder2ToEngine(self):
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = 0.49
        pos.pose.position.y = -0.540
        pos.pose.position.z = 0.92 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF2", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        rospy.sleep(2)

    def addCylinder3ToEngine(self):
        pos = PoseStamped()
    
        pos.header.frame_id = "world"
        pos.pose.position.x = 0.61
        pos.pose.position.y = -0.540
        pos.pose.position.z = 0.92 #-0.672
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        self.scene.add_mesh("OF3", pos, "/home/endre_dell/catkin_ws_crslab/src/unification_roscontrol/meshes/Cylinder.stl", size = (0.0052, 0.0052, 0.0085))
        rospy.sleep(2)

    

    


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
    
    def URToAboveTwelvthPairJ(self):
        self.moveJ(self.URAboveTwelvthPairJoint, a=1.5, v=5, t=3, r=0)
        rospy.sleep(3.5)

    #------------------------------------------------------------------------------------------------------------
    # Sample move methods
    #-----------------------------------------------------------------------------------------------------------
    def URToDummyPose1(self):
        self.moveJ(self.DummyJoint1, a=1.5, v=5, t=3, r=0)
        #rospy.sleep(3.5)
    
    def URToDummyPose2(self):
        self.moveJ(self.DummyJoint2, a=1.5, v=5, t=3, r=0)
        #rospy.sleep(3.5)

    def URToDummyPose3(self):
        self.moveJ(self.DummyJoint3, a=1.5, v=5, t=3, r=0)
        #rospy.sleep(3.5)

    def URToDummyPose4(self):
        self.moveP(self.DummyJoint4)
        #rospy.sleep(3.5)

    def URUnknownToHomeP(self):
        self.moveP(self.URHomeJoint)

    def URHomeToPreMeas1J(self):
        self.moveJ(self.URPreMeas1Joint, a=1.5, v=5, t=5, r=0)
        rospy.sleep(5.5)

   
    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        
        self.scene.remove_world_object()
        self.scene.remove_attached_object("tool0")
        rospy.sleep(2)
        self.addAtlasTool()
        rospy.sleep(2)
        self.addOilFilterTool()
        rospy.sleep(2)
        self.addLFTool()
        rospy.sleep(2)
        self.addAGVMesh()
        rospy.sleep(2)
        self.addFrame()
        rospy.sleep(2)
        self.addReducedEngine()
        rospy.sleep(20)
        self.mircall.publish("move")
        rospy.sleep(6)
        self.addLFMesh()
        rospy.sleep(2)
        
        self.addCylinder1ToMiR()
        rospy.sleep(2)
        self.addCylinder2ToMiR()
        rospy.sleep(2)
        self.addCylinder3ToMiR()
        rospy.sleep(2)
        
        self.URUnknownToHomeP()
        rospy.sleep(2)
        self.URToLFHomeL()
        self.URToAttachLFL()
        self.attachLFTool()
        #rospy.sleep(2)
        self.URToLFHomeL()
        self.URToHomeJ()
        self.URToAboveLFMiRJ()
        self.URToLFMiRL()
        self.attachLF()
        self.URToAboveLFMiRL()
        self.URToMidpointLFL()
        self.URToAbovePlacingLFL()
        self.URToPlacingLFL()
        self.detachLF()
        self.URToAbovePlacingLFL()
        self.URUnknownToHomeP()
        rospy.sleep(1)
        self.URToHomeJ()
        self.URToLFHomeL()
        self.URToAttachLFL()
        self.detachLFTool()
        self.URToLFHomeL()
        self.URToHomeJ()
        self.URToAttachTSL()
        self.attachAtlasTool()
        self.URToTSHomeL()
        #self.URToAboveFirstPairJ()
        self.URToAboveTwelvthPairJ()
        self.URToTwelvthPair()
        self.URToAboveTwelvthPair()
        self.URToAboveEleventhPair()
        self.URToEleventhPair()
        self.URToAboveEleventhPair()
        self.URToAboveTenthPair()
        self.URToTenthPair()
        self.URToAboveTenthPair()
        self.URToHomeJ()
        rospy.sleep(5)
        self.URToAboveNinthPair()
        self.URToNinthPair()
        self.URToAboveNinthPair()
        self.URToAboveEighthPair()
        self.URToEighthPair()
        self.addCylinder1ToEngine()
        rospy.sleep(2)
        self.URToAboveEighthPair()
        self.URToAboveSeventhPair()
        self.URToSeventhPair()
        self.URToAboveSeventhPair()
        self.URToAboveSixthPair()
        self.URToSixthPair()
        self.URToAboveSixthPair()
        self.URToAboveFifthPair()
        self.URToFifthPair()
        self.addCylinder2ToEngine()
        rospy.sleep(2)
        self.URToAboveFifthPair()
        self.URToAboveFourthPair()
        self.URToFourthPair()
        self.URToAboveFourthPair()
        self.URToAboveThirdPair()
        self.URToThirdPair()
        self.URToAboveThirdPair()
        self.URToAboveSecondPair()
        self.URToSecondPair()
        self.addCylinder3ToEngine()
        rospy.sleep(2)
        self.URToAboveSecondPair()
        self.URToAboveFirstPairL()
        self.URToFirstPair()

        self.URToHomeJ()
        self.URToAttachTSL()
        self.detachAtlasTool()
        self.URToTSHomeL()
        self.URToOFHomeL()
        self.URToAttachOFL()
        self.attachOilFilterTool()
        self.URToOFHomeL()
        self.URToAboveOF1J()
        self.URToAtOF1L()
        
        #self.addCylinder1ToEngine()
        #self.addCylinder2ToEngine()
        #self.addCylinder3ToEngine()

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
        
        


        self.ur_pose_state = URPose1()
        while not rospy.is_shutdown():
            
            URPose1.shouldPlan = False
            URPose1.actPos = self.ur_pose_state_actPos
            URPose1.refPos = self.ur_pose_state_refPos
            URPose1.executing = self.ur_pose_state_executing
            #URPose.planning = self.ur_pose_state_planning
            self.URPosePublisher.publish(self.ur_pose_state)
            #print self.ur_pose_state
            #print "------------"
            #self.ur_pose_state_publisher.publish(self.ur_pose_state)
            self.main_rate.sleep()
    
        rospy.spin()

        '''
        self.scene.remove_world_object()
        self.detachRndBox()
        rospy.sleep(2)
        self.addLFMesh()
        rospy.sleep(2)
        self.addEngineMesh()
        self.URToAboveOF1J()
        rospy.sleep(6)
        self.URUnknownToHomeP()
        rospy.sleep(4)
        self.addRndBox()
        rospy.sleep(2)
        self.attachRndBox()
        #self.MiRAddLFBOX()
        #self.MiRAttachLF()
        '''

    #----------------------------------------------------------------------------------------
    # callback
    #----------------------------------------------------------------------------------------
    def ur_pose_unistate_callback(self, data):
        print "asdfasdf"
        
        self.ur_pose_refPos_cmd = data.refPos
        
        if self.ur_pose_refPos_cmd == "URDummyPose1" and self.ur_pose_state_actPos != "URDummyPose1" and self.ur_pose_refPos_cmd != self.ur_pose_refPos_prev_cmd:
            self.ur_pose_refPos_prev_cmd = self.ur_pose_refPos_cmd
            self.ur_pose_state_refPos = self.ur_pose_refPos_cmd
            self.URToDummyPose1()
            print "urposecmd1"
        elif self.ur_pose_refPos_cmd == "URDummyPose2" and self.ur_pose_state_actPos != "URDummyPose2" and self.ur_pose_refPos_cmd != self.ur_pose_refPos_prev_cmd:
            self.ur_pose_refPos_prev_cmd = self.ur_pose_refPos_cmd
            self.ur_pose_state_refPos = self.ur_pose_refPos_cmd
            self.URToDummyPose2()
            print "urposecmd2"
        elif self.ur_pose_refPos_cmd == "URDummyPose3" and self.ur_pose_state_actPos != "URDummyPose3" and self.ur_pose_refPos_cmd != self.ur_pose_refPos_prev_cmd:
            self.ur_pose_refPos_prev_cmd = self.ur_pose_refPos_cmd
            self.ur_pose_state_refPos = self.ur_pose_refPos_cmd
            self.URToDummyPose3()
            print "urposecmd3"
        elif self.ur_pose_refPos_cmd == "URDummyPose4" and self.ur_pose_state_actPos != "URDummyPose4" and self.ur_pose_refPos_cmd != self.ur_pose_refPos_prev_cmd:
            self.ur_pose_refPos_prev_cmd = self.ur_pose_refPos_cmd
            self.ur_pose_state_refPos = self.ur_pose_refPos_cmd
            self.URToDummyPose4()
            print "urposecmd4"
        else:
            pass
        


    #----------------------------------------------------------------------------------------------------------------
    # jointCallback with sample poses
    #----------------------------------------------------------------------------------------------------------------
    def jointCallback(self, joint):
        
        if numpy.isclose(joint.position[0], self.DummyJoint1[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.DummyJoint1[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.DummyJoint1[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.DummyJoint1[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.DummyJoint1[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.DummyJoint1[5], self.isclose_tolerance):
            self.ur_pose_state_actPos = 'URDummyPose1'

        elif numpy.isclose(joint.position[0], self.DummyJoint2[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.DummyJoint2[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.DummyJoint2[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.DummyJoint2[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.DummyJoint2[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.DummyJoint2[5], self.isclose_tolerance):
            self.ur_pose_state_actPos = 'URDummyPose2'

        elif numpy.isclose(joint.position[0], self.DummyJoint3[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.DummyJoint3[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.DummyJoint3[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.DummyJoint3[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.DummyJoint3[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.DummyJoint3[5], self.isclose_tolerance):
            self.ur_pose_state_actPos= 'URDummyPose3'

        elif numpy.isclose(joint.position[0], self.DummyJoint4[0], self.isclose_tolerance) and\
            numpy.isclose(joint.position[1], self.DummyJoint4[1], self.isclose_tolerance) and\
            numpy.isclose(joint.position[2], self.DummyJoint4[2], self.isclose_tolerance) and\
            numpy.isclose(joint.position[3], self.DummyJoint4[3], self.isclose_tolerance) and\
            numpy.isclose(joint.position[4], self.DummyJoint4[4], self.isclose_tolerance) and\
            numpy.isclose(joint.position[5], self.DummyJoint4[5], self.isclose_tolerance):
            self.ur_pose_state_actPos = 'URDummyPose4'
        
        else:
            self.ur_pose_state_actPos ="Unknown"


        if (joint.velocity[0] or joint.velocity[1] or joint.velocity[2] or joint.velocity[3] or joint.velocity[4] or joint.velocity[5] != 0):
            self.ur_pose_state_executing = True
        else:
            self.ur_pose_state_executing = False
            
        
    #----------------------------------------------------------------------------------------------------------------
    # tcpCallback with sample poses
    #----------------------------------------------------------------------------------------------------------------
    def tcpCallback(self, tcp):
        self.x = tcp.x
        self.y = tcp.y
        self.z = tcp.z

        if numpy.isclose(self.x, self.URAtFilter1Tcp[0], self.isclose_tolerance) and\
            numpy.isclose(self.y, self.URAtFilter1Tcp[1], self.isclose_tolerance) and\
            numpy.isclose(self.z, self.URAtFilter1Tcp[2], self.isclose_tolerance):
            self.ur_tcp_pose_state = 'URAtFilter1Tcp'

        elif numpy.isclose(self.x, self.URAboveFilter1TightenedTcp[0], self.isclose_tolerance) and\
            numpy.isclose(self.y, self.URAboveFilter1TightenedTcp[1], self.isclose_tolerance) and\
            numpy.isclose(self.z, self.URAboveFilter1TightenedTcp[2], self.isclose_tolerance):
            self.ur_tcp_pose_state = 'URAboveFilter1TightenedTcp'

        else:
            self.ur_tcp_pose_state = self.ur_pose_state

if __name__ == '__main__':
    try:
        ur_pose_unidriver()
    except rospy.ROSInterruptException:
        pass
