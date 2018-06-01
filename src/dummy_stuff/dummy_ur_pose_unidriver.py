#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Pose Unification Driver for the Universal Robots UR10 
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
import tf
import time
import numpy
from math import pi


class ur_pose_unidriver():

    def __init__(self):
        
        rospy.init_node('ur_pose_unidriver', anonymous=False)
        
        self.ur_pose_state = '_'
        self.isclose_tolerance = 0.005

        #------------------------------------------------------------------------------------------------------------
        # Sample Hardcoded Poses
        #------------------------------------------------------------------------------------------------------------
        self.URHomePosJ = [0, -1.5717671553241175, -1.5510733763324183, -1.5855825583087366, 0, 0.012329895049333572]
        self.URPickPosJ = [0, -1.5718267599688929, 1.5617852210998535, -1.5857146422015589, 0, 0.012365847826004028]
        self.URRandomPosL = [0.374, -0.425, 0.369, 1.1640, -4.7281, 0]

        rospy.Subscriber("/joint_states", JointState, self.jointCallback)
        rospy.Subscriber("/bridge_to_driver", String, self.sp_to_driver_callback)
        rospy.Subscriber("/ur_tcp_pose", Point, self.tcpCallback)

        self.URScriptPublisher = rospy.Publisher("ur_driver/URScript", String, queue_size=200)
        self.ur_pose_state_publisher = rospy.Publisher('ur_pose_unistate', String, queue_size=10)
        self.message_ack_publisher = rospy.Publisher('driver_to_bridge', String, queue_size=10)
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()

    #------------------------------------------------------------------------------------------------------------
    # movej: Move to position (linear in joint-space). When using this command, the robot must be at standstill or come from a movej or movel with a
    # blend. The speed and acceleration parameters controls the trapezoid speed profile of the move. The $t$ parameter can be used instead to
    # set the time for this move. Time setting has priority over speed and acceleration settings. The blend radius can be set with the $r$
    # parameters, to avoid the robot stopping at the point. However, if the blend region of this mover overlaps with previous or following regions,
    # this move will be skipped, and an 'Overlapping Blends' warning message will be generated.
    #------------------------------------------------------------------------------------------------------------
    def urSrciptToStringJ(self, move="movej", jointPose=[0,0,0,0,0,0], a=1.0, v=0.2, t=8, r=0):
        return move + "(" + str(jointPose)+ ", a="+str(a)+ ", v="+str(v)+ ", t="+str(t)+ ", r="+str(r)+ ")"

    #------------------------------------------------------------------------------------------------------------
    # Move to position (linear in tool-space)
    #------------------------------------------------------------------------------------------------------------
    def urSrciptToStringL(self, move="movel", eeTcpPose=[0,0,0,0,0,0], a=1.0, v=0.2, t=8, r=0):
        return move + "(" + "p" + str(eeTcpPose)+ ", a="+str(a)+ ", v="+str(v)+ ", t="+str(t)+ ", r="+str(r)+ ")"


    #------------------------------------------------------------------------------------------------------------
    # Sample move methods
    #------------------------------------------------------------------------------------------------------------
    def URToHomePos(self):
        command = self.urSrciptToStringJ(move = "movej", jointPose = self.URHomePosJ, a = 1.5, v = 7, t = 6, r = 0)
        self.URScriptPublisher.publish(command)
        rospy.sleep(7)
    
    def URToPickPos(self):
        command = self.urSrciptToStringJ(move = "movej", jointPose = self.URPickPosJ, a = 1.5, v = 7, t = 6, r = 0)
        self.URScriptPublisher.publish(command)
        rospy.sleep(7)

    def URToRandomPos(self):
        command = self.urSrciptToStringL(move = "movel", eeTcpPose = self.URRandomPosL, a = 1.5, v = 7, t = 6, r = 0)
        self.URScriptPublisher.publish(command)
        rospy.sleep(7)

    
    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
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
