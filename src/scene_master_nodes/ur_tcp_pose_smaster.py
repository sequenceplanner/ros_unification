#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR TCP Pose Scene Master
    # V.0.2.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
from unification_roscontrol.msg import URTCP
import numpy
import tf
import time

class ur_tcp_pose_smaster():

    def __init__(self):
        
        rospy.init_node('ur_tcp_pose_smaster', anonymous=False)

        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_tcp_pose_smaster", String, self.ur_pose_unidriver_to_ur_tcp_pose_smaster_callback)
        rospy.Subscriber("ethdaq_data", WrenchStamped, self.optoforce_callback)
        rospy.Subscriber("unification_roscontrol/ur_tcp_pose_helper_to_smaster", URTCP, self.urTCPCallback)

        self.ur_tcp_pose_to_unidriver_publisher = rospy.Publisher('unification_roscontrol/ur_tcp_pose_smaster_to_unidriver', String, queue_size=10)
        self.urScriptPublisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

        self.isclose_tolerance = 0.005

        # Unification TCP Poses
        self.tcp_names = ['x', 'y', 'z', 'rx', 'ry', 'rz']

        self.PreAttachAtlasCloseTCPPose = []
        self.AttachAtlasTCPPose = []
        self.PreAttachLFToolCloseTCPPose = []
        self.AttachLFToolTCPPose = []
        self.PreAttachOFToolCloseTCPPose = []
        self.AttachOFToolTCPPose = []
        self.OFToolFrame1TCPPose = []
        self.OFToolFrame2TCPPose = []
        self.OFToolFrame3TCPPose = []
        self.FindEngineRight2TCPPose = []
        self.FindEngineLeft2TCPPose = []
        self.FindEngineMid2TCPPose = []
        self.FindEngineRight3TCPPose = []
        self.FindEngineLeft3TCPPose = []
        self.FindEngineMid3TCPPose = []

        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(5)

        self.main()

    
    def moveL(self, tcpPose, a=1.5, v=0.5, t=0):
        if len(tcpPose) == 6:
            script_str = "movel(p" + str(tcpPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            print "The tcp pose size is not correct."


    def main(self):
        self.FindEngineMid3TCP()

        while not rospy.is_shutdown():        
    
        self.tcp_rate.sleep()
        rospy.spin()



    def ur_pose_unidriver_to_ur_tcp_pose_smaster_callback(self, tcp_cmd):
        self.go_to_tcp_pose = tcp_cmd.data

        if self.go_to_tcp_pose == "PreAttachAtlasCloseTCP":
            self.PreAttachAtlasCloseTCP()

        elif self.go_to_tcp_pose == "AttachAtlasTCP":
            self.AttachAtlasTCP()

        elif self.go_to_tcp_pose == "PreAttachLFToolCloseTCP":
            self.PreAttachLFToolCloseTCP() 

        elif self.go_to_tcp_pose == "AttachLFToolTCP":
            self.AttachLFToolTCP() 

        elif self.go_to_tcp_pose == "PreAttachOFToolCloseTCP":
            self.PreAttachOFToolCloseTCP()

        elif self.go_to_tcp_pose == "AttachOFToolTCP":
            self.AttachOFToolTCP() 

        elif self.go_to_tcp_pose == "OFToolFrame1TCP":
            self.OFToolFrame1TCP()
        
        elif self.go_to_tcp_pose == "OFToolFrame2TCP":
            self.OFToolFrame2TCP() 
        
        elif self.go_to_tcp_pose == "OFToolFrame3TCP":
            self.OFToolFrame3TCP()

        elif self.go_to_joint_pose == "FindEngineRight2TCP":
            self.FindEngineRight2TCP()

        elif self.go_to_joint_pose == "FindEngineLeft2TCP":
            self.FindEngineLeft2TCP()

        elif self.go_to_joint_pose == "FindEngineMid2TCP":
            self.FindEngineMid2TCP()

        elif self.go_to_joint_pose == "FindEngineRight3TCP":
            self.FindEngineRight3TCP()

        elif self.go_to_joint_pose == "FindEngineLeft3TCP":
            self.FindEngineLeft3TCP()

        elif self.go_to_joint_pose == "FindEngineMid3TCP":
            self.FindEngineMid3TCP()

        else:
            pass


    def PreAttachAtlasCloseTCP(self):
        self.moveL(self.PreAttachAtlasCloseTCPPose, a=1.5, v=5, t=3)

    def AttachAtlasTCP(self):
        self.moveL(self.AttachAtlasTCPPose, a=1.5, v=5, t=3)

    def PreAttachLFToolCloseTCP(self):
        self.moveL(self.PreAttachLFToolCloseTCPPose, a=1.5, v=5, t=3)

    def AttachLFToolTCP(self):
        self.moveL(self.AttachLFToolTCPPose, a=1.5, v=5, t=3)
    
    def PreAttachOFToolCloseTCP(self):
        self.moveL(self.PreAttachOFToolCloseTCPPose, a=1.5, v=5, t=3)

    def AttachOFToolTCP(self):
        self.moveL(self.AttachOFToolTCPPose, a=1.5, v=5, t=3)

    def OFToolFrame1TCP(self):
        self.moveL(self.OFToolFrame1TCPPose, a=1.5, v=5, t=3)

    def OFToolFrame2TCP(self):
        self.moveL(self.OFToolFrame2TCPPose, a=1.5, v=5, t=3)

    def OFToolFrame3TCP(self):
        self.moveL(self.OFToolFrame3TCPPose, a=1.5, v=5, t=3)

    def FindEngineRight2TCP(self):
        self.moveL(self.FindEngineRight2TCPPose, a=1.5, v=5, t=3)

    def FindEngineRight3TCP(self):
        self.moveL(self.FindEngineRight3TCPPose, a=1.5, v=5, t=10)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_right_x = self.tcp_x
                self.found_engine_right_y = self.tcp_y
                break
            else:
                pass

    def FindEngineLeft2TCP(self):
        self.moveL(self.FindEngineLeft2TCPPose, a=1.5, v=5, t=3)

    def FindEngineLeft3TCP(self):
        self.moveL(self.FindEngineLeft3TCPPose, a=1.5, v=5, t=10)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_left_x = self.tcp_x
                self.found_engine_left_y = self.tcp_y
                break
            else:
                pass

    def FindEngineMid2TCP(self):
        self.moveL(self.FindEngineMid2TCPPose, a=1.5, v=5, t=3)

    def FindEngineMid3TCP(self):
        self.moveL(self.FindEngineMid3TCPPose, a=1.5, v=5, t=20)
        rospy.sleep(1)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_mid_x = self.tcp_x
                self.found_engine_mid_y = self.tcp_y
                break
            else:
                pass


    def optoforce_callback(self, data):
        self.z = data.wrench.force.z

    def urTCPCallback(self, data):
        self.tcp_x = data.x
        self.tcp_y = data.y
        self.tcp_z = data.z
        self.tcp_r = data.r
        self.tcp_p = data.p
        self.tcp_w = data.w

        for pose in [self.PreAttachAtlasCloseTCPPose, 
            self.AttachAtlasTCPPose,
            self.PreAttachLFToolCloseTCPPose,
            self.AttachLFToolTCPPose,
            self.PreAttachOFToolCloseTCPPose,
            self.AttachOFToolTCPPose,
            self.OFToolFrame1TCPPose,
            self.OFToolFrame2TCPPose,
            self.OFToolFrame3TCPPose
            self.FindEngineRight2TCPPose,
            self.FindEngineLeft2TCPPose,
            self.FindEngineMid2TCPPose,
            self.FindEngineRight3TCPPose,
            self.FindEngineLeft3TCPPose,
            self.FindEngineMid3TCPPose]:

            if  numpy.isclose(self.tcp_x, pose[0], self.isclose_tolerance) and\
                numpy.isclose(self.tcp_y, pose[1], self.isclose_tolerance) and\
                numpy.isclose(self.tcp_z, pose[2], self.isclose_tolerance) and\
                numpy.isclose(self.tcp_r, pose[3], self.isclose_tolerance) and\
                numpy.isclose(self.tcp_p, pose[4], self.isclose_tolerance) and\
                numpy.isclose(self.tcp_w, pose[5], self.isclose_tolerance):
                self.ur_tcp_pose_name = pose
            
            else:
                self.ur_tcp_pose_name = []
                    
            
        if self.ur_tcp_pose_name == self.PreAttachAtlasCloseTCPPose:
            self.ur_tcp_pose_state = "PreAttachAtlasCloseTCP"

        elif self.ur_tcp_pose_name == self.AttachAtlasTCPPose:
            self.ur_tcp_pose_state = "AttachAtlasTCP"

        elif self.ur_tcp_pose_name == self.PreAttachLFToolCloseTCPPose:
            self.ur_tcp_pose_state = "PreAttachLFToolCloseTCP"

        elif self.ur_tcp_pose_name == self.AttachLFToolTCPPose:
            self.ur_tcp_pose_state = "AttachLFToolTCP"

        elif self.ur_tcp_pose_name == self.PreAttachOFToolCloseTCPPose:
            self.ur_tcp_pose_state = "PreAttachOFToolCloseTCP"

        elif self.ur_tcp_pose_name == self.AttachOFToolTCPPose:
            self.ur_tcp_pose_state = "AttachOFToolTCP"

        elif self.ur_tcp_pose_name == self.OFToolFrame1TCPPose:
            self.ur_tcp_pose_state = "OFToolFrame1TCP"

        elif self.ur_tcp_pose_name == self.OFToolFrame2TCPPose:
            self.ur_tcp_pose_state = "OFToolFrame2TCP"

        elif self.ur_tcp_pose_name == self.OFToolFrame3TCPPose:
            self.ur_tcp_pose_state = "OFToolFrame3TCP"

        elif self.ur_tcp_pose_name == self.FindEngineRight2TCPPose:
            self.ur_tcp_pose_state = "FindEngineRight2TCP"

        elif self.ur_tcp_pose_name == self.FindEngineLeft2TCPPose:
            self.ur_tcp_pose_state = "FindEngineLeft2TCP"
        
        elif self.ur_tcp_pose_name == self.FindEngineMid2TCPPose:
            self.ur_tcp_pose_state = "FindEngineMid2TCP"

        else:
            self.ur_tcp_pose_state = 'unknown'


if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster()
    except rospy.ROSInterruptException:
        pass
