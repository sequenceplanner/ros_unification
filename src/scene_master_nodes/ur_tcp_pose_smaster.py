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
import sys
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

        self.z = 0
        self.isclose_tolerance = 0.005
        self.ur_tcp_pose_state = ""
        self.go_to_tcp_pose_prev = ""

        # Unification TCP Poses
        self.tcp_names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        self.ResetTCPPose = []

        #----------------------------------------------------------------------------------------
        # Force sensor engine positioning
        #----------------------------------------------------------------------------------------
        self.FindEngineRightDownTCPPose = [-0.476027542627, -0.560034757959, 1.02107340693, -1.57525004049, 0.299255540234, 0.364537901289]
        self.FindEngineRightColideTCPPose = [-0.476111259039, -0.4, 1.02100659443, -1.57516026151, 0.299218018579, 0.364387736323]
        self.FindEngineLeftDownTCPPose = [-0.870853950336, 0.153700134813, 1.00801072426, 1.1391753846, 1.16284516465, -1.2144095644]
        self.FindEngineLeftColideTCPPose = [-0.870853950336, 0.0, 1.00801072426, 1.1391753846, 1.16284516465, -1.2144095644]
        self.FindEngineMidDownTCPPose = [ 0.0618898965066, -0.203589651576, 1.01116829519, -2.06828472623, -0.148077989726, 2.20184158041]
        self.FindEngineMiColideTCPPose = [-0.0773473988767, -0.20360053125, 1.01112718356, -2.06825873066, -0.148186823079, 2.20172386733]
        #----------------------------------------------------------------------------------------



        #----------------------------------------------------------------------------------------
        # Tool Changing TCP Poses
        #----------------------------------------------------------------------------------------
        # AAPR -- After Attach - Pre Release
        self.PreAttachAtlasCloseTCPPose = [-0.953986242747, -0.234228631949, -0.223034435224, -1.4005366775, -0.925839002818, 1.55970065186]
        self.AttachAtlasTCPPose = [-0.967152647463, -0.234133433682, -0.222960059804, -1.40056793776, -0.925751048978, 1.55957875389]
        self.AAPRAtlasTCPPose = [-1.01261300113, -0.234543501521, 0.111905916583, -1.51448717582, -1.00198300826, 1.48195499752]

        self.PreAttachLFToolCloseTCPPose = [-0.314196796263, -0.460490185045, -0.307773304476, 1.49509891331, 0.991289183192, 1.49449508332]
        self.AttachLFToolTCPPose = [-0.301317571922, -0.462295211687, -0.307756046838, 1.50210590005, 1.00838306356, 1.49525355553]
        self.AAPRLFToolTCPPose = [-0.307231511674, -0.460493467174, -0.307776610243, 1.53641086151, 1.01881285112, 1.4653680225]

        self.PreAttachOFToolCloseTCPPose = [-0.244335748226, -0.304946422001, -0.323058309051, 1.51053055017, 1.01657987242, 1.47450408777]
        self.AttachOFToolTCPPose = [-0.230157982196, -0.304933361338, -0.323093185807, 1.51063178157, 1.01647663729, 1.4742918683]
        self.AAPROFTool1TCPPose = [-0.230106567314, -0.304915066616, -0.44304589107, 1.51061888966, 1.01641048992, 1.47443984473]
        self.AAPROFTool2TCPPose = [-0.302339094126, -0.304920167014, -0.44311872881, 1.51065216074, 1.01640194847, 1.47444601524]
        #----------------------------------------------------------------------------------------




        #----------------------------------------------------------------------------------------
        # LF Tightening
        #----------------------------------------------------------------------------------------
        self.AboveEngineTCPPose = [-0.570401398899, -0.213429808561, 0.444701426899, -1.5022823698, -1.02199286632, 1.46496484875]

        self.FarAboveBoltPair1TCPPose = [-0.0784896487942, -0.198313371814, 0.560034029131, -1.50228268223, -1.02180473399, 1.46494776698]
        self.CloseAboveBoltPair1TCPPose = [-0.0907433283046, -0.198958413547, 0.598781784017, -1.42736061959, -1.01986093313, 1.46158494504]
        self.AtBoltPair1TCPPose = [-0.0907433283046, -0.198958413547, 0.598781784017 + 0.15, -1.42736061959, -1.01986093313, 1.46158494504]

        self.FarAboveBoltPair2TCPPose = [-0.170838119192, -0.198953101009, 0.577627098101, -1.42736849013, -1.01971978766, 1.46174461364]
        self.CloseAboveBoltPair2TCPPose = [-0.170857851833, -0.198957378248, 0.598913374413, -1.42736275317, -1.01991528985, 1.46156533033]
        self.AtBoltPair2TCPPose = [-0.170857851833, -0.198957378248, 0.598913374413 + 0.15, -1.42736275317, -1.01991528985, 1.46156533033]

        self.FarAboveBoltPair3TCPPose = [-0.251794736677, -0.19894098752, 0.560131982471, -1.42736357259, -1.0197090458, 1.46161178315]
        self.CloseAboveBoltPair3TCPPose = [-0.251764739397, -0.198932267163, 0.599280223362, -1.42737193363, -1.01978563604, 1.46151771638]
        self.AtBoltPair3TCPPose = [-0.251764739397, -0.198932267163, 0.599280223362 + 0.15, -1.42737193363, -1.01978563604, 1.46151771638]

        self.FarAboveBoltPair4TCPPose = [] 
        self.CloseAboveBoltPair4TCPPos = []
        self.AtBoltPair4TCPPose = []

        self.FarAboveBoltPair5TCPPose = [] 
        self.CloseAboveBoltPair5TCPPos = []
        self.AtBoltPair5TCPPose = []

        self.FarAboveBoltPair6TCPPose = [] 
        self.CloseAboveBoltPair6TCPPos = []
        self.AtBoltPair6TCPPose = []

        self.FarAboveBoltPair7TCPPose = [] 
        self.CloseAboveBoltPair7TCPPos = []
        self.AtBoltPair7TCPPose = []

        self.FarAboveBoltPair8TCPPose = [] 
        self.CloseAboveBoltPair8TCPPos = []
        self.AtBoltPair8TCPPose = []

        self.FarAboveBoltPair9TCPPose = [] 
        self.CloseAboveBoltPair9TCPPos = []
        self.AtBoltPair9TCPPose = []

        self.FarAboveBoltPair10TCPPose = [] 
        self.CloseAboveBoltPair10TCPPos = []
        self.AtBoltPair10TCPPose = []

        self.FarAboveBoltPair11TCPPose = [] 
        self.CloseAboveBoltPair11TCPPos = []
        self.AtBoltPair11TCPPose = []

        self.FarAboveBoltPair12TCPPose = [] 
        self.CloseAboveBoltPair12TCPPos = []
        self.AtBoltPair12TCPPose = []
        #----------------------------------------------------------------------------------------




        #----------------------------------------------------------------------------------------
        # OF Tightening
        #----------------------------------------------------------------------------------------
        self.OFMidpoint1TCPPose = []
        self.OFMidpoint2TCPPose = []

        self.AboveUntightenedOF1TCPPose = []
        self.AtUntightenedOF1TCPPose = []
        self.AtTightenedOF1TCPPose = []
        self.AboveTightenedOF1TCPPose = []

        self.AboveUntightenedOF2TCPPose = []
        self.AtUntightenedOF2TCPPose = []
        self.AtTightenedOF2TCPPose = []
        self.AboveTightenedOF2TCPPose = []

        self.AboveUntightenedOF3TCPPose = []
        self.AtUntightenedOF3TCPPose = []
        self.AtTightenedOF3TCPPose = []
        self.AboveTightenedOF3TCPPose = []
        #----------------------------------------------------------------------------------------




        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(5)

        self.main()

    
    def moveL(self, tcpPose, a=0.3, v=0.5, t=0):
        if len(tcpPose) == 6:
            script_str = "movel(p" + str(tcpPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            print "The tcp pose size is not correct."


    def main(self):
        #self.FindEngineMid3TCP()

        while not rospy.is_shutdown():        
    
            self.ur_tcp_pose_to_unidriver_publisher.publish(self.ur_tcp_pose_state)

            self.tcp_rate.sleep()
        rospy.spin()



    def ur_pose_unidriver_to_ur_tcp_pose_smaster_callback(self, tcp_cmd):
        self.go_to_tcp_pose = tcp_cmd.data

        if self.go_to_tcp_pose == "reset":
            self.go_to_tcp_pose_prev = "reset"
    
        # This is awesome...
        for pose in ["PreAttachAtlasCloseTCP",
                     "AAPRAtlasTCP",
                     "PreAttachLFToolCloseTCP",
                     "AttachLFToolTCP"]:
            if self.go_to_tcp_pose == pose and self.go_to_tcp_pose_prev != pose:
                self.go_to_tcp_pose_prev = pose
                tcp_pose = "self." + pose + "Pose"
                method = "self.moveL(" + tcp_pose + ", a=0.3, v=0.5, t=0)"
                eval(method)
            else:
                pass
            

        #if self.go_to_tcp_pose == "PreAttachAtlasCloseTCP" and self.go_to_tcp_pose_prev != "PreAttachAtlasCloseTCP":
        #    self.go_to_tcp_pose_prev = "PreAttachAtlasCloseTCP"
        #    self.PreAttachAtlasCloseTCP()

        '''
        if self.go_to_tcp_pose == "AttachAtlasTCP" and self.go_to_tcp_pose_prev != "AttachAtlasTCP":
            self.go_to_tcp_pose_prev = "AttachAtlasTCP"
            self.AttachAtlasTCP()

        elif self.go_to_tcp_pose == "AAPRAtlasTCP" and self.go_to_tcp_pose_prev != "AAPRAtlasTCP":
            self.go_to_tcp_pose_prev = "AAPRAtlasTCP"
            self.AAPRAtlasTCP()

        elif self.go_to_tcp_pose == "PreAttachLFToolCloseTCP" and self.go_to_tcp_pose_prev != "PreAttachLFToolCloseTCP":
            self.go_to_tcp_pose_prev = "PreAttachLFToolCloseTCP"
            self.PreAttachLFToolCloseTCP() 

        elif self.go_to_tcp_pose == "AttachLFToolTCP" and self.go_to_tcp_pose_prev != "AttachLFToolTCP":
            self.go_to_tcp_pose_prev = "AttachLFToolTCP"
            self.AttachLFToolTCP() 

        elif self.go_to_tcp_pose == "AAPRLFToolTCP" and self.go_to_tcp_pose_prev != "AAPRLFToolTCP":
            self.go_to_tcp_pose_prev = "AAPRLFToolTCP"
            self.AAPRLFToolTCP()

        elif self.go_to_tcp_pose == "PreAttachOFToolCloseTCP" and self.go_to_tcp_pose_prev != "PreAttachOFToolCloseTCP":
            self.go_to_tcp_pose_prev = "PreAttachOFToolCloseTCP"
            self.PreAttachOFToolCloseTCP()

        elif self.go_to_tcp_pose == "AttachOFToolTCP" and self.go_to_tcp_pose_prev != "AttachOFToolTCP":
            self.go_to_tcp_pose_prev = "AttachOFToolTCP"
            self.AttachOFToolTCP()

        elif self.go_to_tcp_pose == "AAPROFTool1TCP" and self.go_to_tcp_pose_prev != "AAPROFTool1TCP":
            self.go_to_tcp_pose_prev = "AAPROFTool1TCP"
            self.AAPROFTool1TCP()
        
        elif self.go_to_tcp_pose == "AAPROFTool2TCP" and self.go_to_tcp_pose_prev != "AAPROFTool2TCP":
            self.go_to_tcp_pose_prev = "AAPROFTool2TCP"
            self.AAPROFTool2TCP()

        elif self.go_to_tcp_pose == "AboveEngineTCP" and self.go_to_tcp_pose_prev != "AboveEngineTCP":
            self.go_to_tcp_pose_prev = "AboveEngineTCP"
            self.AboveEngineTCP()


        elif self.go_to_tcp_pose == "FarAboveBoltPair1TCP" and self.go_to_tcp_pose_prev != "FarAboveBoltPair1TCP":
            self.go_to_tcp_pose_prev = "FarAboveBoltPair1TCP"
            self.FarAboveBoltPair1TCP() 

        elif self.go_to_tcp_pose == "CloseAboveBoltPair1TCP" and self.go_to_tcp_pose_prev != "CloseAboveBoltPair1TCP":
            self.go_to_tcp_pose_prev = "CloseAboveBoltPair1TCP"
            self.CloseAboveBoltPair1TCP() 

        elif self.go_to_tcp_pose == "AtBoltPair1TCP" and self.go_to_tcp_pose_prev != "AtBoltPair1TCP":
            self.go_to_tcp_pose_prev = "AtBoltPair1TCP"
            self.AtBoltPair1TCP()


        elif self.go_to_tcp_pose == "FarAboveBoltPair2TCP" and self.go_to_tcp_pose_prev != "FarAboveBoltPair2TCP":
            self.go_to_tcp_pose_prev = "FarAboveBoltPair2TCP"
            self.FarAboveBoltPair2TCP() 

        elif self.go_to_tcp_pose == "CloseAboveBoltPair2TCP" and self.go_to_tcp_pose_prev != "CloseAboveBoltPair2TCP":
            self.go_to_tcp_pose_prev = "CloseAboveBoltPair2TCP"
            self.CloseAboveBoltPair2TCP() 

        elif self.go_to_tcp_pose == "AtBoltPair2TCP" and self.go_to_tcp_pose_prev != "AtBoltPair2TCP":
            self.go_to_tcp_pose_prev = "AtBoltPair2TCP"
            self.AtBoltPair2TCP()


        elif self.go_to_tcp_pose == "FarAboveBoltPair3TCP" and self.go_to_tcp_pose_prev != "FarAboveBoltPair3TCP":
            self.go_to_tcp_pose_prev = "FarAboveBoltPair3TCP"
            self.FarAboveBoltPair3TCP() 

        elif self.go_to_tcp_pose == "CloseAboveBoltPair3TCP" and self.go_to_tcp_pose_prev != "CloseAboveBoltPair3TCP":
            self.go_to_tcp_pose_prev = "CloseAboveBoltPair3TCP"
            self.CloseAboveBoltPair3TCP() 

        elif self.go_to_tcp_pose == "AtBoltPair3TCP" and self.go_to_tcp_pose_prev != "AtBoltPair3TCP":
            self.go_to_tcp_pose_prev = "AtBoltPair3TCP"
            self.AtBoltPair3TCP()


        elif self.go_to_tcp_pose == "OFToolFrame1TCP":
            self.OFToolFrame1TCP()
        
        elif self.go_to_tcp_pose == "OFToolFrame2TCP":
            self.OFToolFrame2TCP() 
        
        elif self.go_to_tcp_pose == "OFToolFrame3TCP":
            self.OFToolFrame3TCP()

        else:
            pass

    
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
        '''

    def ResetTCP(self):
        self.moveL(self.ResetTCPPose, a=0.3, v=0.5, t=0)

    #def PreAttachAtlasCloseTCP(self):
    #    self.moveL(self.PreAttachAtlasCloseTCPPose, a=0.3, v=0.5, t=0)

    def AttachAtlasTCP(self):
        self.moveL(self.AttachAtlasTCPPose, a=0.3, v=0.5, t=0)

    def AAPRAtlasTCP(self):
        self.moveL(self.AAPRAtlasTCPPose, a=0.3, v=0.5, t=0)

    def PreAttachLFToolCloseTCP(self):
        self.moveL(self.PreAttachLFToolCloseTCPPose, a=0.3, v=0.5, t=0)

    def AttachLFToolTCP(self):
        self.moveL(self.AttachLFToolTCPPose, a=0.3, v=0.5, t=0)
    
    def PreAttachOFToolCloseTCP(self):
        self.moveL(self.PreAttachOFToolCloseTCPPose, a=0.3, v=0.5, t=0)

    def AttachOFToolTCP(self):
        self.moveL(self.AttachOFToolTCPPose, a=0.3, v=0.5, t=0)

    def AboveEngineTCP(self):
        self.moveL(self.AboveEngineTCPPose, a=0.3, v=0.5, t=0)


    def AAPRLFToolTCP(self):
        self.moveL(self.AAPRLFToolTCPPose, a=0.3, v=0.5, t=0)

    def AAPROFTool1TCP(self):
        self.moveL(self.AAPROFTool1TCPPose, a=0.3, v=0.5, t=0)

    def AAPROFTool2TCP(self):
        self.moveL(self.AAPROFTool2TCPPose, a=0.3, v=0.5, t=0)


    def FarAboveBoltPair1TCP(self):\
        self.moveL(self.FarAboveBoltPair1TCPPose, a=0.3, v=0.5, t=6)

    def CloseAboveBoltPair1TCP(self):
        self.moveL(self.CloseAboveBoltPair1TCPPose, a=0.3, v=0.5, t=6)

    def AtBoltPair1TCP(self):
        self.moveL(self.AtBoltPair1TCPPose, a=0.3, v=0.5, t=6)
    

    def FarAboveBoltPair2TCP(self):\
        self.moveL(self.FarAboveBoltPair2TCPPose, a=0.3, v=0.5, t=6)

    def CloseAboveBoltPair2TCP(self):
        self.moveL(self.CloseAboveBoltPair2TCPPose, a=0.3, v=0.5, t=6)

    def AtBoltPair2TCP(self):
        self.moveL(self.AtBoltPair2TCPPose, a=0.3, v=0.5, t=6)


    def FarAboveBoltPair3TCP(self):\
        self.moveL(self.FarAboveBoltPair3TCPPose, a=0.3, v=0.5, t=6)

    def CloseAboveBoltPair3TCP(self):
        self.moveL(self.CloseAboveBoltPair3TCPPose, a=0.3, v=0.5, t=6)

    def AtBoltPair3TCP(self):
        self.moveL(self.AtBoltPair3TCPPose, a=0.3, v=0.5, t=6)



    def FindEngineRight2TCP(self):
        self.moveL(self.FindEngineRight2TCPPose, a=0.3, v=0.5, t=0)

    def FindEngineRight3TCP(self):
        self.moveL(self.FindEngineRight3TCPPose, a=0.3, v=0.5, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_right_x = self.tcp_x
                self.found_engine_right_y = self.tcp_y
                break
            else:
                pass

    def FindEngineLeft2TCP(self):
        self.moveL(self.FindEngineLeft2TCPPose, a=0.3, v=0.5, t=0)

    def FindEngineLeft3TCP(self):
        self.moveL(self.FindEngineLeft3TCPPose, a=0.3, v=0.5, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_left_x = self.tcp_x
                self.found_engine_left_y = self.tcp_y
                break
            else:
                pass

    def FindEngineMid2TCP(self):
        self.moveL(self.FindEngineMid2TCPPose, a=0.3, v=0.5, t=0)

    def FindEngineMid3TCP(self):
        self.moveL(self.FindEngineMid3TCPPose, a=0.3, v=0.5, t=0)
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
        self.tcp_rx = data.rx
        self.tcp_ry = data.ry
        self.tcp_rz = data.rz

        self.ResetTCPPose = [self.tcp_x, self.tcp_y, self.tcp_z, self.tcp_rx, self.tcp_ry, self.tcp_rz]

        self.ur_tcp_pose_name = []

        for pose in [self.PreAttachAtlasCloseTCPPose, 
            self.AttachAtlasTCPPose,
            self.PreAttachLFToolCloseTCPPose,
            self.AAPRAtlasTCPPose,
            self.AttachLFToolTCPPose,
            self.AAPRLFToolTCPPose,
            self.PreAttachOFToolCloseTCPPose,
            self.AttachOFToolTCPPose,
            self.AAPROFTool1TCPPose,
            self.AAPROFTool2TCPPose,
            self.FarAboveBoltPair1TCPPose,
            self.CloseAboveBoltPair1TCPPose,
            self.AtBoltPair1TCPPose,
            self.FarAboveBoltPair2TCPPose,
            self.CloseAboveBoltPair2TCPPose,
            self.AtBoltPair2TCPPose,
            self.FarAboveBoltPair3TCPPose,
            self.CloseAboveBoltPair3TCPPose,
            self.AtBoltPair3TCPPose]:

            if  abs((abs(self.tcp_x) - abs(pose[0]))) < 0.001 and\
                abs((abs(self.tcp_y) - abs(pose[1]))) < 0.001 and\
                abs((abs(self.tcp_z) - abs(pose[2]))) < 0.001 and\
                abs((abs(self.tcp_rx) - abs(pose[3]))) < 0.001 and\
                abs((abs(self.tcp_ry) - abs(pose[4]))) < 0.001 and\
                abs((abs(self.tcp_rz) - abs(pose[5]))) < 0.001:
                self.ur_tcp_pose_name = pose
            
                    
            
            if self.ur_tcp_pose_name == self.PreAttachAtlasCloseTCPPose:
                self.ur_tcp_pose_state = "PreAttachAtlasCloseTCP"

            elif self.ur_tcp_pose_name == self.AttachAtlasTCPPose:
                self.ur_tcp_pose_state = "AttachAtlasTCP"

            elif self.ur_tcp_pose_name == self.AAPRAtlasTCPPose:
                self.ur_tcp_pose_state = "AAPRAtlasTCP"

            elif self.ur_tcp_pose_name == self.PreAttachLFToolCloseTCPPose:
                self.ur_tcp_pose_state = "PreAttachLFToolCloseTCP"

            elif self.ur_tcp_pose_name == self.AttachLFToolTCPPose:
                self.ur_tcp_pose_state = "AttachLFToolTCP"

            elif self.ur_tcp_pose_name == self.AAPRLFToolTCPPose:
                self.ur_tcp_pose_state = "AAPRLFToolTCP"

            elif self.ur_tcp_pose_name == self.PreAttachOFToolCloseTCPPose:
                self.ur_tcp_pose_state = "PreAttachOFToolCloseTCP"

            elif self.ur_tcp_pose_name == self.AttachOFToolTCPPose:
                self.ur_tcp_pose_state = "AttachOFToolTCP"

            elif self.ur_tcp_pose_name == self.AAPROFTool1TCPPose:
                self.ur_tcp_pose_state = "AAPROFTool1TCP"

            elif self.ur_tcp_pose_name == self.AAPROFTool2TCPPose:
                self.ur_tcp_pose_state = "AAPROFTool2TCP"


            elif self.ur_tcp_pose_name == self.FarAboveBoltPair1TCPPose:
                self.ur_tcp_pose_state = "FarAboveBoltPair1TCP"

            elif self.ur_tcp_pose_name == self.CloseAboveBoltPair1TCPPose:
                self.ur_tcp_pose_state = "CloseAboveBoltPair1TCP"

            elif self.ur_tcp_pose_name == self.AtBoltPair1TCPPose:
                self.ur_tcp_pose_state = "AtBoltPair1TCP"

            
            elif self.ur_tcp_pose_name == self.FarAboveBoltPair2TCPPose:
                self.ur_tcp_pose_state = "FarAboveBoltPair2TCP"

            elif self.ur_tcp_pose_name == self.CloseAboveBoltPair2TCPPose:
                self.ur_tcp_pose_state = "CloseAboveBoltPair2TCP"

            elif self.ur_tcp_pose_name == self.AtBoltPair2TCPPose:
                self.ur_tcp_pose_state = "AtBoltPair2TCP"

            
            elif self.ur_tcp_pose_name == self.FarAboveBoltPair3TCPPose:
                self.ur_tcp_pose_state = "FarAboveBoltPair3TCP"

            elif self.ur_tcp_pose_name == self.CloseAboveBoltPair3TCPPose:
                self.ur_tcp_pose_state = "CloseAboveBoltPair3TCP"

            elif self.ur_tcp_pose_name == self.AtBoltPair3TCPPose:
                self.ur_tcp_pose_state = "AtBoltPair3TCP"


            else:
                self.ur_tcp_pose_state = 'unknown'

            
        
        '''
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
        '''

        


if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster()
    except rospy.ROSInterruptException:
        pass
