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
        self.FindEngineRightCollideTCPPose = [-0.476111259039, -0.4, 1.02100659443, -1.57516026151, 0.299218018579, 0.364387736323]
        self.FindEngineLeftDownTCPPose = [-0.870853950336, 0.153700134813, 1.00801072426, 1.1391753846, 1.16284516465, -1.2144095644]
        self.FindEngineLeftCollideTCPPose = [-0.870853950336, 0.0, 1.00801072426, 1.1391753846, 1.16284516465, -1.2144095644]
        self.FindEngineMidDownTCPPose = [ 0.0618898965066, -0.203589651576, 1.01116829519, -2.06828472623, -0.148077989726, 2.20184158041]
        self.FindEngineMiCollideTCPPose = [-0.0773473988767, -0.20360053125, 1.01112718356, -2.06825873066, -0.148186823079, 2.20172386733]
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
        self.CloseAboveBoltPair4TCPPose = []
        self.AtBoltPair4TCPPose = []

        self.FarAboveBoltPair5TCPPose = [] 
        self.CloseAboveBoltPair5TCPPose = []
        self.AtBoltPair5TCPPose = []

        self.FarAboveBoltPair6TCPPose = [] 
        self.CloseAboveBoltPair6TCPPose = []
        self.AtBoltPair6TCPPose = []

        self.FarAboveBoltPair7TCPPose = [] 
        self.CloseAboveBoltPair7TCPPose = []
        self.AtBoltPair7TCPPose = []

        self.FarAboveBoltPair8TCPPose = [] 
        self.CloseAboveBoltPair8TCPPose = []
        self.AtBoltPair8TCPPose = []

        self.FarAboveBoltPair9TCPPose = [] 
        self.CloseAboveBoltPair9TCPPose = []
        self.AtBoltPair9TCPPose = []

        self.FarAboveBoltPair10TCPPose = [] 
        self.CloseAboveBoltPair10TCPPose = []
        self.AtBoltPair10TCPPose = []

        self.FarAboveBoltPair11TCPPose = [] 
        self.CloseAboveBoltPair11TCPPose = []
        self.AtBoltPair11TCPPose = []

        self.FarAboveBoltPair12TCPPose = [] 
        self.CloseAboveBoltPair12TCPPose = []
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


        self.pose_names = ["FindEngineRightDownTCP",
            "FindEngineRightCollideTCP", 
            "FindEngineLeftDownTCP",
            "FindEngineMidDownTCP",
            "FindEngineMiCollideTCP",
            "PreAttachAtlasCloseTCP", 
            "AttachAtlasTCP",
            "AAPRAtlasTCP",
            "PreAttachLFToolCloseTCP",
            "AttachLFToolTCP",
            "AAPRLFToolTCP",
            "PreAttachOFToolCloseTCP",
            "AttachOFToolTCP",
            "AAPROFTool1TCP",
            "AAPROFTool2TCP",
            "AboveEngineTCP",
            "FarAboveBoltPair1TCP",
            "CloseAboveBoltPair1TCP", 
            "AtBoltPair1TCP",
            "FarAboveBoltPair2TCP",
            "CloseAboveBoltPair2TCP",
            "AtBoltPair2TCP",
            "FarAboveBoltPair3TCP",
            "CloseAboveBoltPair3TCP", 
            "AtBoltPair3TCP"
            #"FarAboveBoltPair4TCP",
            #"CloseAboveBoltPair4TCP",
            #"AtBoltPair4TCP",
            #"FarAboveBoltPair5TCP",
            #"CloseAboveBoltPair5TCP",
            #"AtBoltPair5TCP",
            #"FarAboveBoltPair6TCP",
            #"CloseAboveBoltPair6TCP",
            #"AtBoltPair6TCP",
            #"FarAboveBoltPair7TCP",
            #"CloseAboveBoltPair7TCP",
            #"AtBoltPair7TCP",
            #"FarAboveBoltPair8TCP",
            #"CloseAboveBoltPair8TCP",
            #"AtBoltPair8TCP",
            #"FarAboveBoltPair9TCP",
            #"CloseAboveBoltPair9TCP",
            #"AtBoltPair9TCP",
            #"FarAboveBoltPair10TCP",
            #"CloseAboveBoltPair10TCP", 
            #"AtBoltPair10TCP",
            #"FarAboveBoltPair11TCP",
            #"CloseAboveBoltPair11TCP", 
            #"AtBoltPair11TCP",
            #"FarAboveBoltPair12TCP",
            #"CloseAboveBoltPair12TCP", 
            #"AtBoltPair12TCP",
            #"OFMidpoint1TCP",
            #"OFMidpoint2TCP",
            #"AboveUntightenedOF1TCP", 
            #"AtUntightenedOF1TCP",
            #"AtTightenedOF1TCP",
            #"AboveTightenedOF1TCP",
            #"AboveUntightenedOF2TCP", 
            #"AtUntightenedOF2TCP",
            #"AtTightenedOF2TCP",
            #"AboveTightenedOF2TCP",
            #"AboveUntightenedOF3TCP", 
            #"AtUntightenedOF3TCP",
            #"AtTightenedOF3TCP",
            #"AboveTightenedOF3TCP"
            ]



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
        for pose in self.pose_names:
            if self.go_to_tcp_pose == pose and self.go_to_tcp_pose_prev != pose:
                self.go_to_tcp_pose_prev = pose
                tcp_pose = "self." + pose + "Pose"
                method = "self.moveL(" + tcp_pose + ", a=0.3, v=0.5, t=0)"
                eval(method)
            else:
                pass
            

        if self.go_to_tcp_pose == "FindEngineRightCollideTCP" and self.go_to_tcp_pose_prev != "FindEngineRightCollideTCP":
            self.go_to_tcp_pose_prev = "FindEngineRightCollideTCP"
            self.FindEngineRightCollideTCP()

        elif self.go_to_tcp_pose == "FindEngineLeftCollideTCP" and self.go_to_tcp_pose_prev != "FindEngineLeftCollideTCP":
            self.go_to_tcp_pose_prev = "FindEngineLeftCollideTCP"
            self.FindEngineLeftCollideTCP()

        elif self.go_to_tcp_pose == "FindEngineMidCollideTCP" and self.go_to_tcp_pose_prev != "FindEngineMidCollideTCP":
            self.go_to_tcp_pose_prev = "FindEngineMidCollideTCP"
            self.FindEngineMidCollideTCP()

        else:
            pass

       
    def FindEngineRightCollideTCP(self):
        self.moveL(self.FindEngineRightCollideTCP, a=0.3, v=0.5, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_right_x = self.tcp_x
                self.found_engine_right_y = self.tcp_y
                break
            else:
                pass

    def FindEngineLeftCollideTCP(self):
        self.moveL(self.FindEngineLeftCollideTCPPose, a=0.3, v=0.5, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.found_engine_left_x = self.tcp_x
                self.found_engine_left_y = self.tcp_y
                break
            else:
                pass

    def FindEngineMidCollideTCP(self):
        self.moveL(self.FindEngineMidCollideTCPPose, a=0.3, v=0.5, t=0)
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

        for pose in [self.FindEngineRightDownTCPPose,
            self.FindEngineRightCollideTCPPose, 
            self.FindEngineLeftDownTCPPose,
            self.FindEngineLeftCollideTCPPose,
            self.FindEngineMidDownTCPPose,
            self.FindEngineMiCollideTCPPose,
            self.PreAttachAtlasCloseTCPPose,
            self.AttachAtlasTCPPose,
            self.AAPRAtlasTCPPose,
            self.PreAttachLFToolCloseTCPPose,
            self.AttachLFToolTCPPose,
            self.AAPRLFToolTCPPose,
            self.PreAttachOFToolCloseTCPPose,
            self.AttachOFToolTCPPose,
            self.AAPROFTool1TCPPose,
            self.AAPROFTool2TCPPose,
            self.AboveEngineTCPPose,
            self.FarAboveBoltPair1TCPPose,
            self.CloseAboveBoltPair1TCPPose,
            self.AtBoltPair1TCPPose,
            self.FarAboveBoltPair2TCPPose,
            self.CloseAboveBoltPair2TCPPose,
            self.AtBoltPair2TCPPose,
            self.FarAboveBoltPair3TCPPose,
            self.CloseAboveBoltPair3TCPPose,
            self.AtBoltPair3TCPPose
            #self.FarAboveBoltPair4TCPPose, 
            #self.CloseAboveBoltPair4TCPPose,
            #self.AtBoltPair4TCPPose,
            #self.FarAboveBoltPair5TCPPose, 
            #self.CloseAboveBoltPair5TCPPose,
            #self.AtBoltPair5TCPPose,
            #self.FarAboveBoltPair6TCPPose, 
            #self.CloseAboveBoltPair6TCPPose,
            #self.AtBoltPair6TCPPose,
            #self.FarAboveBoltPair7TCPPose, 
            #self.CloseAboveBoltPair7TCPPose,
            #self.AtBoltPair7TCPPose,
            #self.FarAboveBoltPair8TCPPose, 
            #self.CloseAboveBoltPair8TCPPose,
            #self.AtBoltPair8TCPPose,
            #self.FarAboveBoltPair9TCPPose,
            #self.CloseAboveBoltPair9TCPPose,
            #self.AtBoltPair9TCPPose,
            #self.FarAboveBoltPair10TCPPose,
            #self.CloseAboveBoltPair10TCPPose, 
            #self.AtBoltPair10TCPPose,
            #self.FarAboveBoltPair11TCPPose,
            #self.CloseAboveBoltPair11TCPPose, 
            #self.AtBoltPair11TCPPose,
            #self.FarAboveBoltPair12TCPPose,
            #self.CloseAboveBoltPair12TCPPose, 
            #self.AtBoltPair12TCPPose,
            #self.OFMidpoint1TCPPose,
            #self.OFMidpoint2TCPPose,
            #self.AboveUntightenedOF1TCPPose,
            #self.AtUntightenedOF1TCPPose,
            #self.AtTightenedOF1TCPPose,
            #self.AboveTightenedOF1TCPPose,
            #self.AboveUntightenedOF2TCPPose,
            #self.AtUntightenedOF2TCPPose,
            #self.AtTightenedOF2TCPPose,
            #self.AboveTightenedOF2TCPPose,
            #self.AboveUntightenedOF3TCPPose,
            #self.AtUntightenedOF3TCPPose,
            #self.AtTightenedOF3TCPPose,
            #self.AboveTightenedOF3TCPPose
            ]:

            if  abs((abs(self.tcp_x) - abs(pose[0]))) < 0.001 and\
                abs((abs(self.tcp_y) - abs(pose[1]))) < 0.001 and\
                abs((abs(self.tcp_z) - abs(pose[2]))) < 0.001 and\
                abs((abs(self.tcp_rx) - abs(pose[3]))) < 0.001 and\
                abs((abs(self.tcp_ry) - abs(pose[4]))) < 0.001 and\
                abs((abs(self.tcp_rz) - abs(pose[5]))) < 0.001:
                self.ur_tcp_pose_name = pose
                
            else:
                self.ur_tcp_pose_state = "unknown"

        # This is also awesome...
        for pose2 in self.pose_names:
            tcp_pose2 = "self." + pose2 + "Pose"
            if self.ur_tcp_pose_name == eval(tcp_pose2):
                self.ur_tcp_pose_state = tcp_pose2.replace("self.", "")
            else:
                pass

        #print self.ur_tcp_pose_state
            
                    

if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster()
    except rospy.ROSInterruptException:
        pass
