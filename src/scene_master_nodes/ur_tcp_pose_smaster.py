#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR TCP Pose Scene Master
    # V.0.4.0.
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
        self.isclose_tolerance = 0.002
        self.ur_tcp_pose_state = ""
        self.go_to_tcp_pose_prev = ""


        self.collision_right = False
        self.collision_left = False
        self.collision_mid = False
        self.found_right_x  = 0
        self.found_right_y  = 0
        self.found_right_z  = 0
        self.found_right_rx = 0
        self.found_right_ry = 0
        self.found_right_rz = 0
        self.found_left_x  = 0
        self.found_left_y  = 0
        self.found_left_z  = 0
        self.found_left_rx = 0
        self.found_left_ry = 0
        self.found_left_rz = 0
        self.found_mid_x  = 0
        self.found_mid_y  = 0
        self.found_mid_z  = 0
        self.found_mid_rx = 0
        self.found_mid_ry = 0
        self.found_mid_rz = 0
        

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
        self.FindEngineMidCollideTCPPose = [-0.0773473988767, -0.20360053125, 1.01112718356, -2.06825873066, -0.148186823079, 2.20172386733]
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

        self.FarAboveBoltPair1TCPPose = [-0.0784896487942 + 0.005, -0.198313371814, 0.560034029131, -1.50228268223, -1.02180473399, 1.46494776698]
        self.CloseAboveBoltPair1TCPPose = [-0.0907433283046 + 0.005, -0.198958413547, 0.598781784017, -1.42736061959, -1.01986093313, 1.46158494504]
        self.AtBoltPair1TCPPose = [-0.0907433283046 + 0.005, -0.198958413547, 0.598781784017 + 0.015, -1.42736061959, -1.01986093313, 1.46158494504]

        self.FarAboveBoltPair2TCPPose = [-0.170838119192 + 0.005, -0.198953101009, 0.577627098101, -1.42736849013, -1.01971978766, 1.46174461364]
        self.CloseAboveBoltPair2TCPPose = [-0.170857851833 + 0.005, -0.198957378248, 0.598913374413, -1.42736275317, -1.01991528985, 1.46156533033]
        self.AtBoltPair2TCPPose = [-0.170857851833 + 0.005, -0.198957378248, 0.598913374413 + 0.015, -1.42736275317, -1.01991528985, 1.46156533033]

        self.FarAboveBoltPair3TCPPose = [-0.251794736677 + 0.005, -0.19894098752, 0.560131982471, -1.42736357259, -1.0197090458, 1.46161178315]
        self.CloseAboveBoltPair3TCPPose = [-0.251764739397 + 0.005, -0.198932267163, 0.599280223362, -1.42737193363, -1.01978563604, 1.46151771638]
        self.AtBoltPair3TCPPose = [-0.251764739397 + 0.005, -0.198932267163, 0.599280223362 + 0.015, -1.42737193363, -1.01978563604, 1.46151771638]

        self.FarAboveBoltPair4TCPPose = [-0.340180377452 + 0.005, -0.198928051199, 0.550467549074, -1.42746744193, -1.0197481687, 1.46158540087] 
        self.CloseAboveBoltPair4TCPPose = [-0.340216113173 + 0.005, -0.198929428147, 0.591334019607, -1.42742023128, -1.0197121412, 1.46154501591]
        self.AtBoltPair4TCPPose = [-0.340216113173 + 0.005, -0.198929428147, 0.591334019607 + 0.015, -1.42742023128, -1.0197121412, 1.46154501591]

        self.FarAboveBoltPair5TCPPose = [-0.42340282158 + 0.005, -0.198906066596, 0.550469298837, -1.4273482069, -1.01964894581, 1.46156326299] 
        self.CloseAboveBoltPair5TCPPose = [-0.42335253622 + 0.005, -0.198907648281, 0.593303203592, -1.42732440084, -1.01949546511, 1.46162737115]
        self.AtBoltPair5TCPPose = [-0.42335253622 + 0.005, -0.198907648281, 0.593303203592 + 0.015, -1.42732440084, -1.01949546511, 1.46162737115]

        self.FarAboveBoltPair6TCPPose = [-0.504277989694 + 0.005, -0.202548054041, 0.547506465843, -1.44351510935, -1.02205067085, 1.46146647002] 
        self.CloseAboveBoltPair6TCPPose = [-0.504257439148 + 0.005, -0.202509430763, 0.600940289801, -1.44328383836, -1.02178092267, 1.46150454678]
        self.AtBoltPair6TCPPose = [-0.504257439148 + 0.005, -0.202509430763, 0.600940289801 + 0.015, -1.44328383836, -1.02178092267, 1.46150454678]

        self.FarAboveBoltPair7TCPPose = [-0.572056239644 + 0.005, -0.202782081513, 0.56050845392, -1.49338121946, -1.06265925706, 1.42029634963] 
        self.CloseAboveBoltPair7TCPPose = [-0.572022944819 + 0.005, -0.202769953341, 0.603700582544, -1.4934025456, -1.06254790148, 1.42033656683]
        self.AtBoltPair7TCPPose = [-0.572022944819 + 0.005, -0.202769953341, 0.603700582544 + 0.015, -1.4934025456, -1.06254790148, 1.42033656683]

        self.FarAboveBoltPair8TCPPose = [-0.671600439824 + 0.005, -0.205083851779, 0.552170642121, -1.43285585336, -1.01947436425, 1.46210150502] 
        self.CloseAboveBoltPair8TCPPose = [-0.67164013522 + 0.005, -0.205040955238, 0.595909217147, -1.4329338934, -1.01950505222, 1.46198608899]
        self.AtBoltPair8TCPPose = [-0.67164013522 + 0.005, -0.205040955238, 0.595909217147 + 0.015, -1.4329338934, -1.01950505222, 1.46198608899]

        self.FarAboveBoltPair9TCPPose = [-0.756337995441 + 0.005, -0.205085148431, 0.554773477302, -1.43290604118, -1.01957035893, 1.46202542067] 
        self.CloseAboveBoltPair9TCPPose = [-0.756362925345 + 0.005, -0.205054927386, 0.594087198338, -1.43292290748, -1.0197174755, 1.46192571169]
        self.AtBoltPair9TCPPose = [-0.756362925345 + 0.005, -0.205054927386, 0.594087198338 + 0.015, -1.43292290748, -1.0197174755, 1.46192571169]

        self.FarAboveBoltPair10TCPPose = [-0.842457953839 + 0.005, -0.206941399681, 0.550707611477, -1.42751773056, -1.01930792912, 1.46186231147] 
        self.CloseAboveBoltPair10TCPPose = [-0.842439483256 + 0.005, -0.207014149385, 0.598627578333, -1.4274069005, -1.01975010409, 1.46153660082]
        self.AtBoltPair10TCPPose = [-0.842439483256 + 0.005, -0.207014149385, 0.598627578333 + 0.015, -1.4274069005, -1.01975010409, 1.46153660082]

        self.FarAboveBoltPair11TCPPose = [-0.925281939518 + 0.005, -0.207054574962, 0.543830160168, -1.42867750271, -1.01862144078, 1.46310135916] 
        self.CloseAboveBoltPair11TCPPose = [-0.925246708399 + 0.005, -0.207001465152, 0.592315204751, -1.42843417652, -1.01862363469, 1.46275547977]
        self.AtBoltPair11TCPPose = [-0.925246708399 + 0.005, -0.207001465152, 0.592315204751 + 0.015, -1.42843417652, -1.01862363469, 1.46275547977]

        self.FarAboveBoltPair12TCPPose = [-0.997709877071 + 0.005, -0.206914907744, 0.546649492367, -1.42969212723, -1.01763850947, 1.46391510099] 
        self.CloseAboveBoltPair12TCPPose = [ -0.997671436660 + 0.005, -0.20692410677, 0.594941151461, -1.42948482006, -1.0177939519, 1.46375133908]
        self.AtBoltPair12TCPPose = [ -0.997671436660 + 0.005, -0.20692410677, 0.594941151461 + 0.015, -1.42948482006, -1.0177939519, 1.46375133908]
        #----------------------------------------------------------------------------------------





        self.pose_names = ["FindEngineRightDownTCP",
            #"FindEngineRightCollideTCP", 
            "FindEngineLeftDownTCP",
            #"FindEngineLeftCollideTCP",
            "FindEngineMidDownTCP",
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
            "AtBoltPair3TCP",
            "FarAboveBoltPair4TCP",
            "CloseAboveBoltPair4TCP",
            "AtBoltPair4TCP",
            "FarAboveBoltPair5TCP",
            "CloseAboveBoltPair5TCP",
            "AtBoltPair5TCP",
            "FarAboveBoltPair6TCP",
            "CloseAboveBoltPair6TCP",
            "AtBoltPair6TCP",
            "FarAboveBoltPair7TCP",
            "CloseAboveBoltPair7TCP",
            "AtBoltPair7TCP",
            "FarAboveBoltPair8TCP",
            "CloseAboveBoltPair8TCP",
            "AtBoltPair8TCP",
            "FarAboveBoltPair9TCP",
            "CloseAboveBoltPair9TCP",
            "AtBoltPair9TCP",
            "FarAboveBoltPair10TCP",
            "CloseAboveBoltPair10TCP", 
            "AtBoltPair10TCP",
            "FarAboveBoltPair11TCP",
            "CloseAboveBoltPair11TCP", 
            "AtBoltPair11TCP",
            "FarAboveBoltPair12TCP",
            "CloseAboveBoltPair12TCP", 
            "AtBoltPair12TCP"
            ]



        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(5)

        self.main()

    
    def moveL(self, tcpPose, a=0.1, v=0.2, t=0):
        if len(tcpPose) == 6:
            script_str = "movel(p" + str(tcpPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            print "The tcp pose size is not correct."


    def main(self):
        #self.FindEngineMid3TCP()

        while not rospy.is_shutdown():        
    
            self.ur_tcp_pose_to_unidriver_publisher.publish(self.ur_tcp_pose_state)
            #print self.FindEngineLeftCollideTCPPose

            

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
                method = "self.moveL(" + tcp_pose + ", a=0.1, v=0.2, t=0)"
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
        self.moveL(self.FindEngineRightCollideTCPPose, a=0.3, v=0.05, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.collision_right = True
                self.ur_tcp_pose_state = "FindEngineRightCollideTCP"
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineRightCollideTCP")
                rospy.sleep(2)
                self.found_right_x = self.tcp_x
                self.found_right_y = self.tcp_y
                self.found_right_z = self.tcp_z
                self.found_right_rx = self.tcp_rx
                self.found_right_ry = self.tcp_ry
                self.found_right_rz = self.tcp_rz
                break
            else:
                pass

    def FindEngineLeftCollideTCP(self):
        self.moveL(self.FindEngineLeftCollideTCPPose, a=0.3, v=0.05, t=0)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.collision_left = True 
                self.ur_tcp_pose_state = "FindEngineLeftCollideTCP"
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineLeftCollideTCP")
                rospy.sleep(2)
                self.found_left_x = self.tcp_x
                self.found_left_y = self.tcp_y
                self.found_left_z = self.tcp_z
                self.found_left_rx = self.tcp_rx
                self.found_left_ry = self.tcp_ry
                self.found_left_rz = self.tcp_rz
                break
            else:
                pass

    def FindEngineMidCollideTCP(self):
        self.moveL(self.FindEngineMidCollideTCPPose, a=0.3, v=0.05, t=0)
        rospy.sleep(1)
        while True:
            if self.z < -25:
                self.urScriptPublisher.publish("stopl(a=5)")
                self.collision_mid = True 
                self.ur_tcp_pose_state = "FindEngineMidCollideTCP"
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                self.ur_tcp_pose_to_unidriver_publisher.publish("FindEngineMidCollideTCP")
                rospy.sleep(2)
                self.found_mid_x = self.tcp_x
                self.found_mid_y = self.tcp_y
                self.found_mid_z = self.tcp_z
                self.found_mid_rx = self.tcp_rx
                self.found_mid_ry = self.tcp_ry
                self.found_mid_rz = self.tcp_rz
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

        #print()

        if self.collision_right == False:
            self.FindEngineRightCollideTCPPose = [-0.476111259039, -0.4, 1.02100659443, -1.57516026151, 0.299218018579, 0.364387736323]
        else:
            self.FindEngineRightCollideTCPPose = [self.found_right_x, self.found_right_y, self.found_right_z, self.found_right_rx, self.found_right_ry, self.found_right_rz]

        if self.collision_left == False:
            self.FindEngineLeftCollideTCPPose = [-0.870853950336, 0.0, 1.00801072426, 1.1391753846, 1.16284516465, -1.2144095644]
        else:
            self.FindEngineLefCollideTCPPose = [self.found_left_x, self.found_left_y, self.found_left_z, self.found_left_rx, self.found_left_ry, self.found_left_rz]

        if self.collision_mid == False:
            self.FindEngineMidCollideTCPPose = [-0.0773473988767, -0.20360053125, 1.01112718356, -2.06825873066, -0.148186823079, 2.20172386733]
        else:
            self.FindEngineMidCollideTCPPose = [self.found_mid_x, self.found_mid_y, self.found_mid_z, self.found_mid_rx, self.found_mid_ry, self.found_mid_rz]

        for pose in [self.FindEngineRightDownTCPPose, 
            self.FindEngineLeftDownTCPPose,
            self.FindEngineMidDownTCPPose,
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
            self.AtBoltPair3TCPPose,
            self.FarAboveBoltPair4TCPPose, 
            self.CloseAboveBoltPair4TCPPose,
            self.AtBoltPair4TCPPose,
            self.FarAboveBoltPair5TCPPose, 
            self.CloseAboveBoltPair5TCPPose,
            self.AtBoltPair5TCPPose,
            self.FarAboveBoltPair6TCPPose, 
            self.CloseAboveBoltPair6TCPPose,
            self.AtBoltPair6TCPPose,
            self.FarAboveBoltPair7TCPPose, 
            self.CloseAboveBoltPair7TCPPose,
            self.AtBoltPair7TCPPose,
            self.FarAboveBoltPair8TCPPose, 
            self.CloseAboveBoltPair8TCPPose,
            self.AtBoltPair8TCPPose,
            self.FarAboveBoltPair9TCPPose,
            self.CloseAboveBoltPair9TCPPose,
            self.AtBoltPair9TCPPose,
            self.FarAboveBoltPair10TCPPose,
            self.CloseAboveBoltPair10TCPPose, 
            self.AtBoltPair10TCPPose,
            self.FarAboveBoltPair11TCPPose,
            self.CloseAboveBoltPair11TCPPose, 
            self.AtBoltPair11TCPPose,
            self.FarAboveBoltPair12TCPPose,
            self.CloseAboveBoltPair12TCPPose, 
            self.AtBoltPair12TCPPose,
            ]:

            #if  abs((abs(self.tcp_x) - abs(self.FindEngineRightCollideTCPPose[0]))) < 0.003 and\
            #    abs((abs(self.tcp_y) - abs(self.FindEngineRightCollideTCPPose[1]))) < 0.003 and\
            #    abs((abs(self.tcp_z) - abs(self.FindEngineRightCollideTCPPose[2]))) < 0.003 and\
            #    abs((abs(self.tcp_rx) - abs(self.FindEngineRightCollideTCPPose[3]))) < 0.003 and\
            ##    abs((abs(self.tcp_ry) - abs(self.FindEngineRightCollideTCPPose[4]))) < 0.003 and\
            #    abs((abs(self.tcp_rz) - abs(self.FindEngineRightCollideTCPPose[5]))) < 0.003:
            #    self.ur_tcp_pose_state = "FindEngineRightCollideTCP"#

            #elif  abs((abs(self.tcp_x) - abs(self.FindEngineLeftCollideTCPPose[0]))) < 0.003 and\
            #    abs((abs(self.tcp_y) - abs(self.FindEngineLeftCollideTCPPose[1]))) < 0.003 and\
            #    abs((abs(self.tcp_z) - abs(self.FindEngineLeftCollideTCPPose[2]))) < 0.003 and\
            #    abs((abs(self.tcp_rx) - abs(self.FindEngineLeftCollideTCPPose[3]))) < 0.003 and\
            #    abs((abs(self.tcp_ry) - abs(self.FindEngineLeftCollideTCPPose[4]))) < 0.003 and\
            #    abs((abs(self.tcp_rz) - abs(self.FindEngineLeftCollideTCPPose[5]))) < 0.003:
            #    self.ur_tcp_pose_state = "FindEngineLeftCollideTCP"#

            #elif  abs((abs(self.tcp_x) - abs(self.FindEngineMidCollideTCPPose[0]))) < 0.003 and\
            #    abs((abs(self.tcp_y) - abs(self.FindEngineMidCollideTCPPose[1]))) < 0.003 and\
            #    abs((abs(self.tcp_z) - abs(self.FindEngineMidCollideTCPPose[2]))) < 0.003 and\
            #    abs((abs(self.tcp_rx) - abs(self.FindEngineMidCollideTCPPose[3]))) < 0.003 and\
            #    abs((abs(self.tcp_ry) - abs(self.FindEngineMidCollideTCPPose[4]))) < 0.003 and\
            #    abs((abs(self.tcp_rz) - abs(self.FindEngineMidCollideTCPPose[5]))) < 0.003:
            #    self.ur_tcp_pose_state = "FindEngineMidCollideTCP"            


            if  abs((abs(self.tcp_x) - abs(pose[0]))) < 0.003 and\
                abs((abs(self.tcp_y) - abs(pose[1]))) < 0.003 and\
                abs((abs(self.tcp_z) - abs(pose[2]))) < 0.003 and\
                abs((abs(self.tcp_rx) - abs(pose[3]))) < 0.003 and\
                abs((abs(self.tcp_ry) - abs(pose[4]))) < 0.003 and\
                abs((abs(self.tcp_rz) - abs(pose[5]))) < 0.003:
                self.ur_tcp_pose_name = pose

            else:
                self.ur_tcp_pose_state = "unknown"

        # This is also awesome...
        for pose2 in self.pose_names:
            tcp_pose2 = "self." + pose2 + "Pose"
            if self.ur_tcp_pose_name == eval(tcp_pose2):
                ur_tcp_pose_state_a = tcp_pose2.replace("self.", "")
                self.ur_tcp_pose_state = ur_tcp_pose_state_a.replace("Pose", "")
            else:
                pass

        #print self.ur_tcp_pose_state
            
                    

if __name__ == '__main__':
    try:
        ur_tcp_pose_smaster()
    except rospy.ROSInterruptException:
        pass
