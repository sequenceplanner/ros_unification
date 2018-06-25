#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Unification Driver
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from unification_roscontrol.msg import ExecutorToSP
from unification_roscontrol.msg import SPToExecutor
from unification_roscontrol.msg import URPoseSPToUni
from unification_roscontrol.msg import URPoseUniToSP
from unification_roscontrol.msg import URModeSPToUni
from unification_roscontrol.msg import URModeUniToSP
from unification_roscontrol.msg import AecuSPToUni
from unification_roscontrol.msg import AecuUniToSP
from unification_roscontrol.msg import RecuSPToUni
from unification_roscontrol.msg import RecuUniToSP
from unification_roscontrol.msg import MiRModeSPToUni
from unification_roscontrol.msg import MiRModeUniToSP
from unification_roscontrol.msg import MiRPoseSPToUni
from unification_roscontrol.msg import MiRModeUniToSP
from unification_roscontrol.msg import HecuUniToSP
from unification_roscontrol.msg import AGVAlvarUniToSP
from std_msgs.msg import String
import os
import threading
import time



class unification_executor():

    def __init__(self):
        
        rospy.init_node('unification_executor', anonymous=False)

        self.executor_to_sp = ExecutorToSP()
        self.ur_pose_sp_to_uni = URPoseSPToUni()
        self.ur_mode_sp_to_uni = URModeSPToUni()
        self.aecu_sp_to_uni = AecuSPToUni()
        self.recu_sp_to_uni = RecuSPToUni()
        self.mir_pose_sp_to_uni = MiRPoseSPToUni()
        self.mir_mode_sp_to_uni = MiRModeSPToUni()

        self.should_plan = False
        self.seq_state = 0
        self.done = False
        self.executing = False
        self.cmd = ""
        self.act_pos = ""
        self.ref_pos = ""
        self.got_cmd = ""

        # subscribers
        rospy.Subscriber("/unification_roscontrol/sp_to_executor", SPToExecutor, self.sp_to_executor_callback)
        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_sp", URPoseUniToSP, self.ur_pose_unidriver_to_sp_callback)
        #rospy.Subscriber("/unification_roscontrol/ur_mode_unidriver_to_sp", URModeUniToSP, self.ur_mode_unidriver_to_sp_callback)
        #rospy.Subscriber("/unification_roscontrol/aecu_unidriver_to_sp", AecuUniToSP, self.aecu_unidriver_to_sp_callback)
        #rospy.Subscriber("/unification_roscontrol/recu_unidriver_to_sp", RecuUniToSP, self.recu_unidriver_to_sp_callback)
        #rospy.Subscriber("/unification_roscontrol/hecu_unidriver_to_sp", HecuUniToSP, self.hecu_unidriver_to_sp_callback)
        # rospy.Subscriber("/unification_roscontrol/agv_alvar_unidriver_to_sp", AGVAlvarUniToSP, self.agv_alvar_unidriver_to_sp_callback)
        # rospy.Subscriber("/unification_roscontrol/mir_pose_unidriver_to_sp", MiRPoseUniToSP, self.mir_pose_unidriver_to_sp_callback)
        # rospy.Subscriber("/unification_roscontrol/mir_mode_unidriver_to_sp", MiRModeUniToSP, self.mir_mode_unidriver_to_sp_callback))

        # publishers
        self.ExecutorToSPPublisher = rospy.Publisher("unification_roscontrol/executor_to_sp", ExecutorToSP, queue_size=10)
        self.URPoseSPToUniPublisher = rospy.Publisher("unification_roscontrol/ur_pose_sp_to_unidriver", URPoseSPToUni, queue_size=10)
        self.URModeSPToUniPublisher = rospy.Publisher("unification_roscontrol/ur_mode_sp_to_unidriver", URModeSPToUni, queue_size=10)
        self.AecuSPToUniPublisher = rospy.Publisher("unification_roscontrol/aecu_sp_to_unidriver", AecuSPToUni, queue_size=10)
        self.RecuSPToUniPublisher = rospy.Publisher("unification_roscontrol/recu_sp_to_unidriver", RecuSPToUni, queue_size=10)
        self.MiRModeSPToUnPublisher = rospy.Publisher("unification_roscontrol/mir_mode_sp_to_unidriver", MiRModeSPToUni, queue_size=10)
        self.MiRPoseSPToUnPublisher = rospy.Publisher("unification_roscontrol/mir_pose_sp_to_unidriver", MiRPoseSPToUni, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()

    def run_picknplace(self):
        def callback_run_picknplace():
            os.system('rosrun unification_roscontrol picknplace.py')
        t1 = threading.Thread(target=callback_run_picknplace)
        t1.daemon = True
        t1.start()

    
    def main(self):

        #self.run_picknplace()

        while not rospy.is_shutdown():

            # --------------------------------------------------------------------------------
            # Attaching the LF Tool seq
            # --------------------------------------------------------------------------------

            if self.cmd == "AttachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachLFToolCloseTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1

            elif self.cmd == "AttachLFTool" and self.act_pos == "PreAttachLFToolCloseTCP" and self.seq_state == 1:
                self.unlockRSP()
                self.seq_state = 2

            elif self.cmd == "AttachLFTool" and self.act_pos == "PreAttachLFToolCloseTCP" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachLFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "AttachLFTool" and self.act_pos == "AttachLFToolTCP" and self.seq_state == 3:
                self.lockRSP()
                self.openGripper()
                self.seq_state = 4

            elif self.cmd == "AttachLFTool" and self.act_pos == "AttachLFToolTCP" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPRLFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5
            
            elif self.cmd == "AttachLFTool" and self.act_pos == "AAPRLFToolTCP" and self.seq_state == 5:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachLFToolFarJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 6

            elif self.cmd == "AttachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 6:
                self.closeGripper()
                self.seq_state = 7

            elif self.cmd == "AttachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 7:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass

            
            # --------------------------------------------------------------------------------
            # Detach the LF Tool seq
            # --------------------------------------------------------------------------------

            if self.cmd == "DetachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                self.openGripper()
                self.seq_state = 1

            elif self.cmd == "DetachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 1:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPRLFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 2

            elif self.cmd == "DetachLFTool" and self.act_pos == "AAPRLFToolTCP" and self.seq_state == 2:
                self.closeGripper()
                self.seq_state = 3

            elif self.cmd == "DetachLFTool" and self.act_pos == "AAPRLFToolTCP" and self.seq_state == 3:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachLFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 4

            elif self.cmd == "DetachLFTool" and self.act_pos == "AttachLFToolTCP" and self.seq_state == 4:
                self.unlockRSP()
                self.seq_state = 5
            
            elif self.cmd == "DetachLFTool" and self.act_pos == "AttachLFToolTCP" and self.seq_state == 5:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachLFToolFarJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 6

            elif self.cmd == "DetachLFTool" and self.act_pos == "PreAttachLFToolFarJOINT" and self.seq_state == 6:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass



            # --------------------------------------------------------------------------------
            # Attaching the OF Tool seq
            # --------------------------------------------------------------------------------


            if self.cmd == "AttachOFTool" and self.act_pos == "PreAttachOFToolFarJOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachOFToolCloseTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1

            elif self.cmd == "AttachOFTool" and self.act_pos == "PreAttachOFToolCloseTCP" and self.seq_state == 1:
                self.unlockRSP()
                self.seq_state = 2

            elif self.cmd == "AttachOFTool" and self.act_pos == "PreAttachOFToolCloseTCP" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachOFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "AttachOFTool" and self.act_pos == "AttachOFToolTCP" and self.seq_state == 3:
                self.lockRSP()
                rospy.sleep(1)
                self.seq_state = 4

            elif self.cmd == "AttachOFTool" and self.act_pos == "AttachOFToolTCP" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPROFTool1TCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5
            
            elif self.cmd == "AttachOFTool" and self.act_pos == "AAPROFTool1TCP" and self.seq_state == 5:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPROFTool2TCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 6

            elif self.cmd == "AttachOFTool" and self.act_pos == "AAPROFTool2TCP" and self.seq_state == 6:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachOFToolFarJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 7

            elif self.cmd == "AttachOFTool" and self.act_pos == "PreAttachOFToolFarJOINT" and self.seq_state == 7:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass




            # --------------------------------------------------------------------------------
            # Detaching the OF Tool seq
            # --------------------------------------------------------------------------------


            if self.cmd == "DetachOFTool" and self.act_pos == "PreAttachOFToolFarJOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPROFTool2TCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1

            elif self.cmd == "DetachOFTool" and self.act_pos == "AAPROFTool2TCP" and self.seq_state == 1:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPROFTool1TCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 2

            elif self.cmd == "DetachOFTool" and self.act_pos == "AAPROFTool1TCP" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachOFToolTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "DetachOFTool" and self.act_pos == "AttachOFToolTCP" and self.seq_state == 3:
                self.unlockRSP()
                self.seq_state = 4

            elif self.cmd == "DetachOFTool" and self.act_pos == "AttachOFToolTCP" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachOFToolFarJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5

            elif self.cmd == "DetachOFTool" and self.act_pos == "PreAttachOFToolFarJOINT" and self.seq_state == 5:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass



            # --------------------------------------------------------------------------------
            # Attaching the Atlas Tool seq
            # --------------------------------------------------------------------------------

            if self.cmd == "AttachAtlas" and self.act_pos == "PreAttachAtlasFarJOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachAtlasCloseTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1

            elif self.cmd == "AttachAtlas" and self.act_pos == "PreAttachAtlasCloseTCP" and self.seq_state == 1:
                self.unlockRSP()
                self.seq_state = 2

            elif self.cmd == "AttachAtlas" and self.act_pos == "PreAttachAtlasCloseTCP" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachAtlasTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "AttachAtlas" and self.act_pos == "AttachAtlasTCP" and self.seq_state == 3:
                self.lockRSP()
                rospy.sleep(1)
                self.seq_state = 4

            elif self.cmd == "AttachAtlas" and self.act_pos == "AttachAtlasTCP" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AAPRAtlasTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5
            
            elif self.cmd == "AttachAtlas" and self.act_pos == "AAPRAtlasTCP" and self.seq_state == 5:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass


            # --------------------------------------------------------------------------------
            # Detaching the Atlas Tool seq
            # --------------------------------------------------------------------------------

            if self.cmd == "DetachAtlas" and self.act_pos == "AAPRAtlasTCP" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AttachAtlasTCP"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1

            elif self.cmd == "DetachAtlas" and self.act_pos == "AttachAtlasTCP" and self.seq_state == 1:
                self.unlockRSP()
                self.seq_state = 2

            elif self.cmd == "DetachAtlas" and self.act_pos == "AttachAtlasTCP" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "PreAttachAtlasFarJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "DetachAtlas" and self.act_pos == "PreAttachAtlasFarJOINT" and self.seq_state == 3:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass

            # --------------------------------------------------------------------------------
            # OF1 seq
            # --------------------------------------------------------------------------------

            if self.cmd == "TightenOF1" and self.act_pos == "OFMidpoint2JOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AboveUntightenedOF1JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1
            
            elif self.cmd == "TightenOF1" and self.act_pos == "AboveUntightenedOF1JOINT" and self.seq_state == 1:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AtUntightenedOF1JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 2
            
            elif self.cmd == "TightenOF1" and self.act_pos == "AtUntightenedOF1JOINT" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AtTightenedOF1JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "TightenOF1" and self.act_pos == "AtTightenedOF1JOINT" and self.seq_state == 3:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AboveTightenedOF1JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 4

            elif self.cmd == "TightenOF1" and self.act_pos == "AboveTightenedOF1JOINT" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "OFMidpoint2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5

            elif self.cmd == "TightenOF1" and self.act_pos == "OFMidpoint2JOINT" and self.seq_state == 5:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass

            
            # --------------------------------------------------------------------------------
            # OF2 seq
            # --------------------------------------------------------------------------------

            if self.cmd == "TightenOF2" and self.act_pos == "OFMidpoint2JOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AboveUntightenedOF2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 1
            
            elif self.cmd == "TightenOF2" and self.act_pos == "AboveUntightenedOF2JOINT" and self.seq_state == 1:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AtUntightenedOF2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 2
            
            elif self.cmd == "TightenOF2" and self.act_pos == "AtUntightenedOF2JOINT" and self.seq_state == 2:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AtTightenedOF2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 3

            elif self.cmd == "TightenOF2" and self.act_pos == "AtTightenedOF2JOINT" and self.seq_state == 3:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AboveTightenedOF2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 4

            elif self.cmd == "TightenOF2" and self.act_pos == "AboveTightenedOF2JOINT" and self.seq_state == 4:
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "OFMidpoint2JOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                self.seq_state = 5

            elif self.cmd == "TightenOF2" and self.act_pos == "OFMidpoint2JOINT" and self.seq_state == 5:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass



            # --------------------------------------------------------------------------------
            # LFseq
            # --------------------------------------------------------------------------------
            
            if self.cmd == "LFMagic" and self.act_pos == "LFOperationMidpoint5JOINT" and self.seq_state == 0 and self.done == False:
                rospy.sleep(2)
                self.executing = True
                self.got_cmd = self.cmd
                self.done = False
                self.run_picknplace()
                self.seq_state = 1
            
            elif self.cmd == "LFMagic" and self.act_pos == "AfterLFOperationJOINT" and self.seq_state == 1:
                self.executing = False
                self.got_cmd = self.cmd
                self.done = True

            elif self.cmd == "" and self.done == True:
                self.seq_state = 0
                self.got_cmd = self.cmd
                self.done = False

            else:
                pass



            ExecutorToSP.got_cmd = self.got_cmd
            ExecutorToSP.executing = self.executing
            ExecutorToSP.done = self.done

            self.main_rate.sleep()

            self.ExecutorToSPPublisher.publish(self.executor_to_sp)
           
        rospy.spin()



            


    def unlockRSP(self):
        rospy.sleep(1)
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = True
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
        

    def lockRSP(self):
        rospy.sleep(1)
        RecuSPToUni.lock_rsp = True
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)

    def openGripper(self):
        rospy.sleep(1)
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = True
        RecuSPToUni.close_gripper = False
        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)

    def closeGripper(self):
        rospy.sleep(1)
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = True
        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)



    def sp_to_executor_callback(self, sp_cmd):
        
        self.cmd = sp_cmd.cmd

    def ur_pose_unidriver_to_sp_callback(self, ur_pose):

        self.act_pos = ur_pose.act_pos


   

if __name__ == '__main__':
    try:
        unification_executor()
    except rospy.ROSInterruptException:
        pass