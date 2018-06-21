#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # tester
    # V.0.6.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from unification_roscontrol.msg import URPoseSPToUni
from unification_roscontrol.msg import RecuSPToUni
from unification_roscontrol.msg import AecuSPToUni
from std_msgs.msg import String
import time



class tester():

    def __init__(self):
        
        rospy.init_node('tester', anonymous=False)

        self.cmd = URPoseSPToUni()
        self.air_cmd = RecuSPToUni()
        self.aecu_cmd = AecuSPToUni()

        self.posepub = rospy.Publisher("unification_roscontrol/ur_pose_sp_to_unidriver", URPoseSPToUni, queue_size=10)
        # for now... real later
        self.airpub = rospy.Publisher("unification_roscontrol/recu_sp_to_unidriver", RecuSPToUni, queue_size=10)
        self.aecupub = rospy.Publisher("unification_roscontrol/aecu_sp_to_unidriver", AecuSPToUni, queue_size=10)

        rospy.sleep(2)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    
    def main(self):

        self.unlockRSP()
        rospy.sleep(1)
        self.lockRSP()
        rospy.sleep(1)
        self.unlockRSP()

        #self.moveToResetJOINT()

        self.Should = False
        self.Pose = 'reset'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(1)

        self.Should = False
        self.Pose = 'HomeJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        '''
        self.Should = False
        self.Pose = 'PreAttachLFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachLFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(3)

        self.Should = False
        self.Pose = 'AttachLFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)


        rospy.sleep(1)

        self.openGripper()

        rospy.sleep(1)

        self.Should = False
        self.Pose = 'AAPRLFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(2)


        self.Should = False
        self.Pose = 'reset'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        self.Should = False
        self.Pose = 'PreAttachLFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)


        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AAPRLFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(2)

        self.closeGripper()

        rospy.sleep(2)

        self.unlockRSP()

        rospy.sleep(2)


        self.Should = False
        self.Pose = 'reset'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        '''

        rospy.sleep(1)

        self.Should = False
        self.Pose = 'PreAttachOFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachOFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AttachOFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(3)

        self.lockRSP()

        rospy.sleep(2)

        self.Should = False
        self.Pose = 'AAPROFTool1TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AAPROFTool2TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        '''

        self.Should = False
        self.Pose = 'AAPROFTool1TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AttachOFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(3)

        self.unlockRSP()

        rospy.sleep(2)

        self.Should = False
        self.Pose = 'reset'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(2)

        self.Should = False
        self.Pose = 'PreAttachOFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'HomeJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachAtlasCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AttachAtlasTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(2)

        self.lockRSP()

        rospy.sleep(2)


        self.Should = False
        self.Pose = 'AboveEngineTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'FarAboveBoltPair1TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'CloseAboveBoltPair1TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.runToolForward()

        rospy.sleep(1)

        self.Should = False
        self.Pose = 'AtBoltPair1TCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.setToolIdle()

        rospy.sleep(1)

        '''






        

        '''
        self.Should = False
        self.Pose = 'PreAttachOFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachOFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AttachOFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachOFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachOFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'HomeJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)
        '''

        rospy.sleep(4)

        

    def unlockRSP(self):
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = True
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.airpub.publish(self.air_cmd)

    def lockRSP(self):
        RecuSPToUni.lock_rsp = True
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.airpub.publish(self.air_cmd)

    def openGripper(self):
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = True
        RecuSPToUni.close_gripper = False
        self.airpub.publish(self.air_cmd)

    def closeGripper(self):
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.unlock_rsp = False
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = True
        self.airpub.publish(self.air_cmd)

    def setToolIdle(self):
        AecuSPToUni.set_tool_idle = True
        AecuSPToUni.run_tool_forward = False
        AecuSPToUni.run_tool_in_reverse = False
        AecuSPToUni.inhibit_all_run_also_manual = False
        AecuSPToUni.activate_unload = False
        AecuSPToUni.activate_lift = False
        self.aecupub.publish(self.aecu_cmd)

    def runToolForward(self):
        AecuSPToUni.set_tool_idle = False
        AecuSPToUni.run_tool_forward = True
        AecuSPToUni.run_tool_in_reverse = False
        AecuSPToUni.inhibit_all_run_also_manual = False
        AecuSPToUni.activate_unload = False
        AecuSPToUni.activate_lift = False
        self.aecupub.publish(self.aecu_cmd)

    def activateUnload(self):
        AecuSPToUni.set_tool_idle = False
        AecuSPToUni.run_tool_forward = False
        AecuSPToUni.run_tool_in_reverse = False
        AecuSPToUni.inhibit_all_run_also_manual = False
        AecuSPToUni.activate_unload = True
        AecuSPToUni.activate_lift = True
        self.aecupub.publish(self.aecu_cmd)

    def activateLift(self):
        AecuSPToUni.set_tool_idle = False
        AecuSPToUni.run_tool_forward = False
        AecuSPToUni.run_tool_in_reverse = False
        AecuSPToUni.inhibit_all_run_also_manual = False
        AecuSPToUni.activate_unload = False
        AecuSPToUni.activate_lift = True
        self.aecupub.publish(self.aecu_cmd)

    

if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass
