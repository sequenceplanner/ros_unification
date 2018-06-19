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
from std_msgs.msg import String
import time



class tester():

    def __init__(self):
        
        rospy.init_node('tester', anonymous=False)

        self.cmd = URPoseSPToUni()

        self.posepub = rospy.Publisher("unification_roscontrol/ur_pose_sp_to_unidriver", URPoseSPToUni, queue_size=10)
        # for now... real later
        self.airpub = rospy.Publisher("CT_RECU_con", String, queue_size=10)
        
        rospy.sleep(2)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    
    def main(self):

        
        self.releaseRSP()
        rospy.sleep(1)
        self.lockRSP()
        rospy.sleep(1)
        self.releaseRSP()

        #self.moveToResetJOINT()

        self.Should = False
        self.Pose = 'ResetJOINT'
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

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'AttachLFToolTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachLFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)

        self.Should = False
        self.Pose = 'PreAttachLFToolFarJOINT'
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

        rospy.sleep(4)

        

        '''
        self.Should = False
        self.Pose = 'PreAttachLFToolFarJOINT'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)

        rospy.sleep(4)
        '''
        
        '''
        self.Should = False
        self.Pose = 'PreAttachLFToolCloseTCP'
        URPoseSPToUni.should_plan = self.Should
        URPoseSPToUni.ref_pos = self.Pose
        self.posepub.publish(self.cmd)
        '''

        rospy.sleep(1)
        #self.moveToHomeJOINT()
        rospy.sleep(4)
        #self.moveToAtlasFarJOINT()

        rospy.spin()


    '''
    def moveToAtlasFarJOINT(self):
        cmd = URPoseSPToUni2()
        URPoseSPToUni2.should_plan = False
        URPoseSPToUni2.ref_pos = 'PreAttachAtlasFarJOINT'
        self.posepub.publish(cmd)
    '''


    def releaseRSP(self):
        self.airpub.publish("release_rsp")

    def lockRSP(self):
        self.airpub.publish("lock_rsp")
    
    '''
    def releaseRSP(self):
        cmd = RecuSPToUni()
        RecuSPToUni.lock_rsp = False
        RecuSPToUni.release_rsp = True
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.airpub.publish(cmd)

    def lockRSP(self):
        cmd = RecuSPToUni()
        RecuSPToUni.lock_rsp = True
        RecuSPToUni.release_rsp = False
        RecuSPToUni.open_gripper = False
        RecuSPToUni.close_gripper = False
        self.airpub.publish(cmd)
    '''

if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass
