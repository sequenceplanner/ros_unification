#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Unification Driver
    # V.0.4.0.
    # NOT DONE YET
#----------------------------------------------------------------------------------------

'''
import rospy
import roslib
import struct
from unification_roscontrol.msg import URPoseSPToUni
from unification_roscontrol.msg import URPoseUniToSP
from std_msgs.msg import String
import time



class ur_pose_unidriver():

    def __init__(self):
        
        rospy.init_node('ur_pose_unidriver', anonymous=False)

        self.ur_pose_so_to_unidriver_timeout = 100
        self.ur_pose_smaster_to_unidriver_timeout = 100

        # state
        self.ur_pose_unidriver_got_msg_from_ur_pose_smaster = False
        self.act_pos = "_"
        self.executing = False
        self.planning = False

        # command
        self.ur_pose_unidriver_got_msg_from_sp = False
        self.got_cmd_should_plan = False
        self.got_cmd_ref_pos = "_"

        # publishers
        self.ur_pose_unidriver_to_smaster_publisher = rospy.Publisher('/unification_roscontrol/ur_pose_unidriver_to_smaster', String, queue_size=10)
        self.ur_pose_unidriver_to_sp_publisher = rospy.Publisher('/unification_roscontrol/ur_pose_unidriver_to_sp', URPoseUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    
    def main(self):

        self.ur_pose_state = URPoseUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/unification_roscontrol/ur_pose_sp_to_unidriver", URPoseSPToUni, self.ur_pose_sp_to_unidriver_callback)

                if time.time() < self.ur_pose_sp_to_unidriver_timeout:
                    URPoseSPToUni.ur_pose_unidriver_got_msg_from_sp = self.ur_pose_unidriver_got_msg_from_sp
                    URPoseUniToSP.got_cmd_ref_pos = self.got_cmd_ref_pos
                    URPoseUniToSP.got_cmd_should_plan = self.got_cmd_should_plan
                else:
                    URPoseSPToUni.ur_pose_unidriver_got_msg_from_sp = False
                    URPoseUniToSP.got_cmd_ref_pos = "_"
                    URPoseUniToSP.got_cmd_should_plan = False

            except rospy.ROSInterruptException:
                pass


            try:
                rospy.Subscriber("/unification_roscontrol/ur_pose_smaster_to_unidriver", String, self.mirPoseCallback)

                if time.time() < self.mir_pose_smaster_to_unidriver_timeout:
                    MiRPoseUniToSP.mir_pose_unidriver_got_msg_from_mir_pose_smaster = self.mir_pose_unidriver_got_msg_from_mir_pose_smaster
                    MiRPoseUniToSP.act_pos = self.act_pos

                else:
                    MiRPoseUniToSP.mir_pose_unidriver_got_msg_from_mir_pose_smaster = False
                    MiRPoseUniToSP.act_pos = "_"
                
            except rospy.ROSInterruptException:
                pass

            self.mir_pose_unidriver_to_sp_publisher.publish(self.mir_pose_state)
            self.main_rate.sleep()

        rospy.spin()


    
    def move_mir_to_kitting_station_pos(self):
        self.mir_pose_unidriver_to_smaster_publisher.publish("mir_to_kitting")

    def move_mir_to_pre_assembly_station_pos(self):
        self.mir_pose_unidriver_to_smaster_publisher.publish("mir_to_pre_assembly")

    def move_mir_to_assembly_station_pos(self):
        self.mir_pose_unidriver_to_smaster_publisher.publish("mir_to_assembly")
    
    def move_mir_to_charging_station_pos(self):
        self.mir_pose_unidriver_to_smaster_publisher.publish("mir_to_charge")


    
    def sp_to_mir_pose_unidriver_callback(self, mir_mode_cmd):

        self.sp_to_mir_pose_unidriver_timeout = time.time() + 2
        self.mir_mode_unidriver_got_msg_from_sp = True

        if mir_mode_cmd.ref_pos == "kitting":
            self.got_cmd_ref_pos = "kitting"
            self.move_mir_to_kitting_station_pos()
        
        elif mir_mode_cmd.ref_pos == "preassembly":
            self.got_cmd_ref_pos = "preassembly"
            self.move_mir_to_pre_assembly_station_pos()

        elif mir_mode_cmd.ref_pos == "assembly":
            self.got_cmd_ref_pos = "assembly"
            self.move_mir_to_assembly_station_pos()

        elif mir_mode_cmd.ref_pos == "charging":
            self.got_cmd_ref_pos = "charging"
            self.move_mir_to_charging_station_pos()

        else:
            pass


        
    def mirPoseCallback(self, mir_mode):

        self.mir_pose_smaster_to_unidriver_timeout = time.time() + 2
        self.mir_pose_unidriver_got_msg_from_mir_pose_smaster = True

        if mir_mode.data == "kitting":
            self.act_pos = "kitting"

        elif mir_mode.data == "preassembly":
            self.act_pos = "preassembly"

        elif mir_mode.data == "assembly":
            self.act_pos = "assembly"

        elif mir_mode.data == "charging":
            self.act_pos = "charging"
        
        else:
            self.act_pos = "unknown"


        
if __name__ == '__main__':
    try:
        mir_pose_unidriver()
    except rospy.ROSInterruptException:
        pass
'''