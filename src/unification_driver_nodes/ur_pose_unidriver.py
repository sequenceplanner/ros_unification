#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Unification Driver
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from unification_roscontrol.msg import URPoseSPToUni
from unification_roscontrol.msg import URPoseUniToSP
from unification_roscontrol.msg import URJointSmasterToUni
from std_msgs.msg import String
import time



class ur_pose_unidriver():

    def __init__(self):
        
        rospy.init_node('ur_pose_unidriver', anonymous=False)

        self.ur_pose_sp_to_unidriver_timeout = 100
        self.ur_tcp_pose_smaster_to_unidriver_timeout = 100
        self.ur_joint_pose_smaster_to_unidriver_timeout = 100
        self.moveit_smaster_to_unidriver_timeout = 100

        # state
        self.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = False
        self.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster = False
        self.ur_pose_unidriver_got_msg_from_moveit_smaster = False
        self.act_pos = "unknown"
        self.act_tcp_pos = "unknown"
        self.act_joint_pos = "unknown"
        self.executing = False
        self.planning = False

        # command
        self.ur_pose_unidriver_got_msg_from_sp = False
        self.got_cmd_should_plan = False
        self.got_cmd_ref_pos = "_"

        # publishers
        self.ur_pose_unidriver_to_ur_tcp_pose_smaster_publisher = rospy.Publisher('/unification_roscontrol/ur_pose_unidriver_to_ur_tcp_pose_smaster', String, queue_size=10)
        self.ur_pose_unidriver_to_ur_joint_pose_smaster_publisher = rospy.Publisher('/unification_roscontrol/ur_pose_unidriver_to_ur_joint_pose_smaster', String, queue_size=10)
        self.ur_pose_unidriver_to_moveit_smaster_publisher = rospy.Publisher('/unification_roscontrol/ur_pose_unidriver_to_moveit_smaster', String, queue_size=10)
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
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_sp = self.ur_pose_unidriver_got_msg_from_sp
                    URPoseUniToSP.got_cmd_ref_pos = self.got_cmd_ref_pos
                    URPoseUniToSP.got_cmd_should_plan = self.got_cmd_should_plan
                else:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_sp = False
                    URPoseUniToSP.got_cmd_ref_pos = "_"
                    URPoseUniToSP.got_cmd_should_plan = False

            except rospy.ROSInterruptException:
                pass

            
            try:
                rospy.Subscriber("/unification_roscontrol/ur_tcp_pose_smaster_to_unidriver", String, self.ur_tcp_pose_smaster_to_unidriver_callback)
                rospy.Subscriber("/unification_roscontrol/ur_joint_pose_smaster_to_unidriver", URJointSmasterToUni, self.ur_joint_pose_smaster_to_unidriver_callback)
                rospy.Subscriber("/unification_roscontrol/moveit_smaster_to_unidriver", String, self.moveit_smaster_to_unidriver_callback)

                if time.time() < self.ur_tcp_pose_smaster_to_unidriver_timeout or\
                    time.time() < self.ur_joint_pose_smaster_to_unidriver_timeout or\
                    time.time() < self.moveit_smaster_to_unidriver_timeout:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = self.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster = self.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_moveit_smaster = self.ur_pose_unidriver_got_msg_from_moveit_smaster
                    URPoseUniToSP.planning = self.planning
                    URPoseUniToSP.executing = self.executing

                    if self.act_tcp_pos != "unknown" and self.act_joint_pos != "unknown":
                        self.act_pos = self.act_joint_pos

                    elif self.act_tcp_pos != "unknown" and self.act_joint_pos == "unknown":
                        self.act_pos = self.act_tcp_pos

                    elif self.act_tcp_pos == "unknown" and self.act_joint_pos != "unknown":
                        self.act_pos = self.act_joint_pos

                    elif self.act_tcp_pos == "unknown" and self.act_joint_pos == "unknown":
                        self.act_pos = "unknown"

                    else:
                        pass
                    
                    URPoseUniToSP.act_pos = self.act_pos

                else:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = False
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster = False
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_moveit_smaster = False
                    URPoseUniToSP.planning = False
                    URPoseUniToSP.executing = False
                    URPoseUniToSP.act_pos = "_"

                
            except rospy.ROSInterruptException:
                pass

            '''

            try:
                rospy.Subscriber("/unification_roscontrol/ur_tcp_pose_smaster_to_unidriver", String, self.ur_tcp_pose_smaster_to_unidriver_callback)

                if time.time() < self.ur_tcp_pose_smaster_to_unidriver_timeout:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = self.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster

                else:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = False
                
            except rospy.ROSInterruptException:
                pass


            try:
                rospy.Subscriber("/unification_roscontrol/ur_joint_pose_smaster_to_unidriver", URJointSmasterToUni, self.ur_joint_pose_smaster_to_unidriver_callback)

                if time.time() < self.ur_joint_pose_smaster_to_unidriver_timeout:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster = self.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster
                    URPoseUniToSP.executing = self.executing

                else:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_ur_joint_pose_smaster = False
                    URPoseUniToSP.executing = False
                
            except rospy.ROSInterruptException:
                pass

            
            try:
                rospy.Subscriber("/unification_roscontrol/moveit_smaster_to_unidriver", String, self.moveit_smaster_to_unidriver_callback)

                if time.time() < self.moveit_smaster_to_unidriver_timeout:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_moveit_smaster = self.ur_pose_unidriver_got_msg_from_moveit_smaster
                    URPoseUniToSP.planning = self.planning

                else:
                    URPoseUniToSP.ur_pose_unidriver_got_msg_from_moveit_smaster = False
                    URPoseUniToSP.planning = False
                
            except rospy.ROSInterruptException:
                pass


            if self.act_tcp_pos != "unknown" and self.act_joint_pos != "unknown":
                self.act_pos = self.act_joint_pos

            elif self.act_tcp_pos != "unknown" and self.act_joint_pos == "unknown":
                self.act_pos = self.act_tcp_pos
            
            elif self.act_tcp_pos == "unknown" and self.act_joint_pos != "unknown":
                self.act_pos = self.act_joint_pos

            elif self.act_tcp_pos == "unknown" and self.act_joint_pos == "unknown":
                self.act_pos = "unknown"

            else:
                pass

            URPoseUniToSP.act_pos = self.act_pos
            '''

            self.ur_pose_unidriver_to_sp_publisher.publish(self.ur_pose_state)
            self.main_rate.sleep()

        rospy.spin()

    
    def ur_pose_sp_to_unidriver_callback(self, ur_mode_cmd):

        self.ur_pose_sp_to_unidriver_timeout = time.time() + 2
        self.ur_pose_unidriver_got_msg_from_sp = True

        if "TCP" in ur_mode_cmd.ref_pos and self.got_cmd_should_plan == False:
            self.ur_pose_unidriver_to_ur_tcp_pose_smaster_publisher.publish(ur_mode_cmd.ref_pos)
            self.got_cmd_ref_pos = ur_mode_cmd.ref_pos

        elif "JOINT" in ur_mode_cmd.ref_pos and self.got_cmd_should_plan == False:
            self.ur_pose_unidriver_to_ur_joint_pose_smaster_publisher.publish(ur_mode_cmd.ref_pos)
            self.got_cmd_ref_pos = ur_mode_cmd.ref_pos

        elif ("TCP" in ur_mode_cmd.ref_pos or "JOINT" in ur_mode_cmd.ref_pos) and self.got_cmd_should_plan == True:
            self.ur_pose_unidriver_to_moveit_smaster_publisher.publish(ur_mode_cmd.ref_pos)
            self.got_cmd_ref_pos = ur_mode_cmd.ref_pos
        
        else:
            pass


        
    def ur_tcp_pose_smaster_to_unidriver_callback(self, tcp_pose):

        self.ur_tcp_pose_smaster_to_unidriver_timeout = time.time() + 2
        self.up_pose_unidriver_got_msg_from_ur_tcp_pose_smaster = True
        self.act_tcp_pos = tcp_pose.data


    def ur_joint_pose_smaster_to_unidriver_callback(self, joint_pose):

        self.ur_joint_pose_smaster_to_unidriver_timeout = time.time() + 2
        self.up_pose_unidriver_got_msg_from_ur_joint_pose_smaster = True
        self.act_joint_pos = joint_pose.pose
        self.executing = joint_pose.executing


    def moveit_smaster_to_unidriver_callback(self, moveit):

        self.moveit_smaster_to_unidriver_timeout = time.time() + 2
        self.up_pose_unidriver_got_msg_from_moveit_smaster = True
        
        if moveit.planning == "planning":
            self.planning = True

        else:
            self.planning = False


if __name__ == '__main__':
    try:
        ur_pose_unidriver()
    except rospy.ROSInterruptException:
        pass