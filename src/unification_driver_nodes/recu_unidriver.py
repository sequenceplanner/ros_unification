#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Peter Lagerkvist
    # RECU Unification Driver based on Setek RECU ROS Driver specification
    # Recu and Recu Unidriver nodes now merged into one
    # Java-ROS plugin for SP removes need for Kafka
    # To emulate the Raspberry pi, uncomment the import for the emulator and comment the real
    # Get the Raspi Emulator at: https://github.com/paly2/GPIOEmu
    # Works as a msg reperater
    # V.0.9.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import RecuSPToUni
from unification_roscontrol.msg import RecuUniToSP
from unification_roscontrol.msg import RecuSPToUni2
from unification_roscontrol.msg import RecuUniToSP2
#import GPIOEmu as GPIO
#import RPi.GPIO as GPIO
import time


class recu_unidriver():

    def __init__(self):
        
        rospy.init_node('recu_unidriver', anonymous=False)

        self.sp_to_recu_unidriver_timeout = 100

        # state
        self.robot_not_connected_to_tool = False
        self.robot_connected_to_lf_tool = False
        self.robot_connected_to_atlas_tool = False
        self.robot_connected_to_filter_tool = False
        self.undefined_connection_detected = False
        self.robot_tool_connection_failure = False
        self.ladder_frame_not_gripped = False
        self.ladder_frame_gripped = False
        self.ladder_frame_connection_failure = False
        self.pressure_ok = False

        # command
        self.recu_unidriver_got_msg_from_sp = False
        self.lock_rsp = False
        self.unlock_rsp = False
        self.open_gripper = False
        self.close_gripper = False
        
        # subscribers
        rospy.Subscriber("/unification_roscontrol/recu_sp_to_unidriver", RecuSPToUni, self.sp_to_recu_unidriver_callback)
        rospy.Subscriber("/unification_roscontrol/recu_smaster_to_unidriver", RecuUniToSP2, self.recu_smaster_to_unidriver_callback)

        # publishers
        self.recu_unidriver_to_sp_publisher = rospy.Publisher('/unification_roscontrol/recu_unidriver_to_sp', RecuUniToSP, queue_size=10)
        self.recu_unidriver_to_smaster_publisher = rospy.Publisher('/unification_roscontrol/recu_unidriver_to_smaster', RecuSPToUni2, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    def main(self):

        self.recu_state = RecuUniToSP()
        self.recu_cmd = RecuSPToUni2()

        while not rospy.is_shutdown():

            if time.time() < self.sp_to_recu_unidriver_timeout:
                RecuUniToSP.recu_unidriver_got_msg_from_sp = self.recu_unidriver_got_msg_from_sp
                RecuUniToSP.got_cmd_lock_rsp = self.lock_rsp
                RecuUniToSP.got_cmd_unlock_rsp = self.unlock_rsp
                RecuUniToSP.got_cmd_open_gripper = self.open_gripper
                RecuUniToSP.got_cmd_close_gripper = self.close_gripper

            else:
                RecuUniToSP.recu_unidriver_got_msg_from_sp = False
                RecuUniToSP.got_cmd_lock_rsp = False
                RecuUniToSP.got_cmd_unlock_rsp = False
                RecuUniToSP.got_cmd_open_gripper = False
                RecuUniToSP.got_cmd_close_gripper = False
                

            RecuUniToSP.robot_not_connected_to_tool = self.robot_not_connected_to_tool
            RecuUniToSP.robot_connected_to_lf_tool = self.robot_connected_to_lf_tool
            RecuUniToSP.robot_connected_to_atlas_tool = self.robot_connected_to_atlas_tool
            RecuUniToSP.robot_connected_to_filter_tool = self.robot_connected_to_filter_tool
            RecuUniToSP.undefined_connection_detected = self.undefined_connection_detected
            RecuUniToSP.robot_tool_connection_failure = self.robot_tool_connection_failure
            RecuUniToSP.ladder_frame_not_connected = self.ladder_frame_not_gripped
            RecuUniToSP.ladder_frame_connected = self.ladder_frame_gripped
            RecuUniToSP.ladder_frame_connection_failure = self.ladder_frame_connection_failure
            RecuUniToSP.pressure_ok = self.pressure_ok

            RecuSPToUni2.lock_rsp = self.lock_rsp
            RecuSPToUni2.unlock_rsp  = self.unlock_rsp 
            RecuSPToUni2.open_gripper  = self.open_gripper 
            RecuSPToUni2.close_gripper = self.close_gripper

            self.recu_unidriver_to_smaster_publisher.publish(self.recu_cmd)
            self.recu_unidriver_to_sp_publisher.publish(self.recu_state)
            self.main_rate.sleep()

        rospy.spin()
    

    def sp_to_recu_unidriver_callback(self, recu_cmd):

        self.sp_to_recu_unidriver_timeout = time.time() + 2
        self.recu_unidriver_got_msg_from_sp = True
        self.lock_rsp = recu_cmd.lock_rsp
        self.unlock_rsp = recu_cmd.unlock_rsp
        self.open_gripper = recu_cmd.open_gripper
        self.close_gripper = recu_cmd.close_gripper

    
    def recu_smaster_to_unidriver_callback(self, recu_state):

        self.robot_not_connected_to_tool = recu_state.robot_not_connected_to_tool
        self.robot_connected_to_lf_tool = recu_state.robot_connected_to_lf_tool
        self.robot_connected_to_atlas_tool = recu_state.robot_connected_to_atlas_tool
        self.robot_connected_to_filter_tool = recu_state.robot_connected_to_filter_tool
        self.undefined_connection_detected = recu_state.undefined_connection_detected
        self.robot_tool_connection_failure = recu_state.robot_tool_connection_failure
        self.ladder_frame_not_gripped = recu_state.ladder_frame_not_connected
        self.ladder_frame_gripped = recu_state.ladder_frame_connected
        self.ladder_frame_connection_failure = recu_state.ladder_frame_connection_failure
        self.pressure_ok = recu_state.pressure_ok


if __name__ == '__main__':
    try:
        recu_unidriver()
    except rospy.ROSInterruptException:
        pass
