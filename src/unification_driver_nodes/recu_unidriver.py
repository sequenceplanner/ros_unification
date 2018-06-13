#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # RECU Unification Driver based on Setek RECU ROS Driver specification
    # Java-ROS plugin for SP removes need for Kafka
    # V.0.7.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import RecuSPToUni
from unification_roscontrol.msg import RecuUniToSP
import time


class recu_unidriver():

    def __init__(self):
        
        rospy.init_node('recu_unidriver', anonymous=False)

        self.RTC_int_state = 0
        self.LFG_int_state = 0
        self.APS_int_state = 0
        self.RECU_int_other = 0

        self.sp_to_recu_unidriver_timeout = 100
        self.recu_to_recu_unidriver_timeout = 100

        # state
        self.recu_unidriver_got_msg_from_recu = False
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
        self.pressure_failure = False

        # command
        self.recu_unidriver_got_msg_from_sp = False
        self.connect_to_tool = False
        self.disconect_from_tool = False
        self.release_lf = False
        self.grab_lf = False
        
        # publishers
        self.recu_control_publisher = rospy.Publisher('/CT_RECU_con', UInt16, queue_size=10)
        self.recu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/recu_unidriver_to_sp', RecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.recu_state = RecuUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/unification_roscontrol/recu_sp_to_unidriver", RecuSPToUni, self.sp_to_recu_unidriver_callback)

                if time.time() < self.sp_to_recu_unidriver_timeout:
                    RecuUniToSP.recu_unidriver_got_msg_from_sp = self.recu_unidriver_got_msg_from_sp
                    RecuUniToSP.connect_to_tool =  self.connect_to_tool
                    RecuUniToSP.disconnect_from_tool = self.disconect_from_tool
                    RecuUniToSP.release_lf = self.release_lf
                    RecuUniToSP.grab_lf = self.grab_lf
                else:
                    RecuUniToSP.recu_unidriver_got_msg_from_sp = False
                    RecuUniToSP.connect_to_tool = False
                    RecuUniToSP.disconect_from_tool = False
                    RecuUniToSP.release_lf = False
                    RecuUniToSP.grab_lf = False
                
            except rospy.ROSInterruptException:
                pass  


            try:
                rospy.Subscriber("/RECU_status", UInt16, self.recuCallback)

                if time.time() < self.recu_to_recu_unidriver_timeout:
                    RecuUniToSP.recu_unidriver_got_msg_from_recu = self.recu_unidriver_got_msg_from_recu
                    RecuUniToSP.robot_not_connected_to_tool = self.robot_not_connected_to_tool
                    RecuUniToSP.robot_connected_to_lf_tool = self.robot_connected_to_lf_tool
                    RecuUniToSP.robot_connected_to_atlas_tool = self.robot_connected_to_atlas_tool
                    RecuUniToSP.robot_connected_to_filter_tool = self.robot_connected_to_filter_tool
                    RecuUniToSP.undefined_connection_detected = self.undefined_connection_detected
                    RecuUniToSP.robot_tool_connection_failure = self.robot_tool_connection_failure
                    RecuUniToSP.ladder_frame_not_gripped = self.ladder_frame_not_gripped
                    RecuUniToSP.ladder_frame_gripped = self.ladder_frame_gripped
                    RecuUniToSP.ladder_frame_connection_failure = self.ladder_frame_connection_failure
                    RecuUniToSP.pressure_ok = self.pressure_ok
                    RecuUniToSP.pressure_failure = self.pressure_failure
                else:
                    RecuUniToSP.recu_unidriver_got_msg_from_recu = False
                    RecuUniToSP.robot_not_connected_to_tool = False
                    RecuUniToSP.robot_connected_to_lf_tool = False
                    RecuUniToSP.robot_connected_to_atlas_tool = False
                    RecuUniToSP.robot_connected_to_filter_tool = False
                    RecuUniToSP.undefined_connection_detected = False
                    RecuUniToSP.robot_tool_connection_failure = False
                    RecuUniToSP.ladder_frame_not_gripped = False
                    RecuUniToSP.ladder_frame_gripped = False
                    RecuUniToSP.ladder_frame_connection_failure = False
                    RecuUniToSP.pressure_ok = False
                    RecuUniToSP.pressure_failure = False
            
            except rospy.ROSInterruptException:
                pass

            self.recu_to_sp_publisher.publish(self.recu_state)
            self.main_rate.sleep()

        rospy.spin()

    
    #----------------------------------------------------------------------------------------
    # bridge to driver methods
    #----------------------------------------------------------------------------------------
    def recu_connect_tool(self):
        self.recu_control_publisher.publish(int('01' + '00' + '000000000000', 2))

    def recu_disconnect_tool(self):
        self.recu_control_publisher.publish(int('10' + '00' + '000000000000', 2))

    def recu_release_lf(self):
        self.recu_control_publisher.publish(int('00' + '01' + '000000000000', 2))

    def recu_grab_lf(self):
        self.recu_control_publisher.publish(int('00' + '10' + '000000000000', 2))


    #----------------------------------------------------------------------------------------
    # Main callback for SP communication
    #----------------------------------------------------------------------------------------
    def sp_to_recu_unidriver_callback(self, recu_cmd):

        self.sp_to_recu_unidriver_timeout = time.time() + 2

        self.recu_unidriver_got_msg_from_sp = True
        self.connect_to_tool = recu_cmd.connect_to_tool
        self.disconect_from_tool = recu_cmd.disconect_from_tool
        self.release_lf = recu_cmd.release_lf
        self.grab_lf = recu_cmd.grab_lf

        if self.connect_to_tool == True and\
            self.disconect_from_tool == False and\
            self.release_lf == False and\
            self.grab_lf == False:
            self.recu_connect_tool()

        elif self.connect_to_tool == False and\
            self.disconect_from_tool == True and\
            self.release_lf == False and\
            self.grab_lf == False:
            self.recu_disconnect_tool()

        elif self.connect_to_tool == False and\
            self.disconect_from_tool == False and\
            self.release_lf == False and\
            self.grab_lf == False:
            self.recu_release_lf()

        elif self.connect_to_tool == False and\
            self.disconect_from_tool == False and\
            self.release_lf == False and\
            self.grab_lf == True:
            self.recu_grab_lf()

        else:
            pass


    #----------------------------------------------------------------------------------------------------------------
    # recuCallback
    #----------------------------------------------------------------------------------------------------------------
    def recuCallback(self, recu):
        self.recu_bin = format(recu.data, '016b')
        self.recu_to_recu_unidriver_timeout = time.time() + 2
        self.recu_unidriver_got_msg_from_recu = True

        self.RTC_int_state = int(self.recu_bin[0:3], 2)
        self.LFG_int_state = int(self.recu_bin[3:6], 2)
        self.APS_int_state = int(self.recu_bin[6:8], 2)
        self.RECU_int_other = int(self.recu_bin[8:16], 2)

        if self.RTC_int_state == 1:
            #self.RTC_state = "robot_not_connected_to_tool"
            self.robot_not_connected_to_tool = True
            self.robot_connected_to_lf_tool = False
            self.robot_connected_to_atlas_tool = False
            self.robot_connected_to_filter_tool = False
            self.undefined_connection_detected = False
            self.robot_tool_connection_failure = False

        elif self.RTC_int_state == 2:
            #self.RTC_state = "robot_connected_to_lf_tool"
            self.robot_not_connected_to_tool = False
            self.robot_connected_to_lf_tool = True
            self.robot_connected_to_atlas_tool = False
            self.robot_connected_to_filter_tool = False
            self.undefined_connection_detected = False
            self.robot_tool_connection_failure = False

        elif self.RTC_int_state == 3:
            #self.RTC_state = "robot_connected_to_atlas_tool"
            self.robot_not_connected_to_tool = False
            self.robot_connected_to_lf_tool = False
            self.robot_connected_to_atlas_tool = True
            self.robot_connected_to_filter_tool = False
            self.undefined_connection_detected = False
            self.robot_tool_connection_failure = False

        elif self.RTC_int_state == 4:
            #self.RTC_state = "robot_connected_to_filter_tool"
            self.robot_not_connected_to_tool = False
            self.robot_connected_to_lf_tool = False
            self.robot_connected_to_atlas_tool = False
            self.robot_connected_to_filter_tool = True
            self.undefined_connection_detected = False
            self.robot_tool_connection_failure = False

        elif self.RTC_int_state == 5:
            #self.RTC_state = "undefined_connection_detected"
            self.robot_not_connected_to_tool = False
            self.robot_connected_to_lf_tool = False
            self.robot_connected_to_atlas_tool = False
            self.robot_connected_to_filter_tool = False
            self.undefined_connection_detected = True
            self.robot_tool_connection_failure = False

        elif self.RTC_int_state == 6:
            #self.RTC_state = "robot_tool_connection_failure"
            self.robot_not_connected_to_tool = False
            self.robot_connected_to_lf_tool = False
            self.robot_connected_to_atlas_tool = False
            self.robot_connected_to_filter_tool = False
            self.undefined_connection_detected = False
            self.robot_tool_connection_failure = True

        else:
            pass


        if self.LFG_int_state == 1:
            #self.LFG_state = "lf_not_connected"
            self.ladder_frame_not_gripped = True
            self.ladder_frame_gripped = False
            self.ladder_frame_connection_failure = False

        elif self.LFG_int_state == 2:
            #self.LFG_state = "lf_connected"
            self.ladder_frame_not_gripped = False
            self.ladder_frame_gripped = True
            self.ladder_frame_connection_failure = False

        elif self.LFG_int_state== 3:
            #self.LFG_state = "lf_connection_failure"
            self.ladder_frame_not_gripped = False
            self.ladder_frame_gripped = False
            self.ladder_frame_connection_failure = True

        else:
            pass


        if self.APS_int_state == 1:
            #self.APS_state = "pressure_ok"
            self.pressure_ok = True
            self.pressure_failure = False

        elif self.APS_int_state == 2:
            #self.APS_state = "pressure_failure"
            self.pressure_ok = False
            self.pressure_failure = True

        else:
            pass


if __name__ == '__main__':
    try:
        recu_unidriver()
    except rospy.ROSInterruptException:
        pass
