#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # RECU Unification Driver based on Setek RECU ROS Driver specification
    # Java-ROS plugin for SP removes need for Kafka
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
from std_msgs.msg import UInt16
import time


class recu_unidriver():

    def __init__(self):
        
        rospy.init_node('recu_unidriver', anonymous=False)

        self.RTC_state = '_'    # Robot-tool connection state
        self.LFG_state = '_'    # LF Grip state
        self.APS_state = '_'    # Air Pressure Supervision
        
        rospy.Subscriber("/RECU_status", UInt16, self.recuCallback)
        rospy.Subscriber("/sp_to_recu_unidriver", String, self.sp_to_recu_unidriver_callback)

        self.recu_rtc_state_publisher = rospy.Publisher('recu_rtc_unistate', String, queue_size=10)
        self.recu_lfg_state_publisher = rospy.Publisher('recu_lfg_unistate', String, queue_size=10)
        self.recu_control_publisher = rospy.Publisher('/CT_RECU_con', UInt16, queue_size=10)

        # Maybe not needed anymore? Uncomment if needed
        # self.recu_ack_publisher = rospy.Publisher('recu_ack', String, queue_size=10)

        # testing
        self.testpub = rospy.Publisher('/RECU_status', UInt16, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            self.recu_rtc_state_publisher.publish(self.RTC_state)
            self.recu_lfg_state_publisher.publish(self.LFG_state)

            # testing
            self.testpub.publish(int('000' + '000' + '0000000000', 2))

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
    def sp_to_recu_unidriver_callback(self, sp_data):
        self.recu_data = sp_data.data
        if self.recu_data == "connect_to_tool":
            # Uncomment if needed
            # self.recu_ack_publisher.publish("recu_unidriver got msg: connect_to_tool")
            self.recu_connect_tool()
        elif self.recu_data == "disconnect_from_tool":
            # Uncomment if needed
            # self.recu_ack_publisher.publish("recu_unidriver got msg: disconnect_from_tool")
            self.recu_disconnect_tool()
        elif self.recu_data == "release_lf":
            # Uncomment if needed
            # self.recu_ack_publisher.publish("recu_unidriver got msg: release_lf")
            self.recu_release_lf()
        elif self.recu_data == "grab_lf":
            # Uncomment if needed
            # self.recu_ack_publisher.publish("recu_unidriver got msg: grab_lf")
            self.recu_grab_lf()
        else:
            pass


    #----------------------------------------------------------------------------------------------------------------
    # recuCallback
    #----------------------------------------------------------------------------------------------------------------
    def recuCallback(self, recu):
        self.recu_bin = format(recu.data, '016b')

        self.RTC_int_state = int(self.recu_bin[0:3], 2)
        self.LFG_int_state = int(self.recu_bin[3:6], 2)
        self.APS_int_state = int(self.recu_bin[6:8], 2)
        self.RECU_int_other = int(self.recu_bin[8:16], 2)

        if self.RTC_int_state == 1:
            self.RTC_state = "robot_not_connected_to_tool"
        elif self.RTC_int_state == 2:
            self.RTC_state = "robot_connected_to_lf_tool"
        elif self.RTC_int_state == 3:
            self.RTC_state = "robot_connected_to_atlas_tool"
        elif self.RTC_int_state == 4:
            self.RTC_state = "robot_connected_to_filter_tool"
        elif self.RTC_int_state == 5:
            self.RTC_state = "undefined_connection_detected"
        elif self.RTC_int_state == 6:
            self.RTC_state = "robot_tool_connection_failure"
        else:
            self.RTC_state = "RECU_RTC_NDEF"

        if self.LFG_int_state == 1:
            self.LFG_state = "lf_not_connected"
        elif self.LFG_int_state == 2:
            self.LFG_state = "lf_connected"
        elif self.LFG_int_state== 3:
            self.LFG_state = "lf_connection_failure"
        else:
            self.LFG_state = "RECU_LFG_NDEF"

        if APS_int_state == 1:
            self.APS_state = "pressure_ok"
        elif APS_int_state == 2:
            self.APS_state = "pressure_failure"
        else:
            self.APS_state = "RECU_APS_NDEF"

        # testing
        print self.RTC_state + ' and ' + self.LFG_state
        

if __name__ == '__main__':
    try:
        recu_unidriver()
    except rospy.ROSInterruptException:
        pass
