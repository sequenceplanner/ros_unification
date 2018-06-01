#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # AECU Unification Driver based on Setek AECU ROS Driver specification
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


class aecu_unidriver():

    def __init__(self):
        
        rospy.init_node('aecu_unidriver', anonymous=False)

        self.ATR_state = '_'    # Atlas Tool Run
        self.ATP_state = '_'    # Atlas Tool Position
        self.ATQ_state = '_'    # Atlas Tool Torque
        
        rospy.Subscriber("/AECU_status", UInt16, self.aecuCallback)
        rospy.Subscriber("/sp_to_aecu_unidriver", String, self.sp_to_aecu_unidriver_callback)

        self.aecu_atr_state_publisher = rospy.Publisher('aecu_atr_unistate', String, queue_size=10)
        self.aecu_atp_state_publisher = rospy.Publisher('aecu_atp_unistate', String, queue_size=10)
        self.aecu_atq_state_publisher = rospy.Publisher('aecu_atq_unistate', String, queue_size=10)
        self.aecu_control_publisher = rospy.Publisher('/CT_AECU_con', UInt16, queue_size=10)

        # Maybe not needed anymore? Uncomment if needed
        # self.aecu_ack_publisher = rospy.Publisher('aecu_ack, String, queue_size=10)

        # testing
        #self.testpub = rospy.Publisher('/AECU_status', UInt16, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            self.aecu_atr_state_publisher.publish(self.ATR_state)
            self.aecu_atp_state_publisher.publish(self.ATP_state)
            self.aecu_atq_state_publisher.publish(self.ATQ_state)

            # testing
            # self.testpub.publish(int('000' + '000' + '000' + '0000000', 2))

            self.main_rate.sleep()

        rospy.spin()

    
    #----------------------------------------------------------------------------------------
    # bridge to driver methods
    #----------------------------------------------------------------------------------------
    def aecu_set_tool_idle(self):
        self.aecu_control_publisher.publish(int('00' + '001' + '00000000000', 2))

    def aecu_run_tool_forward(self):
        self.aecu_control_publisher.publish(int('00' + '010' + '00000000000', 2))

    def aecu_run_tool_in_reverse(self):
        self.aecu_control_publisher.publish(int('00' + '011' + '00000000000', 2))


    #----------------------------------------------------------------------------------------
    # Main callback for sp communication
    #----------------------------------------------------------------------------------------
    def sp_to_aecu_unidriver_callback(self, sp_data):
        self.aecu_data = sp_data.data
        if self.aecu_data == "set_tool_idle":
            # Uncomment if needed
            # self.aecu_ack_publisher.publish("aecu_unidriver got msg: set_tool_idle")
            self.aecu_set_tool_idle()
        elif self.aecu_data == "run_tool_forward":
            # Uncomment if needed
            # self.aecu_ack_publisher.publish("aecu_unidriver got msg: run_tool_forward")
            self.aecu_run_tool_forward()
        elif self.aecu_data == "run_tool_in_reverse":
            # Uncomment if needed
            # self.aecu_ack_publisher.publish("aecu_unidriver got msg: run_tool_in_reverse")
            self.aecu_run_tool_in_reverse()
        else:
            pass


    #----------------------------------------------------------------------------------------------------------------
    # aecuCallback
    #----------------------------------------------------------------------------------------------------------------
    def aecuCallback(self, aecu):
        self.aecu_bin = format(aecu.data, '016b')

        self.ATR_int_state = int(self.aecu_bin[0:3], 2)
        self.ATP_int_state = int(self.aecu_bin[3:6], 2)
        self.ATQ_int_state = int(self.aecu_bin[6:9], 2)
        self.AECU_int_other = int(self.aecu_bin[9:16], 2)

        if self.ATR_int_state == 1:
            self.ATR_state = "tool_is_idle"
        elif self.ATR_int_state == 2:
            self.ATR_state = "tool_is_run_manually_forward"
        elif self.ATR_int_state == 3:
            self.ATR_state = "tool_is_run_manually_reverse"
        elif self.ATR_int_state == 4:
            self.ATR_state = "tool_is_run_from_ros_forward"
        elif self.ATR_int_state == 5:
            self.ATR_state = "tool_is_run_from_ros_reverse"
        elif self.ATR_int_state == 6:
            self.ATR_state = "unknown_run_status"
        else:
            self.ATR_state = "AECU_ATR_NDEF"

        if self.ATP_int_state == 1:
            self.ATP_state = "positioned_at_home_station"
        elif self.ATP_int_state == 2:
            self.ATP_state = "taken_by_operator"
        elif self.ATP_int_state== 3:
            self.ATP_state = "taken_by_robot"
        elif self.ATP_int_state== 4:
            self.ATP_state = "unknown_position"
        else:
            self.ATP_state = "AECU_ATP_NDEF"

        if self.ATQ_int_state == 1:
            self.ATQ_state = "torque_not_reached"
        elif self.ATQ_int_state == 2:
            self.ATQ_state = "programmed_torque_reached"
        elif self.ATQ_int_state== 3:
            self.ATQ_state = "unknown_torque"
        else:
            self.ATQ_state = "AECU_ATQ_NDEF"

        # testing
        # print self.ATR_state + ' and ' + self.ATP_state + ' and ' + self.ATQ_state
        

if __name__ == '__main__':
    try:
        aecu_unidriver()
    except rospy.ROSInterruptException:
        pass