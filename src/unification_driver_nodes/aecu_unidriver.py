#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # AECU Unification Driver based on Setek AECU ROS Driver specification
    # Java-ROS plugin for SP removes need for Kafka
    # V.0.7.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import AecuSPToUni
from unification_roscontrol.msg import AecuUniToSP
import time


class aecu_unidriver():

    def __init__(self):
        
        rospy.init_node('aecu_unidriver', anonymous=False)

        self.ATR_int_state = 0
        self.ATP_int_state = 0
        self.ATQ_int_state = 0
        self.AECU_int_other = 0

        self.sp_to_aecu_unidriver_timeout = 100
        self.aecu_to_aecu_unidriver_timeout = 100

        # state
        self.aecu_unidriver_got_msg_from_aecu = False
        self.tool_is_idle = False
        self.tool_is_run_manually_forward = False
        self.tool_is_run_manually_reverse = False
        self.tool_is_run_from_ros_forward = False
        self.tool_is_run_from_ros_reverse = False
        self.unknown_run_status = False
        self.positioned_at_home_station = False
        self.operating_position = False
        self.pre_home_position = False
        self.unclear_position = False
        self.torque_not_reached = False
        self.programmed_torque_reached = False
        self.unknown_torque = False

        # command
        self.aecu_unidriver_got_msg_from_sp = False
        self.set_tool_idle = False
        self.run_tool_forward = False
        self.run_tool_in_reverse = False
        self.inhibit_all_run_also_manual = False

        # publishers
        self.aecu_control_publisher = rospy.Publisher('/CT_AECU_con', UInt16, queue_size=10)
        self.aecu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/aecu_unidriver_to_sp', AecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.aecu_state = AecuUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/unification_roscontrol/sp_to_aecu_unidriver", AecuSPToUni, self.sp_to_aecu_unidriver_callback)

                if time.time() < self.sp_to_aecu_unidriver_timeout:
                    AecuUniToSP.aecu_unidriver_got_msg_from_sp = self.aecu_unidriver_got_msg_from_sp
                    AecuUniToSP.got_cmd_set_tool_idle = self.set_tool_idle
                    AecuUniToSP.got_cmd_run_tool_forward = self.run_tool_forward
                    AecuUniToSP.got_cmd_run_tool_in_reverse = self.run_tool_in_reverse
                    AecuUniToSP.got_cmd_inhibit_all_run_also_manual = self.inhibit_all_run_also_manual
                else:
                    AecuUniToSP.aecu_unidriver_got_msg_from_sp = False
                    AecuUniToSP.got_cmd_set_tool_idle = False
                    AecuUniToSP.got_cmd_run_tool_forward = False
                    AecuUniToSP.got_cmd_run_tool_in_reverse = False
                    AecuUniToSP.got_cmd_inhibit_all_run_also_manual = False

            except rospy.ROSInterruptException:
                pass


            try:
                rospy.Subscriber("/AECU_status", UInt16, self.aecuCallback)

                if time.time() < self.aecu_to_aecu_unidriver_timeout:
                    AecuUniToSP.aecu_unidriver_got_msg_from_aecu = self.aecu_unidriver_got_msg_from_aecu
                    AecuUniToSP.tool_is_idle = self.tool_is_idle
                    AecuUniToSP.tool_is_run_manually_forward = self.tool_is_run_manually_forward
                    AecuUniToSP.tool_is_run_manually_reverse = self.tool_is_run_manually_reverse
                    AecuUniToSP.tool_is_run_from_ros_forward = self.tool_is_run_from_ros_forward
                    AecuUniToSP.tool_is_run_from_ros_reverse = self.tool_is_run_from_ros_reverse
                    AecuUniToSP.unknown_run_status = self.unknown_run_status
                    AecuUniToSP.positioned_at_home_station = self.positioned_at_home_station
                    AecuUniToSP.operating_position = self.operating_position
                    AecuUniToSP.pre_home_position = self.pre_home_position
                    AecuUniToSP.unclear_position = self.unclear_position
                    AecuUniToSP.torque_not_reached = self.torque_not_reached
                    AecuUniToSP.programmed_torque_reached = self.programmed_torque_reached
                    AecuUniToSP.unknown_torque = self.unknown_torque

                else:
                    AecuUniToSP.aecu_unidriver_got_msg_from_aecu = False
                    AecuUniToSP.tool_is_idle = False
                    AecuUniToSP.tool_is_run_manually_forward = False
                    AecuUniToSP.tool_is_run_manually_reverse = False
                    AecuUniToSP.tool_is_run_from_ros_forward = False
                    AecuUniToSP.tool_is_run_from_ros_reverse = False
                    AecuUniToSP.unknown_run_status = False
                    AecuUniToSP.positioned_at_home_station = False
                    AecuUniToSP.operating_position = False
                    AecuUniToSP.pre_home_position = False
                    AecuUniToSP.unclear_position = False
                    AecuUniToSP.torque_not_reached = False
                    AecuUniToSP.programmed_torque_reached = False
                    AecuUniToSP.unknown_torque = False
                
            except rospy.ROSInterruptException:
                pass

            self.aecu_to_sp_publisher.publish(self.aecu_state)
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

    def aecu_inhibit_all_run_also_manual(self):
        self.aecu_control_publisher.publish(int('00' + '100' + '00000000000', 2))


    #----------------------------------------------------------------------------------------
    # Main callback for sp communication
    #----------------------------------------------------------------------------------------
    def sp_to_aecu_unidriver_callback(self, aecu_cmd):

        self.sp_to_aecu_unidriver_timeout = time.time() + 2

        self.aecu_unidriver_got_msg_from_sp = True
        self.set_tool_idle = aecu_cmd.set_tool_idle
        self.run_tool_forward = aecu_cmd.run_tool_forward
        self.run_tool_in_reverse = aecu_cmd.run_tool_in_reverse
        self.inhibit_all_run_also_manual = aecu_cmd.inhibit_all_run_also_manual

        if self.set_tool_idle == True and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_set_tool_idle()

        elif self.set_tool_idle == False and\
            self.run_tool_forward == True and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_run_tool_forward()

        elif self.set_tool_idle == False and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == True and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_run_tool_in_reverse()
        
        elif self.set_tool_idle == False and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == True:
            self.aecu_inhibit_all_run_also_manual()

        else:
            pass


    #----------------------------------------------------------------------------------------------------------------
    # aecuCallback
    #----------------------------------------------------------------------------------------------------------------
    def aecuCallback(self, aecu):
        self.aecu_bin = format(aecu.data, '016b')
        self.aecu_to_aecu_unidriver_timeout = time.time() + 2
        self.aecu_unidriver_got_msg_from_aecu = True

        self.ATR_int_state = int(self.aecu_bin[0:3], 2)
        self.ATP_int_state = int(self.aecu_bin[3:6], 2)
        self.ATQ_int_state = int(self.aecu_bin[6:9], 2)
        self.AECU_int_other = int(self.aecu_bin[9:16], 2)

        if self.ATR_int_state == 1:
            #self.ATR_state = "tool_is_idle"
            self.tool_is_idle = True
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = False
            
        elif self.ATR_int_state == 2:
            #self.ATR_state = "tool_is_run_manually_forward"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = True
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = False

        elif self.ATR_int_state == 3:
            #self.ATR_state = "tool_is_run_manually_reverse"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = True
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = False

        elif self.ATR_int_state == 4:
            #self.ATR_state = "tool_is_run_from_ros_forward"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = True
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = False

        elif self.ATR_int_state == 5:
            #self.ATR_state = "tool_is_run_from_ros_reverse"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = True
            self.unknown_run_status = False

        elif self.ATR_int_state == 6:
            #self.ATR_state = "unknown_run_status"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = True
        else:
            # all false should indicate comm error
            #self.ATR_state = "AECU_ATR_NDEF"
            self.tool_is_idle = False
            self.tool_is_run_manually_forward = False
            self.tool_is_run_manually_reverse = False
            self.tool_is_run_from_ros_forward = False
            self.tool_is_run_from_ros_reverse = False
            self.unknown_run_status = False


        if self.ATP_int_state == 1:
            #self.ATP_state = "positioned_at_home_station"
            self.positioned_at_home_station = True
            self.operating_position = False
            self.pre_home_position = False
            self. unclear_position = False

        elif self.ATP_int_state == 4:
            #self.ATP_state = "taken_by_operator"
            self.positioned_at_home_station = False
            self.operating_position = True
            self.pre_home_position = False
            self. unclear_position = False

        elif self.ATP_int_state== 5:
            #self.ATP_state = "taken_by_robot"
            self.positioned_at_home_station = False
            self.operating_position = False
            self.pre_home_position = True
            self.unclear_position = False

        elif self.ATP_int_state== 6:
            #self.ATP_state = "unknown_position"
            self.positioned_at_home_station = False
            self.operating_position = False
            self.pre_home_position = False
            self.unclear_position = True

        else:
            # all false should indicate comm error
            # self.ATP_state = "AECU_ATP_NDEF"
            self.positioned_at_home_station = False
            self.operating_position = False
            self.pre_home_position = False
            self.unclear_position = False


        if self.ATQ_int_state == 1:
            #self.ATQ_state = "torque_not_reached"
            self.torque_not_reached = True
            self.programmed_torque_reached = False
            self.unknown_torque = False

        elif self.ATQ_int_state == 2:
            #self.ATQ_state = "programmed_torque_reached"
            self.torque_not_reached = False
            self.programmed_torque_reached = True
            self.unknown_torque = False

        elif self.ATQ_int_state== 3:
            #self.ATQ_state = "unknown_torque"
            self.torque_not_reached = False
            self.programmed_torque_reached = False
            self.unknown_torque = True

        else:
            # all false should indicate comm error
            #self.ATQ_state = "AECU_ATQ_NDEF"
            self.torque_not_reached = False
            self.programmed_torque_reached = False
            self.unknown_torque = False

if __name__ == '__main__':
    try:
        aecu_unidriver()
    except rospy.ROSInterruptException:
        pass