#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Peter Lagerkvist
    # AECU Unification Driver based on Setek AECU ROS Driver specification
    # Aecu and Aecu Unidriver now merged into one
    # Java-ROS plugin for SP removes need for Kafka
    # What actual input that tool is taken from the robot?
    # To emulate the Raspberry pi, uncomment the import for the emulator and comment the real
    # Get the Raspi Emulator at: https://github.com/paly2/GPIOEmu
    # V.0.8.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import AecuSPToUni
from unification_roscontrol.msg import AecuUniToSP
from unification_roscontrol.msg import AecuSPToUni2
from unification_roscontrol.msg import AecuUniToSP2
#import GPIOEmu as GPIO
#import RPi.GPIO as GPIO
import time


class aecu_unidriver():

    def __init__(self):
        
        rospy.init_node('aecu_unidriver', anonymous=False)

        self.sp_to_aecu_unidriver_timeout = 100
    
        # state
        self.tool_is_idle = False
        self.tool_is_running_forward = False
        self.tool_is_running_reverse = False
        self.positioned_at_home_station = False
        self.operating_position = False
        self.pre_home_position = False
        self.unclear_position = False
        self.programmed_torque_reached = False

        # command
        self.aecu_unidriver_got_msg_from_sp = False
        self.set_tool_idle = False
        self.run_tool_forward = False
        self.run_tool_in_reverse = False
        self.inhibit_all_run_also_manual = False
        self.activate_unload = False
        self.activate_lift = False

        # subscribers
        rospy.Subscriber("/unification_roscontrol/aecu_sp_to_unidriver", AecuSPToUni, self.sp_to_aecu_unidriver_callback)
        rospy.Subscriber("/unification_roscontrol/aecu_smaster_to_unidriver", AecuUniToSP2, self.aecu_smaster_to_unidriver_callback)

        # publishers
        self.aecu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/aecu_unidriver_to_sp', AecuUniToSP, queue_size=10)
        self.aecu_unidriver_to_smaster_publisher = rospy.Publisher('/unification_roscontrol/aecu_unidriver_to_smaster', AecuSPToUni2, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()



    def main(self):

        self.aecu_state = AecuUniToSP()
        self.aecu_cmd = AecuSPToUni2()

        while not rospy.is_shutdown():

            if time.time() < self.sp_to_aecu_unidriver_timeout:
                AecuUniToSP.aecu_unidriver_got_msg_from_sp = self.aecu_unidriver_got_msg_from_sp
                AecuUniToSP.got_cmd_set_tool_idle = self.set_tool_idle
                AecuUniToSP.got_cmd_run_tool_forward = self.run_tool_forward
                AecuUniToSP.got_cmd_run_tool_in_reverse = self.run_tool_in_reverse
                AecuUniToSP.got_cmd_inhibit_all_run_also_manual = self.inhibit_all_run_also_manual
                AecuUniToSP.got_cmd_activate_unload = self.activate_unload
                AecuUniToSP.got_cmd_activate_lift = self.activate_lift
            else:
                AecuUniToSP.aecu_unidriver_got_msg_from_sp = False
                AecuUniToSP.got_cmd_set_tool_idle = False
                AecuUniToSP.got_cmd_run_tool_forward = False
                AecuUniToSP.got_cmd_run_tool_in_reverse = False
                AecuUniToSP.got_cmd_inhibit_all_run_also_manual = False
                AecuUniToSP.got_cmd_activate_unload = False
                AecuUniToSP.got_cmd_activate_lift = False


            AecuUniToSP.tool_is_idle = self.tool_is_idle
            AecuUniToSP.tool_is_running_forward = self.tool_is_running_forward
            AecuUniToSP.tool_is_running_reverse = self.tool_is_running_reverse
            AecuUniToSP.positioned_at_home_station = self.positioned_at_home_station
            AecuUniToSP.operating_position = self.operating_position
            AecuUniToSP.pre_home_position = self.pre_home_position
            AecuUniToSP.unclear_position = self.unclear_position
            AecuUniToSP.programmed_torque_reached = self.programmed_torque_reached

            
            AecuSPToUni2.set_tool_idle = self.set_tool_idle
            AecuSPToUni2.run_tool_forward = self.run_tool_forward
            AecuSPToUni2.run_tool_in_reverse = self.run_tool_in_reverse
            AecuSPToUni2.inhibit_all_run_also_manual = self.inhibit_all_run_also_manual
            AecuSPToUni2.activate_unload = self.activate_unload
            AecuSPToUni2.activate_lift = self.activate_lift

            self.aecu_unidriver_to_smaster_publisher.publish(self.aecu_cmd)
            self.aecu_to_sp_publisher.publish(self.aecu_state)
            self.main_rate.sleep()

        rospy.spin()


    def sp_to_aecu_unidriver_callback(self, aecu_cmd):

        self.sp_to_aecu_unidriver_timeout = time.time() + 2
        self.aecu_unidriver_got_msg_from_sp = True
        self.set_tool_idle = aecu_cmd.set_tool_idle
        self.run_tool_forward = aecu_cmd.run_tool_forward
        self.run_tool_in_reverse = aecu_cmd.run_tool_in_reverse
        self.inhibit_all_run_also_manual = aecu_cmd.inhibit_all_run_also_manual
        self.activate_unload = aecu_cmd.activate_unload
        self.activate_lift = aecu_cmd.activate_lift

    def aecu_smaster_to_unidriver_callback(self, aecu_state):

        self.tool_is_idle = aecu_state.tool_is_idle
        self.tool_is_running_forward = aecu_state.tool_is_running_forward
        self.tool_is_running_reverse = aecu_state.tool_is_running_reverse
        self.positioned_at_home_station = aecu_state.positioned_at_home_station
        self.operating_position = aecu_state.operating_position
        self.pre_home_position = aecu_state.pre_home_position
        self.unclear_position = aecu_state.unclear_position
        self.programmed_torque_reached = aecu_state.programmed_torque_reached
    

if __name__ == '__main__':
    try:
        aecu_unidriver()
    except rospy.ROSInterruptException:
        pass