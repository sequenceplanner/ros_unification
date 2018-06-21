#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # HECU Unification Driver based on Setek HECU ROS Driver specification
    # V.0.7.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import HecuUniToSP
import time


class hecu_unidriver():

    def __init__(self):
        
        rospy.init_node('hecu_unidriver', anonymous=False)

        self.hecu_to_hecu_unidriver_timeout = 100

        # state
        self.hecu_unidriver_got_msg_from_hecu = False
        self.hecu_alive = False
        self.lf_tool_home = False
        self.filter_tool_home = False
        
        # publishers
        self.hecu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/hecu_unidriver_to_sp', HecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.hecu_state = HecuUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/HECU_status", UInt16, self.hecuCallback)
            
                if time.time() < self.hecu_to_hecu_unidriver_timeout:
                    HecuUniToSP.hecu_unidriver_got_msg_from_hecu = self.hecu_unidriver_got_msg_from_hecu
                    HecuUniToSP.hecu_alive = self.hecu_alive
                    HecuUniToSP.lf_tool_home = self.lf_tool_home
                    HecuUniToSP.filter_tool_home = self.filter_tool_home
                else:
                    HecuUniToSP.hecu_unidriver_got_msg_from_hecu = False
                    HecuUniToSP.hecu_alive = False
                    HecuUniToSP.lf_tool_home = False
                    HecuUniToSP.filter_tool_home = False

            except rospy.ROSInterruptException:
                pass

            self.hecu_to_sp_publisher.publish(self.hecu_state)
            self.main_rate.sleep()

        rospy.spin()


    #----------------------------------------------------------------------------------------------------------------
    # hecuCallback
    #----------------------------------------------------------------------------------------------------------------
    def hecuCallback(self, hecu):
        self.hecu_bin = format(hecu.data, '016b')
        self.hecu_to_hecu_unidriver_timeout = time.time() + 2
        self.hecu_unidriver_got_msg_from_hecu = True

        self.HCA_int_state = int(self.hecu_bin[0:1], 2)
        self.LFT_int_state = int(self.hecu_bin[1:2], 2)
        self.OFT_int_state = int(self.hecu_bin[2:3], 2)
        self.HECU_int_other = int(self.hecu_bin[3:16], 2)

        if self.HCA_int_state == 1:
            self.hecu_alive = True
        else:
            self.hecu_alive = False
        
        if self.LFT_int_state == 1:
            self.lf_tool_home = True
        else:
            self.lf_tool_home = False            

        if self.OFT_int_state == 1:
            self.filter_tool_home = True
        else:
            self.filter_tool_home = False

if __name__ == '__main__':
    try:
        hecu_unidriver()
    except rospy.ROSInterruptException:
        pass