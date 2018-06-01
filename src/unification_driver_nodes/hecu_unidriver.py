#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # HECU Unification Driver based on Setek HECU ROS Driver specification
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


class hecu_unidriver():

    def __init__(self):
        
        rospy.init_node('hecu_unidriver', anonymous=False)

        self.HCA_state = '_'    # HECU alive or not
        self.LFT_state = '_'    # LF Tool home or not
        self.OFT_state = '_'    # Filter Tool home or not
        
        rospy.Subscriber("/HECU_status", UInt16, self.hecuCallback)

        self.hecu_hca_state_publisher = rospy.Publisher('hecu_hca_unistate', String, queue_size=10)
        self.hecu_lft_state_publisher = rospy.Publisher('hecu_lft_unistate', String, queue_size=10)
        self.hecu_oft_state_publisher = rospy.Publisher('hecu_oft_unistate', String, queue_size=10)

        # testing
        self.testpub = rospy.Publisher('/HECU_status', UInt16, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            self.hecu_hca_state_publisher.publish(self.HCA_state)
            self.hecu_lft_state_publisher.publish(self.LFT_state)
            self.hecu_oft_state_publisher.publish(self.OFT_state)

            # testing
            self.testpub.publish(int('0' + '0' + '0' + '0000000000000', 2))

            self.main_rate.sleep()

        rospy.spin()


    #----------------------------------------------------------------------------------------------------------------
    # hecuCallback
    #----------------------------------------------------------------------------------------------------------------
    def hecuCallback(self, hecu):
        self.hecu_bin = format(hecu.data, '016b')

        self.HCA_int_state = int(self.hecu_bin[0:1], 2)
        self.LFT_int_state = int(self.hecu_bin[1:2], 2)
        self.OFT_int_state = int(self.hecu_bin[2:3], 2)
        self.HECU_int_other = int(self.hecu_bin[3:16], 2)

        if self.HCA_int_state == 1:
            self.HCA_state = "hecu_is_alive"
        else:
            self.HCA_state = "hecu_is_not_alive"

        if self.LFT_int_state == 1:
            self.LFT_state = "lf_tool_home"
        else:
            self.LFT_state = "lf_tool_not_home"

        if self.OFT_int_state == 1:
            self.OFT_state = "filter_tool_home"
        else:
            self.OFT_state = "filter_tool_not_home"

        # testing
        print self.HCA_state + ' and ' + self.LFT_state + ' and ' + self.OFT_state
        

if __name__ == '__main__':
    try:
        hecu_unidriver()
    except rospy.ROSInterruptException:
        pass