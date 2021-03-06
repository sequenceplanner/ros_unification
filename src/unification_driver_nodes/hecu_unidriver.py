#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # HECU Unification Driver
    # V.0.8.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import HecuUniToSP
from unification_roscontrol.msg import HecuUniToSP2
import time


class hecu_unidriver():

    def __init__(self):
        
        rospy.init_node('hecu_unidriver', anonymous=False)

        self.hecu_smaster_to_hecu_unidriver_timeout = 100

        # state
        self.hecu_alive = False
        self.lf_tool_home = False
        self.filter_tool_home = False
        self.lf_tool_home_sm = False
        self.filter_tool_home_sm = False
        
        # subscribers
        rospy.Subscriber('unification_roscontrol/hecu_smaster_to_unidriver', HecuUniToSP2, self.hecuSmasterCallback)

        # publishers
        self.hecu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/hecu_unidriver_to_sp', HecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()



    def main(self):

        self.hecu_state = HecuUniToSP()

        while not rospy.is_shutdown():
        
            HecuUniToSP.lf_tool_home = self.lf_tool_home
            HecuUniToSP.filter_tool_home = self.filter_tool_home
        
            if self.lf_tool_home_sm == True:
                self.lf_tool_home = True
            else:
                self.lf_tool_home = False   

            if self.filter_tool_home_sm == True:
                self.filter_tool_home = True
            else:
                self.filter_tool_home = False   
                
            self.hecu_to_sp_publisher.publish(self.hecu_state)
            self.main_rate.sleep()

        rospy.spin()

    def hecuSmasterCallback(self, data):
        
        self.lf_tool_home_sm = data.lf_tool_home
        self.filter_tool_home_sm = data.filter_tool_home


if __name__ == '__main__':
    try:
        hecu_unidriver()
    except rospy.ROSInterruptException:
        pass