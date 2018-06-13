#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # AGV Alvar Tag Unification Driver
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
from unification_roscontrol.msg import AGVAlvarUniToSP
from geometry_msgs.msg import Point
import time

class agv_alvar_unidriver():

    def __init__(self):
        
        rospy.init_node('agv_alvar_unidriver', anonymous=False)

        self.agv_alvar_smaster_to_unidriver_timeout = 100

        self.A = -0.05  # Test value, change later
        self.B = 0.05   # Test value, change later
        self.C = -0.05  # Test value, change later
        self.D = 0.05   # Test value, change later

        self.agv_alvar_unidriver_got_msg_from_agv_alvar_smaster = False
        self.in_position = False
        self.xpos = 100
        self.ypos = 100

        self.agv_alvar_state_publisher = rospy.Publisher("unification_roscontrol/agv_alvar_unidriver_to_sp", AGVAlvarUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.agv_alvar_state = AGVAlvarUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/agv_alvar_smaster_to_unidriver", Point, self.agvAlvarCallback)

                if time.time() < self.agv_alvar_smaster_to_unidriver_timeout:
                    AGVAlvarUniToSP.agv_alvar_unidriver_got_msg_from_agv_alvar_smaster = self.agv_alvar_unidriver_got_msg_from_agv_alvar_smaster
                    AGVAlvarUniToSP.in_position = self.in_position
                    AGVAlvarUniToSP.x = self.xpos
                    AGVAlvarUniToSP.y = self.ypos
                else:
                    AGVAlvarUniToSP.agv_alvar_unidriver_got_msg_from_agv_alvar_smaster = False
                    AGVAlvarUniToSP.in_position = False
                    AGVAlvarUniToSP.x = 100
                    AGVAlvarUniToSP.y = 100

            except rospy.ROSInterruptException:
                pass

            self.agv_alvar_state_publisher.publish(self.agv_alvar_state)
            self.main_rate.sleep()

        rospy.spin()


    #----------------------------------------------------------------------------------------------------------------
    # agvAlvarCallback
    #----------------------------------------------------------------------------------------------------------------
    def agvAlvarCallback(self, agv_pos):

        self.agv_alvar_smaster_to_unidriver_timeout= time.time() + 2
        self.agv_alvar_unidriver_got_msg_from_agv_alvar_smaster  = True

        self.xpos = agv_pos.x
        self.ypos = agv_pos.y
        self.zpos = agv_pos.z

        if self.xpos > self.A and self.xpos < self.B and\
           self.ypos > self.C and self.ypos < self.D:
            self.in_position = True
        else:
            self.in_position = False


if __name__ == '__main__':
    try:
        agv_alvar_unidriver()
    except rospy.ROSInterruptException:
        pass