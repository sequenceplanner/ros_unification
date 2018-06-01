#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # AGV Alvar Tag Unification Driver
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Point
import threading
import time

class agv_alvar_unidriver():

    def __init__(self):
        
        rospy.init_node('agv_alvar_unidriver', anonymous=False)

        self.A = -0.05  # Test value
        self.B = 0.05   # Test value
        self.C = -0.05  # Test value
        self.D = 0.05   # Test value

        self.agv_alvar_state = '_'    # AGV at position or not
        
        rospy.Subscriber("/agv_alvar_pose", Point, self.agvAlvarCallback)

        self.agv_alvar_state_publisher = rospy.Publisher("agv_alvar_unistate", String, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            self.agv_alvar_state_publisher.publish(self.agv_alvar_state)
            self.main_rate.sleep()

        rospy.spin()


    #----------------------------------------------------------------------------------------------------------------
    # agvAlvarCallback
    #----------------------------------------------------------------------------------------------------------------
    def agvAlvarCallback(self, agv_pos):
        self.x = agv_pos.x
        self.y = agv_pos.y
        self.z = agv_pos.z

        if self.x > self.A and self.x < self.B and\
           self.y > self.C and self.y < self.D:
            self.agv_alvar_state = "agv_in_position"
        else:
            self.agv_alvar_state = "agv_not_in_position"


if __name__ == '__main__':
    try:
        agv_alvar_unidriver()
    except rospy.ROSInterruptException:
        pass