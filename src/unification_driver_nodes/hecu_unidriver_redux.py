#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Peter Lagerkvist
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
import RPi.GPIO as GPIO
#import GPIOEmu as GPIO
import time


class hecu_unidriver():

    def __init__(self):
        
        rospy.init_node('hecu_unidriver', anonymous=False)

        self.GPO1 = 4
        self.GPO2 = 17
        self.GPO3 = 18
        self.GPO4 = 27
        self.GPI1 = 5
        self.GPI2 = 6
        self.GPI3 = 12
        self.GPI4 = 13
        self.GPI5 = 16
        self.GPI6 = 19
        self.GPI7 = 22
        self.GPI8 = 23

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)               
        GPIO.setup(self.GPO1, GPIO.OUT)
        GPIO.setup(self.GPO2, GPIO.OUT)
        GPIO.setup(self.GPO3, GPIO.OUT)
        GPIO.setup(self.GPO4, GPIO.OUT)
        GPIO.setup(self.GPI1, GPIO.IN)
        GPIO.setup(self.GPI2, GPIO.IN)
        GPIO.setup(self.GPI3, GPIO.IN)
        GPIO.setup(self.GPI4, GPIO.IN)
        GPIO.setup(self.GPI5, GPIO.IN)
        GPIO.setup(self.GPI6, GPIO.IN)
        GPIO.setup(self.GPI7, GPIO.IN)
        GPIO.setup(self.GPI8, GPIO.IN)

        GPIO.output(self.GPO1, False)
        GPIO.output(self.GPO2, False)
        GPIO.output(self.GPO3, False)
        GPIO.output(self.GPO4, False)

        self.hecu_to_hecu_unidriver_timeout = 100

        # state
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
        
            HecuUniToSP.lf_tool_home = self.lf_tool_home
            HecuUniToSP.filter_tool_home = self.filter_tool_home
        
            if GPIO.input(self.GPI1) == 1:
                self.lf_tool_home = True
            else:
                self.lf_tool_home = False   

            if GPIO.input(self.GPI2) == 1:
                self.filter_tool_home = True
            else:
                self.filter_tool_home = False   
                
            self.hecu_to_sp_publisher.publish(self.hecu_state)
            self.main_rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        hecu_unidriver()
    except rospy.ROSInterruptException:
        pass