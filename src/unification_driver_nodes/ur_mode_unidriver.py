#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Mode Unification Driver for the Universal Robots UR10 
    # V.0.3.0.
#----------------------------------------------------------------------------------------


import rospy
import roslib
import socket
import struct
import json
import threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Point
import tf
import time
import numpy
from math import pi

HOST = "0.0.0.0"
DASHBOARD_SERVER = 29999

class ur_mode_unidriver():

    def __init__(self):
        
        rospy.init_node('ur_mode_unidriver', anonymous=False)

        self.ur_mode_unistate = "_"

        rospy.Subscriber("/ur_safetymode", String, self.safetyCallback)
        rospy.Subscriber("/sp_to_ur_mode_unidriver", String, self.sp_to_ur_mode_unidriver_callback)

        self.ur_mode_publisher = rospy.Publisher('ur_mode_unistate', String, queue_size=200)
        
        # Maybe not needed anymore? Uncomment if needed
        # self.ur_mode_ack_publisher = rospy.Publisher('ur_mode_ack', String, queue_size=10)

        self.main_rate = rospy.Rate(10)
        self.sf_stop_on = False

        rospy.sleep(1)

        self.main()

    
    #--------------------------------------------------------------------------------------------------------------------
    # UR10 IO rosservice
    # --------------------------------------------------------------------------------------------------------------------
    def set_IO_states(self, fun, pin, state):
        rospy.wait_for_service('/ur_driver/set_io')
        try:
            set_io = rospy.ServiceProxy('/ur_driver/set_io', SetIO)
            resp = set_io(fun, pin, state)
            return resp
        except rospy.ServiceException, e:
            print "Service call Failed: %s"%e


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            self.ur_mode_publisher.publish(self.safety)
            self.main_rate.sleep()
    
        rospy.spin()


    #----------------------------------------------------------------------------------------
    # Kafka Callbacks
    #----------------------------------------------------------------------------------------
    def URActivateSafeguard(self):
        set_IO_states(1, 8, 0)
        time.sleep(1)
        set_IO_states(1, 8, 1)
        self.sf_stop_on = True
    
    def URDisengageSafeguard(self):
        if self.sf_stop_on == True:
            set_IO_states(1, 10, 1)
            time.sleep(1)
            set_IO_states(1, 10, 0)
            self.sf_stop_on = False
        else:
            pass

    def URDisengageProtective(self):
        HOST = HOST_IP
        PORT = DASHBOARD_SERVER
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send ("unlock protective stop" + "\n")
        s.close()  


    #----------------------------------------------------------------------------------------
    # Main callback for kafka communication, edit later
    #----------------------------------------------------------------------------------------
    def sp_to_ur_mode_unidriver_callback(self, data):
        self.ur_safetystate = data.data
        if self.ur_safetystate == "activate_safeguard":
            # Uncomment if needed
            # self.ur_mode_ack_publisher.publish("ur_unidriver got msg: activate_safeguard")
            self.URActivateSafeguard()
        elif self.ur_safetystate == "disengage_safeguard":
            # Uncomment if needed
            # self.ur_mode_ack_publisher.publish("ur_unidriver got msg: disengage_safeguard")
            self.URDisengageSafeguard()
        elif self.ur_safetystate == "disengage_protective":
            # Uncomment if needed
            # self.ur_mode_ack_publisher.publish("ur_unidriver got msg: disengage_protective")
            self.URDisengageProtective()
        else:
            pass


    #----------------------------------------------------------------------------------------
    # safetyCallback
    #----------------------------------------------------------------------------------------
    def safetyCallback(self, safety_data):
        self.safety = safety_data.data
        if "NORMAL" in self.safety:
            self.ur_mode_unistate = "normal"
        elif "REDUCED" in self.safety:
            self.ur_mode_unistate = "reduced"
        elif "PROTECTIVE_STOP" in self.safety:
            self.ur_mode_unistate = "protective_stop"
        elif "RECOVERY" in self.safety:
            self.ur_mode_unistate = "recovery"
        elif "SAFEGUARD_STOP" in self.safety:
            self.ur_mode_unistate = "safeguard_stop"
        elif "SYSTEM_EMERGENCY_STOP" in self.safety:
            self.ur_mode_unistate = "system_emergency_stop"
        elif "ROBOT_EMERGENCY_STOP" in self.safety:
            self.ur_mode_unistate = "robot_emergency_stop"
        elif "VIOLATION" in self.safety:
            self.ur_mode_unistate = "violation"
        elif "FAULT" in self.safety:
            self.ur_mode_unistate = "fault"
        else:
            pass


if __name__ == '__main__':
    try:
        ur_mode_unidriver()
    except rospy.ROSInterruptException:
        pass