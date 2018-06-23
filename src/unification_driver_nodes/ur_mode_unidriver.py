#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Mode Unification Driver
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from unification_roscontrol.msg import URModeSPToUni
from unification_roscontrol.msg import URModeUniToSP
from std_msgs.msg import String
import socket
import time


class ur_mode_unidriver():

    def __init__(self):
        
        rospy.init_node('ur_mode_unidriver', anonymous=False)

        self.ur_mode_sp_to_unidriver_timeout = 100
        self.ur_mode_smaster_to_unidriver_timeout = 100

        # state
        self.ur_mode_unidriver_got_msg_from_ur_mode_smaster = False
        self.normal = False
        self.reduced = False
        self.protective_stop = False
        self.recovery = False
        self.safeguard_stop = False
        self.system_emergency_stop = False
        self.robot_emergency_stop = False
        self.violation = False
        self.fault = False

        # command
        self.ur_mode_unidriver_got_msg_from_sp = False
        self.ur_activate_safeguard = False
        self.ur_disengage_safeguard = False
        self.ur_disengage_protective = False

        # subscribers
        rospy.Subscriber("/unification_roscontrol/ur_mode_sp_to_unidriver", URModeSPToUni, self.ur_mode_sp_to_unidriver_callback)
        rospy.Subscriber("/unification_roscontrol/ur_mode_smaster_to_unidriver", String, self.ur_mode_smaster_to_unidriver_callback)

        # publishers
        self.ur_mode_unidriver_to_smaster_publisher = rospy.Publisher('unification_roscontrol/ur_mode_unidriver_to_smaster', String, queue_size=10)
        self.ur_mode_unidriver_to_sp_publisher = rospy.Publisher('unification_roscontrol/ur_mode_unidriver_to_sp', URModeUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.ur_mode_state = URModeUniToSP()

        while not rospy.is_shutdown():
            
            if time.time() < self.ur_mode_sp_to_unidriver_timeout:
                URModeUniToSP.ur_mode_unidriver_got_msg_from_sp = self.ur_mode_unidriver_got_msg_from_sp
                URModeUniToSP.got_cmd_ur_activate_safeguard = self.ur_activate_safeguard
                URModeUniToSP.got_cmd_ur_disengage_safeguard = self.ur_disengage_safeguard
                URModeUniToSP.got_cmd_ur_disengage_protective = self.ur_disengage_protective
            else:
                URModeUniToSP.ur_mode_unidriver_got_msg_from_sp = False
                URModeUniToSP.got_cmd_ur_activate_safeguard = False
                URModeUniToSP.got_cmd_ur_disengage_safeguard = False
                URModeUniToSP.got_cmd_ur_disengage_protective = False
                self.ur_activate_safeguard = False
                self.ur_disengage_safeguard = False
                self.ur_disengage_protective = False
            


            if time.time() < self.ur_mode_smaster_to_unidriver_timeout:
                URModeUniToSP.ur_mode_unidriver_got_msg_from_ur_mode_smaster = self.ur_mode_unidriver_got_msg_from_ur_mode_smaster
                URModeUniToSP.normal = self.normal
                URModeUniToSP.reduced = self.reduced
                URModeUniToSP.protective_stop = self.protective_stop
                URModeUniToSP.recovery = self.recovery
                URModeUniToSP.safeguard_stop = self.safeguard_stop
                URModeUniToSP.system_emergency_stop = self.system_emergency_stop
                URModeUniToSP.robot_emergency_stop = self.robot_emergency_stop
                URModeUniToSP.violation = self.violation
                URModeUniToSP.fault = self.fault

            else:
                URModeUniToSP.ur_mode_unidriver_got_msg_from_ur_mode_smaster = False
                URModeUniToSP.normal = False
                URModeUniToSP.reduced = False
                URModeUniToSP.protective_stop = False
                URModeUniToSP.recovery = False
                URModeUniToSP.safeguard_stop = False
                URModeUniToSP.system_emergency_stop = False
                URModeUniToSP.robot_emergency_stop = False
                URModeUniToSP.violation = False
                URModeUniToSP.fault = False
                

            self.ur_mode_unidriver_to_sp_publisher.publish(self.ur_mode_state)
            self.main_rate.sleep()

        rospy.spin()

    
    def activate_safeguard(self):
        self.ur_mode_unidriver_to_smaster_publisher.publish("activate_safeguard")

    def disengage_safeguard(self):
        self.ur_mode_unidriver_to_smaster_publisher.publish("disengage_safeguard")

    def disengage_protective(self):
        HOST = "192.168.1.14"
        PORT = 29999 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send ("unlock protective stop" + "\n")
        s.close()  


    def ur_mode_sp_to_unidriver_callback(self, ur_mode_cmd):

        self.ur_mode_sp_to_unidriver_timeout = time.time() + 2
        self.ur_mode_unidriver_got_msg_from_sp = True
        self.ur_activate_safeguard = ur_mode_cmd.ur_activate_safeguard
        self.ur_disengage_safeguard = ur_mode_cmd.ur_disengage_safeguard
        self.ur_disengage_protective = ur_mode_cmd.ur_disengage_protective


        if self.ur_activate_safeguard == True and\
            self.ur_disengage_safeguard == False and\
            self.ur_disengage_protective == False:
            self.activate_safeguard()

        elif self.ur_activate_safeguard == False and\
            self.ur_disengage_safeguard == True and\
            self.ur_disengage_protective == False:
            self.disengage_safeguard()

        elif self.ur_activate_safeguard == False and\
            self.ur_disengage_safeguard == False and\
            self.ur_disengage_protective == True:
            self.disengage_protective()

        else:
            pass

        
    def ur_mode_smaster_to_unidriver_callback(self, ur_mode):
        self.ur_mode_smaster_to_unidriver_timeout = time.time() + 2
        self.ur_mode_unidriver_got_msg_from_ur_mode_smaster = True

        if "NORMAL" in ur_mode.data:
            self.normal = True
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False
            
        elif "REDUCED" in ur_mode.data:
            self.normal = False
            self.reduced = True
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False

        elif "PROTECTIVE" in ur_mode.data:
            self.normal = False
            self.reduced = False
            self.protective_stop = True
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False

        elif "RECOVERY" in ur_mode.data:
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = True
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False

        elif "SAFEGUARD" in ur_mode.data :
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = True
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False

        elif ur_mode.data == "system_emergency_stop":
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = True
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = False
        
        elif ur_mode.data == "robot_emergency_stop":
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = True
            self.violation = False
            self.fault = False

        elif ur_mode.data == "violation":
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = True
            self.fault = False

        elif ur_mode.data == "fault":
            self.normal = False
            self.reduced = False
            self.protective_stop = False
            self.recovery = False
            self.safeguard_stop = False
            self.system_emergency_stop = False
            self.robot_emergency_stop = False
            self.violation = False
            self.fault = True
        
        else:
            pass

if __name__ == '__main__':
    try:
        ur_mode_unidriver()
    except rospy.ROSInterruptException:
        pass