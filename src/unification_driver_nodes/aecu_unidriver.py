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
import GPIOEmu as GPIO
#import RPi.GPIO as GPIO
import time


class aecu_unidriver():

    def __init__(self):
        
        rospy.init_node('aecu_unidriver', anonymous=False)

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

        # publishers
        self.aecu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/aecu_unidriver_to_sp', AecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()



    def main(self):

        self.aecu_state = AecuUniToSP()

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
            AecuUniToSP.tool_is_runiing_reverse = self.tool_is_running_reverse
            AecuUniToSP.positioned_at_home_station = self.positioned_at_home_station
            AecuUniToSP.operating_position = self.operating_position
            AecuUniToSP.pre_home_position = self.pre_home_position
            AecuUniToSP.unclear_position = self.unclear_position
            AecuUniToSP.programmed_torque_reached = self.programmed_torque_reached

            
            # Read manual commands
            if GPIO.input(self.GPI1) == 1 and\
                GPIO.input(self.GPI2) == 0:
                self.tool_running_forward = True
                self.tool_running_reverse = False
                self.tool_is_idle = False

            elif GPIO.input(self.GPI1) == 0 and\
                GPIO.input(self.GPI2) == 1:
                self.tool_running_forward = False
                self.tool_running_reverse = True
                self.tool_is_idle = False

            else:
                self.tool_running_forward = False
                self.tool_running_reverse = False
                self.tool_is_idle = True

            
            # Read atlas tool position
            if GPIO.input(self.GPI3) == 1 or\
                (GPIO.input(self.GPI5) == 1 and\
                GPIO.input(self.GPI4) == 0):
                self.positioned_at_home_station = False
                self.operating_position = False
                self.pre_home_position = False
                self.unclear_position = True
            
            elif GPIO.input(self.GPI5) == 1 and\
                GPIO.input(self.GPI4) == 1:
                self.positioned_at_home_station = True
                self.operating_position = False
                self.pre_home_position = False
                self.unclear_position = False

            elif GPIO.input(self.GPI5) == 0 and\
                GPIO.input(self.GPI4) == 1:
                self.positioned_at_home_station = False
                self.operating_position = False
                self.pre_home_position = True
                self.unclear_position = False

            else:
                self.positioned_at_home_station = False
                self.operating_position = True
                self.pre_home_position = False
                self.unclear_position = False
            

            # Read torque
            if GPIO.input(self.GPI6) == True:
                self.programmed_torque_reached = True
            
            else:
                self.programmed_torque_reached = False

            # Not sure what this is
            if GPIO.input(self.GPI7) == True:
                self.tool_is_idle = True
            
            else:
                self.tool_is_idle = True


            self.aecu_to_sp_publisher.publish(self.aecu_state)
            self.main_rate.sleep()

        rospy.spin()



    def aecu_set_tool_idle(self):
        GPIO.output(self.GPO1, 0)
        GPIO.output(self.GPO2, 0)

    def aecu_run_tool_forward(self):
        GPIO.output(self.GPO1, 1)                 
        GPIO.output(self.GPO2, 0) 

    def aecu_run_tool_in_reverse(self):
        GPIO.output(self.GPO1, 1)
        GPIO.output(self.GPO2, 1)

    def aecu_inhibit_all_run_also_manual(self):
        GPIO.output(self.GPO1, 0)
        GPIO.output(self.GPO2, 0)

    def aecu_activate_unload(self):
        # What outputs?
        pass

    def aecu_disactivate_unload(self):
        # What outputs?
        pass

    def aecu_activate_lift(self):
        # What outputs?
        pass

    def aecu_disactivate_lift(self):
        # What outputs?
        pass


    def sp_to_aecu_unidriver_callback(self, aecu_cmd):

        self.sp_to_aecu_unidriver_timeout = time.time() + 2
        self.aecu_unidriver_got_msg_from_sp = True
        self.set_tool_idle = aecu_cmd.set_tool_idle
        self.run_tool_forward = aecu_cmd.run_tool_forward
        self.run_tool_in_reverse = aecu_cmd.run_tool_in_reverse
        self.inhibit_all_run_also_manual = aecu_cmd.inhibit_all_run_also_manual
        self.activate_unload = aecu_cmd.activate_unload
        self.activate_lift = aecu_cmd.activate_lift

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
            self.aecu_set_tool_idle()

        
        if self.activate_lift == True:
            self.aecu_activate_lift()
        else:
            self.disable_lift()

        
        if self.activate_unload == True:
            self.aecu_activate_unload()
        else:
            self.disable_unload()


if __name__ == '__main__':
    try:
        aecu_unidriver()
    except rospy.ROSInterruptException:
        pass