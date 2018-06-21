#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Peter Lagerkvist
    # RECU Unification Driver based on Setek RECU ROS Driver specification
    # Recu and Recu Unidriver nodes now merged into one
    # Java-ROS plugin for SP removes need for Kafka
    # To emulate the Raspberry pi, uncomment the import for the emulator and comment the real
    # Get the Raspi Emulator at: https://github.com/paly2/GPIOEmu
    # V.0.8.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from std_msgs.msg import UInt16
from unification_roscontrol.msg import RecuSPToUni
from unification_roscontrol.msg import RecuUniToSP
import GPIOEmu as GPIO
#import RPi.GPIO as GPIO
import time


class recu_unidriver():

    def __init__(self):
        
        rospy.init_node('recu_unidriver', anonymous=False)

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

        self.sp_to_recu_unidriver_timeout = 100

        # state
        self.robot_not_connected_to_tool = False
        self.robot_connected_to_lf_tool = False
        self.robot_connected_to_atlas_tool = False
        self.robot_connected_to_filter_tool = False
        self.undefined_connection_detected = False
        self.robot_tool_connection_failure = False
        self.ladder_frame_not_gripped = False
        self.ladder_frame_gripped = False
        self.ladder_frame_connection_failure = False
        self.pressure_ok = False

        # command
        self.recu_unidriver_got_msg_from_sp = False
        self.lock_rsp = False
        self.unlock_rsp = False
        self.open_gripper = False
        self.close_gripper = False
        
        # subscribers
        rospy.Subscriber("/unification_roscontrol/recu_sp_to_unidriver", RecuSPToUni, self.sp_to_recu_unidriver_callback)

        # publishers
        self.recu_to_sp_publisher = rospy.Publisher('/unification_roscontrol/recu_unidriver_to_sp', RecuUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    def main(self):

        self.recu_state = RecuUniToSP()

        while not rospy.is_shutdown():

            if time.time() < self.sp_to_recu_unidriver_timeout:
                RecuUniToSP.recu_unidriver_got_msg_from_sp = self.recu_unidriver_got_msg_from_sp
                RecuUniToSP.got_cmd_lock_rsp = self.lock_rsp
                RecuUniToSP.got_cmd_unlock_rsp = self.unlock_rsp
                RecuUniToSP.got_cmd_open_gripper = self.open_gripper
                RecuUniToSP.got_cmd_close_gripper = self.close_gripper

            else:
                RecuUniToSP.recu_unidriver_got_msg_from_sp = False
                RecuUniToSP.got_cmd_lock_rsp = False
                RecuUniToSP.got_cmd_unlock_rsp = False
                RecuUniToSP.got_cmd_open_gripper = False
                RecuUniToSP.got_cmd_close_gripper = False
                

            RecuUniToSP.robot_not_connected_to_tool = self.robot_not_connected_to_tool
            RecuUniToSP.robot_connected_to_lf_tool = self.robot_connected_to_lf_tool
            RecuUniToSP.robot_connected_to_atlas_tool = self.robot_connected_to_atlas_tool
            RecuUniToSP.robot_connected_to_filter_tool = self.robot_connected_to_filter_tool
            RecuUniToSP.undefined_connection_detected = self.undefined_connection_detected
            RecuUniToSP.robot_tool_connection_failure = self.robot_tool_connection_failure
            RecuUniToSP.ladder_frame_not_connected = self.ladder_frame_not_gripped
            RecuUniToSP.ladder_frame_connected = self.ladder_frame_gripped
            RecuUniToSP.ladder_frame_connection_failure = self.ladder_frame_connection_failure
            RecuUniToSP.pressure_ok = self.pressure_ok



            if GPIO.input(self.GPI3) == 0 and\
                GPIO.input(self.GPI4) == 0 and\
                GPIO.input(self.GPI5) == 0:
                self.robot_not_connected_to_tool = True
                self.robot_connected_to_filter_tool = False
                self.robot_connected_to_atlas_tool = False
                self.robot_connected_to_lf_tool = False
                self.undefined_connection_detected = False
            
            elif GPIO.input(self.GPI3) == 1 and\
                GPIO.input(self.GPI4) == 0 and\
                GPIO.input(self.GPI5) == 0:
                self.robot_not_connected_to_tool = False
                self.robot_connected_to_filter_tool = True
                self.robot_connected_to_atlas_tool = False
                self.robot_connected_to_lf_tool = False
                self.undefined_connection_detected = False

            elif GPIO.input(self.GPI3) == 0 and\
                GPIO.input(self.GPI4) == 1 and\
                GPIO.input(self.GPI5) == 0:
                self.robot_not_connected_to_tool = False
                self.robot_connected_to_filter_tool = False
                self.robot_connected_to_atlas_tool = True
                self.robot_connected_to_lf_tool = False
                self.undefined_connection_detected = False

            elif GPIO.input(self.GPI3) == 0 and\
                GPIO.input(self.GPI4) == 0 and\
                GPIO.input(self.GPI5) == 1:
                self.robot_not_connected_to_tool = False
                self.robot_connected_to_filter_tool = False
                self.robot_connected_to_atlas_tool = False
                self.robot_connected_to_lf_tool = True
                self.undefined_connection_detected = False

            else:
                self.robot_not_connected_to_tool = False
                self.robot_connected_to_filter_tool = False
                self.robot_connected_to_atlas_tool = False
                self.robot_connected_to_lf_tool = False
                self.undefined_connection_detected = True



            if GPIO.input(self.GPI1) == 0 and\
                GPIO.input(self.GPI2) == 0:
                self.ladder_frame_not_gripped = True
                self.ladder_frame_gripped = False
                self.ladder_frame_connection_failure = False

            elif GPIO.input(self.GPI1) == 1 and\
                GPIO.input(self.GPI2) == 1:
                self.ladder_frame_not_gripped = False
                self.ladder_frame_gripped = True
                self.ladder_frame_connection_failure = False

            else:
                self.ladder_frame_not_gripped = False
                self.ladder_frame_gripped = False
                self.ladder_frame_connection_failure = True
            

            if GPIO.input(self.GPI7) == 1:
                self.pressure_ok = True
            
            else:
                self.pressure_ok = False


            self.recu_to_sp_publisher.publish(self.recu_state)
            self.main_rate.sleep()

        rospy.spin()


    def recu_lock_rsp(self):
        GPIO.output(self.GPO1, 0)

    def recu_unlock_rsp(self):
        GPIO.output(self.GPO1, 1)

    def recu_close_gripper(self):
        GPIO.output(self.GPO2, 0)

    def recu_open_gripper(self):
        GPIO.output(self.GPO2, 1)

    
    def sp_to_recu_unidriver_callback(self, recu_cmd):

        self.sp_to_recu_unidriver_timeout = time.time() + 2
        self.recu_unidriver_got_msg_from_sp = True
        self.lock_rsp = recu_cmd.lock_rsp
        self.unlock_rsp = recu_cmd.unlock_rsp
        self.open_gripper = recu_cmd.open_gripper
        self.close_gripper = recu_cmd.close_gripper

        if self.lock_rsp == True and\
            self.unlock_rsp == False and\
            self.open_gripper == False and\
            self.close_gripper == False:
            self.recu_lock_rsp()

        elif self.lock_rsp == False and\
            self.unlock_rsp == True and\
            self.open_gripper == False and\
            self.close_gripper == False:
            self.recu_unlock_rsp()

        elif self.lock_rsp == False and\
            self.unlock_rsp == False and\
            self.open_gripper == True and\
            self.close_gripper == False:
            self.recu_open_gripper()

        elif self.lock_rsp == False and\
            self.unlock_rsp == False and\
            self.open_gripper == False and\
            self.close_gripper == True:
            self.recu_close_gripper()

        else:
            pass


if __name__ == '__main__':
    try:
        recu_unidriver()
    except rospy.ROSInterruptException:
        pass
