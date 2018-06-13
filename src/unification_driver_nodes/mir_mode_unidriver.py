#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # MiR Mode Unification Driver
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import struct
from unification_roscontrol.msg import MiRModeSPToUni
from unification_roscontrol.msg import MiRModeUniToSP
from std_msgs.msg import String
import time


class mir_mode_unidriver():

    def __init__(self):
        
        rospy.init_node('mir_mode_unidriver', anonymous=False)

        self.sp_to_mir_mode_unidriver_timeout = 100
        self.mir_mode_smaster_to_unidriver_timeout = 100

        # state
        self.mir_mode_unidriver_got_msg_from_mir_mode_smaster = False
        self.none = False
        self.starting = False
        self.shutting_down = False
        self.ready = False
        self.pause = False
        self.executing = False
        self.aborted = False
        self.completed = False
        self.docked = False
        self.docking = False
        self.emergency_stop = False
        self.manual_control = False
        self.error = False

        # command
        self.mir_mode_unidriver_got_msg_from_sp = False
        self.set_state_to_ready = False
        self.set_state_to_paused = False
        self.set_state_to_executing = False
        self.set_state_to_aborted = False

        # publishers
        self.mir_mode_unidriver_to_smaster_publisher = rospy.Publisher('/unification_roscontrol/mir_mode_unidriver_to_smaster', String, queue_size=10)
        self.mir_mode_unidriver_to_sp_publisher = rospy.Publisher('/unification_roscontrol/mir_mode_unidriver_to_sp', MiRModeUniToSP, queue_size=10)
        
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):

        self.mir_mode_state = MiRModeUniToSP()

        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/unification_roscontrol/mir_mode_sp_to_unidriver", MiRModeSPToUni, self.sp_to_mir_mode_unidriver_callback)

                if time.time() < self.sp_to_mir_mode_unidriver_timeout:
                    MiRModeUniToSP.mir_mode_unidriver_got_msg_from_sp = self.mir_mode_unidriver_got_msg_from_sp
                    MiRModeUniToSP.got_cmd_set_state_to_ready = self.set_state_to_ready
                    MiRModeUniToSP.got_cmd_set_state_to_paused = self.set_state_to_paused
                    MiRModeUniToSP.got_cmd_set_state_to_executing = self.set_state_to_executing
                    MiRModeUniToSP.got_cmd_set_state_to_aborted = self.set_state_to_aborted
                else:
                    MiRModeUniToSP.mir_mode_unidriver_got_msg_from_sp = False
                    MiRModeUniToSP.got_cmd_set_state_to_ready = False
                    MiRModeUniToSP.got_cmd_set_state_to_paused = False
                    MiRModeUniToSP.got_cmd_set_state_to_executing = False
                    MiRModeUniToSP.got_cmd_set_state_to_aborted = False

            except rospy.ROSInterruptException:
                pass


            try:
                rospy.Subscriber("/unification_roscontrol/mir_mode_smaster_to_unidriver", String, self.mirModeCallback)

                if time.time() < self.mir_mode_smaster_to_unidriver_timeout:
                    MiRModeUniToSP.mir_mode_unidriver_got_msg_from_mir_mode_smaster = self.mir_mode_unidriver_got_msg_from_mir_mode_smaster
                    MiRModeUniToSP.none = self.none
                    MiRModeUniToSP.starting = self.starting
                    MiRModeUniToSP.shutting_down = self.shutting_down
                    MiRModeUniToSP.ready = self.ready
                    MiRModeUniToSP.pause = self.pause
                    MiRModeUniToSP.executing = self.executing
                    MiRModeUniToSP.aborted = self.aborted
                    MiRModeUniToSP.completed = self.completed
                    MiRModeUniToSP.docked = self.docked
                    MiRModeUniToSP.docking = self.docking
                    MiRModeUniToSP.emergency_stop = self.emergency_stop
                    MiRModeUniToSP.manual_control = self.manual_control
                    MiRModeUniToSP.error = self.error

                else:
                    MiRModeUniToSP.mir_mode_unidriver_got_msg_from_mir_mode_smaster = False
                    MiRModeUniToSP.starting = False
                    MiRModeUniToSP.shutting_down = False
                    MiRModeUniToSP.ready = False
                    MiRModeUniToSP.pause = False
                    MiRModeUniToSP.executing = False
                    MiRModeUniToSP.aborted = False
                    MiRModeUniToSP.completed = False
                    MiRModeUniToSP.docked = False
                    MiRModeUniToSP.docking = False
                    MiRModeUniToSP.emergency_stop = False
                    MiRModeUniToSP.manual_control = False
                    MiRModeUniToSP.error = False
                
            except rospy.ROSInterruptException:
                pass

            self.mir_mode_unidriver_to_sp_publisher.publish(self.mir_mode_state)
            self.main_rate.sleep()

        rospy.spin()

    
    def set_mir_mode_to_ready(self):
        self.mir_mode_unidriver_to_smaster_publisher.publish("set_mir_mode_to_ready")

    def set_mir_mode_to_paused(self):
        self.mir_mode_unidriver_to_smaster_publisher.publish("set_mir_mode_to_paused")

    def set_mir_mode_to_executing(self):
        self.mir_mode_unidriver_to_smaster_publisher.publish("set_mir_mode_to_executing")

    def set_mir_mode_to_aborted(self):
        self.mir_mode_unidriver_to_smaster_publisher.publish("set_mir_mode_to_aborted")


    def sp_to_mir_mode_unidriver_callback(self, mir_mode_cmd):

        self.sp_to_mir_mode_unidriver_timeout = time.time() + 2
        self.mir_mode_unidriver_got_msg_from_sp = True
        self.set_state_to_ready = mir_mode_cmd.set_state_to_ready
        self.set_state_to_paused = mir_mode_cmd.set_state_to_paused
        self.set_state_to_executing = mir_mode_cmd.set_state_to_executing
        self.set_state_to_aborted = mir_mode_cmd.set_state_to_aborted

        if self.set_state_to_ready == True and\
            self.set_state_to_paused == False and\
            self.set_state_to_executing == False and\
            self.set_state_to_aborted == False:
            self.set_mir_mode_to_ready()
        
        elif self.set_state_to_ready == False and\
            self.set_state_to_paused == True and\
            self.set_state_to_executing == False and\
            self.set_state_to_aborted == False:
            self.set_mir_mode_to_pause()

        elif self.set_state_to_ready == False and\
            self.set_state_to_paused == False and\
            self.set_state_to_executing == True and\
            self.set_state_to_aborted == False:
            self.set_mir_mode_to_executing()

        elif self.set_state_to_ready == False and\
            self.set_state_to_paused == False and\
            self.set_state_to_executin == False and\
            self.set_state_to_aborted == True:
            self.set_mir_mode_to_aborted()

        else:
            pass

        
    def mirModeCallback(self, mir_mode):
        self.mir_mode_smaster_to_unidriver_timeout = time.time() + 2
        self.mir_mode_unidriver_got_msg_from_mir_mode_smaster = True

        if mir_mode.data == "none":
            self.none = True
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False
            
        elif mir_mode.data == "starting":
            self.none = False
            self.starting = True
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "shutting_down":
            self.none = False
            self.starting = False
            self.shutting_down = True
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "ready":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = True
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "pause":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = True
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "executing":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = True
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False
        
        elif mir_mode.data == "aborted":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = True
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "completed":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = True
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "docked":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = True
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "docking":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = True
            self.emergency_stop = False
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "emergency_stop":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = True
            self.manual_control = False
            self.error = False

        elif mir_mode.data == "manual_control":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = True
            self.error = False

        elif mir_mode.data == "error":
            self.none = False
            self.starting = False
            self.shutting_down = False
            self.ready = False
            self.pause = False
            self.executing = False
            self.aborted = False
            self.completed = False
            self.docked = False
            self.docking = False
            self.emergency_stop = False
            self.manual_control = False
            self.error = True
        

if __name__ == '__main__':
    try:
        mir_mode_unidriver()
    except rospy.ROSInterruptException:
        pass