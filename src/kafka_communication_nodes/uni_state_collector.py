#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # State Collector for Unification Drivers
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import time
import random
import rospy
import roslib
import json
import threading
from std_msgs.msg import String

class uni_state_collector():

    def __init__(self):

        rospy.init_node('uni_state_collector', anonymous=False)
        self.main_pub = rospy.Publisher('driver_to_bridge', String, queue_size=10)

        # Universal Robots UR10 State
        rospy.Subscriber("ur_pose_unistate", String, self.ur_pose_unistate_callback)    # Universal Robots UR10 Pose State
        rospy.Subscriber("ur_mode_unistate", String, self.ur_mode_unistate_callback)    # Universal Robots UR10 Mode State

        # Mobile Industrial Robots MiR100 State
        rospy.Subscriber("mir_pose_unistate", String, self.mir_pose_unistate_callback)  # Mobile Industrial Robots MiR100 Pose State
        rospy.Subscriber("mir_mode_unistate", String, self.mir_mode_unistate_callback)  # Mobile Industrial Robots MiR100 Mode State

        # RSP Connector Electronic Control Unit State
        rospy.Subscriber("recu_rtc_unistate", String, self.recu_rtc_unistate_callback)  # RECU Robot-Tool Connection State
        rospy.Subscriber("recu_lfg_unistate", String, self.recu_lfg_unistate_callback)  # RECU LF Grip State

        # Atlas Twin Spin Tool Electronic Control Unit State
        rospy.Subscriber("aecu_atr_unistate", String, self.aecu_atr_unistate_callback)  # AECU Atlas Tool Run State
        rospy.Subscriber("aecu_atp_unistate", String, self.aecu_atp_unistate_callback)  # AECU Atlas Tool Position State
        rospy.Subscriber("aecu_atq_unistate", String, self.aecu_atq_unistate_callback)  # AECU Atlas Tool Torque State

        # Home Station Electronic Control Unit State
        rospy.Subscriber("hecu_hca_unistate", String, self.hecu_hca_unistate_callback)  # HECU Alive or Not
        rospy.Subscriber("hecu_lft_unistate", String, self.hecu_lft_unistate_callback)  # HECU LF Tool Home or Not
        rospy.Subscriber("hecu_oft_unistate", String, self.hecu_oft_unistate_callback)  # HECU Filter Tool Home or Not

        # AGV and MiR Alvar Tag States

        
        self.main_rate = rospy.Rate(10)
        self.ur_pose = "_"
        self.ur_mode = "_"
        #self.hmn_state = "_"
        #self.grp_state = "_"
        self.uni_system_state = "_"

        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        def main_callback():
            while not rospy.is_shutdown():

                #self.uni_system_state = str({"ur_unidriver" : self.ur_state, "human" : self.hmn_state, "gripper" : self.grp_state})
                
                #system state consists only of ur state
                self.uni_system_state = str({"actPos" : self.ur_pose, "URMode" : self.ur_mode})

                #self.main_pub.publish(json.dumps(self.uni_system_state))
                self.main_pub.publish(self.uni_system_state)
                self.main_rate.sleep()
        t = threading.Thread(target=main_callback)
        t.daemon = True
        t.start()
    
        rospy.spin()
    

    #----------------------------------------------------------------------------------------
    # Callbacks for discrete unidriver states
    #----------------------------------------------------------------------------------------
    def ur_pose_unistate_callback(self, ur_pose_data):
        self.ur_pose = ur_pose_data.data

    def ur_mode_unistate_callback(self, ur_mode_data):
        self.ur_mode = ur_mode_data.data

    #def hmn_uni_state_callback(self, hmn_state_data):
    #    self.hmn_state = hmn_state_data.data

    #def grp_uni_state_callback(self, grp_state_data):
    #    self.grp_state = grp_state_data.data


if __name__ == '__main__':
    try:
        uni_state_collector()
    except rospy.ROSInterruptException:
        pass
