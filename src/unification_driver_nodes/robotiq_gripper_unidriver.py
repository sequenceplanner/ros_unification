#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Unification Driver for the Robotiq Gripper
    # V.0.2.0.
#----------------------------------------------------------------------------------------

import time
import json
import rospy
import roslib
import threading
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from digilab_control.cModelGripper import CModelGripper
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

#--------------------------------------------------------------------------------------------------------------------
# the main class
#--------------------------------------------------------------------------------------------------------------------
class robotiq_gripper_unidriver(CModelGripper):

    def __init__(self):
        
        rospy.init_node('robotiq_gripper_unidriver', anonymous=False)
        CModelGripper.__init__(self)

        rospy.Subscriber("/CModelRobotInput", inputMsg.CModel_robot_input, self.gripperCallback)
        rospy.Subscriber("/bridge_to_driver", String, self.sp_to_driver_callback)
        self.gripper_state_publisher = rospy.Publisher('grp_uni_state', String, queue_size=10)
        self.message_ack_publisher = rospy.Publisher('driver_to_bridge', String, queue_size=10)
        rospy.sleep(1)

        self.gripper_state = '_'
        self.resetGripper()
        rospy.sleep(1)
        self.activateGripper()
        self.state_rate = rospy.Rate(10)       

        self.main()

    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        def main_callback():
            while not rospy.is_shutdown():
                self.gripper_state_publisher.publish(self.gripper_state)
                self.state_rate.sleep()
        t = threading.Thread(target=main_callback)
        t.daemon = True
        t.start()
    
        rospy.spin()

    #----------------------------------------------------------------------------------------
    # Main callback for external communication
    #----------------------------------------------------------------------------------------
    def sp_to_driver_callback(self, cmd_data):
        self.sp_to_driver = json.loads(cmd_data.data)
        if self.sp_to_driver['receiver'] == "grp_unidriver":
            if self.sp_to_driver['command'] == "open_gripper":
                self.message_ack_publisher.publish("grp_unidriver got cmd: open_gripper")
                self.openGripper()
            elif self.sp_to_driver['command'] == "close_gripper":
                self.message_ack_publisher.publish("grp_unidriver got cmd: close_gripper")
                self.closeGripper()
            else:
                pass
        else:
            pass

    #----------------------------------------------------------------------------------------
    # Discretizing gripper state from gripper callback
    #----------------------------------------------------------------------------------------
    def gripperCallback(self, obj):
        self.gripper_data = obj

        if obj.gPO < 10:
            self.gripper_state = 'gripperOpen'
        elif obj.gPO > 221:
            self.gripper_state = 'gripperClosed'
        elif obj.gPO > 110 and obj.gPO < 120:
            self.gripper_state = 'gripperBig'
        elif obj.gPO > 125 and obj.gPO < 135:
            self.gripper_state = 'gripperSmall'
        elif obj.gPO > 210 and obj.gPO < 220:
            self.gripper_state = 'gripperBolt'
        elif obj.gPO > 190 and obj.gPO < 205:
            self.gripper_state = 'gripperNut'
        else:
            self.gripper_state = 'gripperError'


if __name__ == '__main__':
    try:
        robotiq_gripper_unidriver()
    except rospy.ROSInterruptException:
        pass
