#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # MiR Pose Scene Master
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
import json
import requests
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
import numpy
import sys
import tf
import time

class mir_pose_smaster():

    def __init__(self):
        
        rospy.init_node('mir_pose_smaster', anonymous=False)

        rospy.Subscriber("/unification_roscontrol/mir_pose_unidriver_to_smaster", String, self.mir_pose_unidriver_to_smaster_callback)

        self.mir_pose_to_unidriver_publisher = rospy.Publisher('unification_roscontrol/mir_pose_smaster_to_unidriver', String, queue_size=10)

        self.mir_rate = rospy.Rate(10)

        self.mir_pose_state = ""

        rospy.sleep(2)

        self.main()

# Set this PLC register to 1 to start movement
    def mir_start_movement(self):
        task = {"value" : "1"}
        resp = requests.post('http://192.168.1.100:8080/v1.0.0/registers/1', json=task)

    # Set desired MiR state
    def mir_pause_state(self):
        task = {"state" : 4}
        resp = requests.put('http://192.168.1.100:8080/v1.0.0/state', json=task)

    # Set desired MiR state
    def mir_executing_state(self):
        task = {"state" : 5}
        resp = requests.put('http://192.168.1.100:8080/v1.0.0/state', json=task)

    # Charging mission
    def mission_uni_demo_to_queue(self):
        task ={"mission" : "3e19ba3e-77f3-11e8-909f-f44d306bb564"} 
        resp = requests.post('http://192.168.1.100:8080/v1.0.0/mission_queue', json=task)
        #self.charging = 1
    
    # Delete the Mission Queue
    def delete_mir_mission_queue(self):
        resp = requests.delete('http://192.168.1.100:8080/v1.0.0/mission_queue')

    # Starting Mission 1
    def start_mission(self):
        self.delete_mir_mission_queue()
        time.sleep(2)
        self.mission_uni_demo_to_queue()
        time.sleep(2)
        self.mir_executing_state()
        time.sleep(2)
        self.mir_start_movement()


    def main(self):

        while not rospy.is_shutdown():

            self.mir_pose_to_unidriver_publisher.publish(self.mir_pose_state)

        self.mir_rate.sleep()

        rospy.spin()

    
    def mir_pose_unidriver_to_smaster_callback(self, data):
        if data.data == "mir_to_assembly":
            self.start_mission()
        else:
            pass


if __name__ == '__main__':
    try:
        mir_pose_smaster()
    except rospy.ROSInterruptException:
        pass