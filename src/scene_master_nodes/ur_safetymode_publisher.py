#!/usr/bin/env python

import json
import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Point
import tf
import time
import numpy
from math import pi


class ur_safetymode_publisher():

    def __init__(self):
        
        rospy.init_node('ur_safetymode_publisher', anonymous=False)
        self.scene_master_ur_safetymode_publisher = rospy.Publisher('ur_safetymode', String, queue_size=200)
        
        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(3)

        self.robot_safetymode_reader()
    
    def robot_safetymode_reader(self):
        def robot_safetymode_reader_callback():
            HOST = "0.0.0.0"
            PORT = 29999 
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))

            while (1):
                s.send ("safetymode" + "\n")
                data = s.recv(1024)
                #print data
                if not data: break
                self.tcp_rate.sleep()
                self.scene_master_ur_safetymode_publisher.publish(data)
            s.close()
        t7 = threading.Thread(target=robot_safetymode_reader_callback)
        t7.daemon = True
        t7.start()

        rospy.spin()
        
if __name__ == '__main__':
    try:
        ur_safetymode_publisher()
    except rospy.ROSInterruptException:
        pass
