#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Mode Scene Master
    # V.0.5.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
import struct
import threading
from std_msgs.msg import String
import time

#HOST = "192.168.1.14"
#DASHBOARD_SERVER = 29999

class ur_mode_smaster():

    def __init__(self):
        
        rospy.init_node('ur_mode_smaster', anonymous=False)

        self.scene_master_ur_safetymode_publisher = rospy.Publisher('unification_roscontrol/ur_mode_smaster_to_unidriver', String, queue_size=200)
        
        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(3)

        self.robot_safetymode_reader()
    
    def robot_safetymode_reader(self):
        def robot_safetymode_reader_callback():
            HOST = "192.168.1.14"
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
        ur_mode_smaster()
    except rospy.ROSInterruptException:
        pass
