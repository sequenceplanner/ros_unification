#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # AGV pose publisher based on alvar code /ar_marker_0 
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import socket
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import tf
import threading

class agv_alvar_smaster():

    def __init__(self):

        rospy.init_node('agv_alvar_smaster', anonymous=False)

        self.tf_listener = tf.TransformListener()
        self.AGVAlvarPublisher = rospy.Publisher("agv_alvar_smaster_to_unidriver", Point, queue_size=10)

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.alvarCallback)
        
        time.sleep(1)

        self.do_pub_0 = False

        self.i = 0 

        self.rate = rospy.Rate(30)

        self.ar_marker_0_publisher()

    def alvarCallback(self, alvar_data):
        if alvar_data.markers:

            if alvar_data.markers[0].id == 0:
                self.do_pub_0 = True
            if alvar_data.markers[0].id != 0:
                self.do_pub_0 = False
                       
        else:
            self.do_pub_0 = False
            self.do_pub_1 = False
                           

    def ar_marker_0_publisher(self):
        while not rospy.is_shutdown():
            try:
                (self.trans_agv_0,self.rot_agv_0) = self.tf_listener.lookupTransform('/camera', '/ar_marker_0', rospy.Time(0))

                self.alvar_agv_0_x = self.trans_agv_0[0]
                self.alvar_agv_0_y = self.trans_agv_0[1]

                
                if self.do_pub_0 == True:
                    self.i = self.i + 1
                    # just to eliminate random ghost readings
                    if self.i > 3:
                        self.AGVAlvarPublisher.publish(self.alvar_agv_0_x, self.alvar_agv_0_y, 0)
                        self.i = 0
                    else:
                        pass
                else:
                    pass                

                self.rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


if __name__ == '__main__':
    try:
        agv_alvar_smaster()
        
    except rospy.ROSInterruptException:
        pass
