#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Gazebo Simulated MiR100 pose unidifation driver
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
import threading
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class fake_mir_unidriver():

    def __init__(self):
        
        rospy.init_node('fake_mir_unidriver', anonymous=False)

        self.fake_mir_pose_state = '_'    # Fake mir pose

        self.g_pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self.g_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self.g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
        self.pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10, latch=True)

        #rospy.Subscriber("/mir100_diff_drive_controller/odom", Odometry, self.odometryCallback)
        rospy.Subscriber("/sp_to_fake_mir_unidriver", String, self.sp_to_fake_mir_unidriver_callback)

        self.fake_mir_control_publisher = rospy.Publisher('/mir100_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)

        self.main_rate = rospy.Rate(10)
        
        self.main()


    #---------------------------------
    # Teleport Methods
    #---------------------------------
    def teleport1(self):
    
        rospy.wait_for_service("/gazebo/pause_physics")
        rospy.loginfo("Pausing physics")
    
        try:      
            self.g_pause()
            
        except Exception, e:
            rospy.logerr('Error on calling service: %s',str(e))
      
    
        pose = Pose()

        pose.position.x = 3
        pose.position.y = 3
        pose.position.z = 0

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
    
        state = ModelState()
    
        state.model_name = "robot"
        state.pose = pose
    
    
        rospy.loginfo("Moving robot")
        try:
            
            ret = self.g_set_state(state)
      
            print ret.status_message
            
        except Exception, e:
        
            rospy.logerr('Error on calling service: %s',str(e))
    
    
      
      
        rospy.loginfo("Unpausing physics") 
      
        try:
            
            self.g_unpause()
            
        except Exception, e:
        
            rospy.logerr('Error on calling service: %s',str(e))

        loc = PoseWithCovarianceStamped()
    
        loc.pose.pose = pose
        loc.header.frame_id = "base_footprint"
        loc.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
        rospy.loginfo("Adjusting localization")
        self.pub.publish(loc)
        self.pub.publish(loc)
        self.pub.publish(loc)

        rospy.sleep(1)
      



    #----------------------------------------------------------------------------------------
    # Fake MiR move methods
    #----------------------------------------------------------------------------------------
    def move1(self):
        vel_msg1 = Twist()
        vel_msg1.linear.x = -1
        vel_msg1.linear.y = 0
        vel_msg1.linear.z = 0
        vel_msg1.angular.x = 0
        vel_msg1.angular.y = 0
        vel_msg1.angular.z = 0

        self.fake_mir_control_publisher.publish(vel_msg1)

        timeout = time.time() + 8   # 5 second movement
    
       # while True:
            
        #    if time.time() > timeout:
        #        break

    def move2(self):
        vel_msg1 = Twist()
        vel_msg1.linear.x = 0
        vel_msg1.linear.y = 0
        vel_msg1.linear.z = 0
        vel_msg1.angular.x = 0
        vel_msg1.angular.y = 0
        vel_msg1.angular.z = 0

        self.fake_mir_control_publisher.publish(vel_msg1)

        #timeout = time.time() + 4   # 5 second movement
    
        #while True:
            
        #    if time.time() > timeout:
        #        break


    def rotate1(self):
        vel_msg1 = Twist()
        vel_msg1.linear.x = 0
        vel_msg1.linear.y = 0
        vel_msg1.linear.z = 0
        vel_msg1.angular.x = 0
        vel_msg1.angular.y = 0
        vel_msg1.angular.z = -1

        timeout = time.time() + 1.35 

        while True:
            self.fake_mir_control_publisher.publish(vel_msg1)
            if time.time() > timeout:
                break

    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        self.teleport1()
        #self.move1()
        #rospy.sleep(3)
        #self.move2()
        #rospy.sleep(3)
        #self.rotate1()
        

        rospy.spin()

    def sp_to_fake_mir_unidriver_callback(self, data):
        self.fake_mir_move = data.data
        if self.fake_mir_move == "move":
            self.move1()
        elif self.fake_mir_move == "dontmove":
            self.move2()
        else:
            pass



if __name__ == '__main__':
    try:
        fake_mir_unidriver()
    except rospy.ROSInterruptException:
        pass