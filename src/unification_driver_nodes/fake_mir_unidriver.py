#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Gazebo Simulated MiR100 pose unification driver
    # V.0.4.0.
#----------------------------------------------------------------------------------------

import json
import rospy
import roslib
import socket
import struct
import tf
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


# no teleporting (makes mess)
# initial moves to kitting stateion:
    # 1 - move 1
    # 2 - rotate 1
    # 3 - move 2

# moves back to assembly stateion:
    # 1 - move 3 (waits for agv)
    # 2 - move 4 (to final pos)

class fake_mir_unidriver():

    def __init__(self):
        
        rospy.init_node('fake_mir_unidriver', anonymous=False)

        self.fake_mir_pose_state = '_'    # Fake mir pose

        self.available = False

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
    # Teleport Methods , use with caution, messes with physics
    #---------------------------------

    def final_teleport(self):
        rospy.wait_for_service("/gazebo/pause_physics")
        rospy.loginfo("Pausing physics")
        try:    
            self.g_pause()   
        except Exception, e:
            rospy.logerr('Error on calling service: %s',str(e))

        qpos = Pose()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.5707)
        
        qpos.orientation.x = quaternion[0]
        qpos.orientation.y = quaternion[1]
        qpos.orientation.z = quaternion[2]
        qpos.orientation.w = quaternion[3]

        pose = Pose()
        
        pose.position.x = - 0.52
        pose.position.y = 0.188
        pose.position.z = 0
        
        pose.orientation.x = qpos.orientation.x
        pose.orientation.y = qpos.orientation.y
        pose.orientation.z = qpos.orientation.z
        pose.orientation.w = qpos.orientation.w
        
        state = ModelState()
        
        state.model_name = "robot"
        state.pose = pose

        rospy.loginfo("Moving robot")
        try:
            ret = self.g_set_state(state)
            print ret.status_message 
        except Exception, e:
            rospy.logerr('Error on calling service: %s',str(e))

        rospy.loginfo("Physics paused, MiR not usable")

        '''
        rospy.loginfo("Unpausing physics") 

        
        try:
                
            self.g_unpause()
                
        except Exception, e:
            
            rospy.logerr('Error on calling service: %s',str(e))
        '''

    
    #----------------------------------------------------------------------------------------
    # Fake MiR move methods
    #----------------------------------------------------------------------------------------
    def move1(self):
        vel_msg = Twist()
        vel_msg.linear.x = -1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        timeout = time.time() + 1.1   # 1.05 second movement
    
       # If SP not ticking, use while
        while True:
            self.fake_mir_control_publisher.publish(vel_msg)
            if time.time() > timeout:
                break

    def move2(self):
        vel_msg = Twist()
        vel_msg.linear.x = - 1 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        timeout = time.time() + 10   # 10 second movement
    
        # If SP not ticking, use while
        while True:
            self.fake_mir_control_publisher.publish(vel_msg)
            if time.time() > timeout:
                break

        self.available = True

    def move3(self):
        vel_msg = Twist()
        vel_msg.linear.x = 1 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        timeout = time.time() + 3.95   # 3.9 second movement
    
        # If SP not ticking, use while
        while True:
            self.fake_mir_control_publisher.publish(vel_msg)
            if time.time() > timeout:
                break


    def rotate1(self):
        
        rospy.sleep(2)

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -1

        timeout = time.time() + 1.4  # 1.4 second movement

        # If SP not ticking, use while
        while True:
            self.fake_mir_control_publisher.publish(vel_msg)
            if time.time() > timeout:
                break

    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        
        self.move1()
        rospy.sleep(3)
        self.rotate1()
        rospy.sleep(3)
        self.move2()
        #rospy.sleep(12)
        #self.move3()

        rospy.spin()

    def sp_to_fake_mir_unidriver_callback(self, data):
        self.fake_mir_move = data.data
        if self.available == True and self.fake_mir_move == "move":
            self.move3()
            rospy.sleep(6)
            # self.final_teleport()
        else:
            pass



if __name__ == '__main__':
    try:
        fake_mir_unidriver()
    except rospy.ROSInterruptException:
        pass