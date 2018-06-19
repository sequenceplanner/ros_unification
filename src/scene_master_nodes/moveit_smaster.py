#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR MoveIt Pose/Scene Scene Master
    # V.0.1.0.
#----------------------------------------------------------------------------------------
'''
import rospy
import roslib
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
import numpy
import sys
import tf
import time

class ur_moveit_smaster():

    def __init__(self):

        roscpp_initialize(sys.argv)
        
        rospy.init_node('ur_moveit_smaster', anonymous=False)

        self.scene = PlanningSceneInterface()
        self.robot = MoveGroupCommander("manipulator")

        self.moveit_planning = False

        #rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_moveit_smaster", String, self.ur_pose_unidriver_to_ur_moveit_smaster_callback)

        self.tcp_rate = rospy.Rate(10)

        rospy.sleep(5)

        self.main()


    def main(self):

        
        pos = Pose()
    
        pos.pose.position.x = -0.164
        pos.pose.position.y = -0.704
        pos.pose.position.z = 0.7293 #-0.672

        roll = 1.571
        pitch = 0
        yaw = 0

           # From Euler to Quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        ##type(pose) = geometry_msgs.msg.Pose
        pos.orientation.x = quaternion[0]
        pos.orientation.y = quaternion[1]
        pos.orientation.z = quaternion[2]
        pos.orientation.w = quaternion[3]
        

        pose = [0.0, 0.0, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.0]
        self.robot.plan(pose)

        while not rospy.is_shutdown():

            print self.robot.get_current_pose(end_effector_link = "tool0")

            self.tcp_rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    try:
        ur_moveit_smaster()
    except rospy.ROSInterruptException:
        pass
'''


