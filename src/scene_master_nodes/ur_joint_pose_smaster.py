#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Joint Pose Scene Master
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import socket
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from unification_roscontrol.msg import URJointSmasterToUni
import numpy
import time



class ur_joint_pose_smaster():

    def __init__(self):
        
        rospy.init_node('ur_joint_pose_smaster', anonymous=False)

        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_to_ur_joint_pose_smaster", String, self.ur_pose_unidriver_to_ur_joint_pose_smaster_callback)
        rospy.Subscriber("/joint_states", JointState, self.jointCallback)

        self.ur_joint_pose_to_unidriver_publisher = rospy.Publisher('unification_roscontrol/ur_joint_pose_smaster_to_unidriver', URJointSmasterToUni, queue_size=10)
        self.urScriptPublisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
    
        self.isclose_tolerance = 0.5
        self.go_to_joint_pose_prev = ""

        self.ur_joint_pose = ""
        self.ur_executing = False

        # Unification JOINT Poses
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.HomeJOINTPose = [0, 0.000000000, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.000000000000]
        self.PreAttachAtlasFarJOINTPose = [0.08005275577306747, -0.09790021577943975, 1.3764376640319824, 1.7152074575424194, -1.6239331404315394, 0.36349916458129883]
        self.PreAttachLFToolFarJOINTPose = [0.48709583282470703, -0.4043362776385706, 1.8112993240356445, -1.4140384832965296, -1.0778668562518519, 0.41233524680137634]
        self.PreAttachOFToolFarJOINTPose = [0.30180624127388, -0.37626773515810186, 2.060617446899414, -1.704867188130514, -1.2735651175128382, 0.41501858830451965]
        self.AboveEngineJOINTPose = [0.08036433905363083, -1.4317691961871546, 2.296618700027466, 2.2488040924072266, -1.664436165486471, 0.36170265078544617]

        self.joint_rate = rospy.Rate(125)

        rospy.sleep(2)

        self.main()



    def moveJ(self, jointPose, a=1.5, v=0.5, t=0, r=0):

        if len(jointPose) == 6:
            script_str = "movej(" + str(jointPose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(t) +  ", r=" + str(r) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            print "The joint pose size is not correct."



    def main(self):
        self.ur_joint_pose_state = URJointSmasterToUni()
        while not rospy.is_shutdown():

            URJointSmasterToUni.pose = self.ur_joint_pose
            URJointSmasterToUni.executing = self.ur_executing

            self.ur_joint_pose_to_unidriver_publisher.publish(self.ur_joint_pose_state)
            self.joint_rate.sleep()

        rospy.spin()


    
    def ur_pose_unidriver_to_ur_joint_pose_smaster_callback(self, joint_cmd):
        self.go_to_joint_pose = joint_cmd.data

        # There might be an issue during restart if the command coincides with the previous command after a fault

        if self.go_to_joint_pose == "HomeJOINT" and self.go_to_joint_pose_prev != "HomeJOINT":
            self.go_to_joint_pose_prev = "HomeJOINT"
            self.HomeJOINT()

        elif self.go_to_joint_pose == "reset" and self.go_to_joint_pose_prev != "reset":
            self.go_to_joint_pose_prev = "reset"
            self.ResetJOINT()

        elif self.go_to_joint_pose == "AboveEngineJOINT" and self.go_to_joint_pose_prev != "AboveEngineJOINT":
            self.go_to_joint_pose_prev = "AboveEngineJOINT"
            self.AboveEngineJOINT()

        elif self.go_to_joint_pose == "PreAttachAtlasFarJOINT" and self.go_to_joint_pose_prev != "PreAttachAtlasFarJOINT":
            self.go_to_joint_pose_prev = "PreAttachAtlasFarJOINT"
            self.PreAttachAtlasFarJOINT()

        elif self.go_to_joint_pose == "PreAttachLFToolFarJOINT" and self.go_to_joint_pose_prev != "PreAttachLFToolFarJOINT":
            self.go_to_joint_pose_prev = "PreAttachLFToolFarJOINT"
            self.PreAttachLFToolFarJOINT()

        elif self.go_to_joint_pose == "PreAttachOFToolFarJOINT" and self.go_to_joint_pose_prev != "PreAttachOFToolFarJOINT":
            self.go_to_joint_pose_prev = "PreAttachOFToolFarJOINT"
            self.PreAttachOFToolFarJOINT()
        
        elif self.go_to_joint_pose == "PreFindEngineJOINT" and self.go_to_joint_pose_prev != "PreFindEngineJOINT":
            self.go_to_joint_pose_prev = "PreFindEngineJOINT"
            self.PreFindEngineJOINT()

        elif self.go_to_joint_pose == "FindEngineRightJOINT" and self.go_to_joint_pose_prev != "FindEngineRightJOINT":
            self.go_to_joint_pose_prev = "FindEngineRightJOINT"
            self.FindEngineRightJOINT()

        elif self.go_to_joint_pose == "FindEngineLeftJOINT" and self.go_to_joint_pose_prev != "FindEngineLeftJOINT":
            self.go_to_joint_pose_prev = "FindEngineLeftJOINT"
            self.FindEngineLeftJOINT()

        elif self.go_to_joint_pose == "FindEngineMidJOINT" and self.go_to_joint_pose_prev != "FindEngineMidJOINT":
            self.go_to_joint_pose_prev = "FindEngineMidJOINT"
            self.FindEngineMidJOINT()

        else:
            pass


    def HomeJOINT(self):
        self.moveJ(self.HomeJOINTPose, a=1.5, v=5, t=3, r=0)

    def ResetJOINT(self):
        self.moveJ(self.ResetJOINTPose, a=1.5, v=5, t=3, r=0)

    def AboveEngineJOINT(self):
        self.moveJ(self.AboveEngineJOINTPose, a=1.5, v=5, t=3, r=0)

    def PreAttachAtlasFarJOINT(self):
        self.moveJ(self.PreAttachAtlasFarJOINTPose, a=1.5, v=5, t=3, r=0)

    def PreAttachLFToolFarJOINT(self):
        self.moveJ(self.PreAttachLFToolFarJOINTPose, a=1.5, v=5, t=3, r=0)

    def PreAttachOFToolFarJOINT(self):
        self.moveJ(self.PreAttachOFToolFarJOINTPose, a=1.5, v=5, t=3, r=0)

    def jointCallback(self, joint):

        self.ResetJOINTPose = [joint.position[0], joint.position[1], joint.position[2], joint.position[3], joint.position[4], joint.position[5]]

        #print numpy.isclose(joint.position[0], self.HomeJOINTPose[0], self.isclose_tolerance)
        #print joint.position[0]

        # maybe for pose in (pose1, pose2, pose3...)

        self.ur_joint_pose_name = []

        for pose in [self.HomeJOINTPose, 
            self.PreAttachAtlasFarJOINTPose, 
            self.PreAttachLFToolFarJOINTPose, 
            self.PreAttachOFToolFarJOINTPose,
            self.AboveEngineJOINTPose]:

            if  abs((abs(joint.position[0]) - abs(pose[0]))) < 0.01 and\
                abs((abs(joint.position[1]) - abs(pose[1]))) < 0.01 and\
                abs((abs(joint.position[2]) - abs(pose[2]))) < 0.01 and\
                abs((abs(joint.position[3]) - abs(pose[3]))) < 0.01 and\
                abs((abs(joint.position[4]) - abs(pose[4]))) < 0.01 and\
                abs((abs(joint.position[5]) - abs(pose[5]))) < 0.01:
                self.ur_joint_pose_name = pose
        
            if self.ur_joint_pose_name == self.HomeJOINTPose:
                self.ur_joint_pose = "HomeJOINT"
    
            elif self.ur_joint_pose_name == self.PreAttachAtlasFarJOINTPose:
                self.ur_joint_pose = "PreAttachAtlasFarJOINT"
    
            elif self.ur_joint_pose_name == self.PreAttachLFToolFarJOINTPose:
                self.ur_joint_pose = "PreAttachLFToolFarJOINT"
    
            elif self.ur_joint_pose_name == self.PreAttachOFToolFarJOINTPose:
                self.ur_joint_pose = "PreAttachOFToolFarJOINT"
    
            elif self.ur_joint_pose_name == self.AboveEngineJOINTPose:
                self.ur_joint_pose = "AboveEngineJOINT"
    
            else:
                self.ur_joint_pose = "unknown"
    

    
        if (joint.velocity[0] or joint.velocity[1] or joint.velocity[2] or joint.velocity[3] or joint.velocity[4] or joint.velocity[5] != 0):
            self.ur_executing = True
        else:
            self.ur_executing = False

if __name__ == '__main__':
    try:
        ur_joint_pose_smaster()
    except rospy.ROSInterruptException:
        pass

    