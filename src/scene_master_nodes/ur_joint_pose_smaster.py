#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Joint Pose Scene Master
    # V.0.3.0.
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

        #self.HomeJOINTPose = [0, 0.000000000, 1.5707963705062866, 1.5707963705062866, -1.570796314870016, 0.000000000000]
        self.HomeJOINTPose = [0.8492594957351685, 0.013160228729248047, 1.1464190483093262, -1.1650527159320276, 0.7387031316757202, -1.350863281880514]
        self.PreHomeJOINTPose = [0.8324329257011414, 0.5406302213668823, -1.906015698109762, -0.2643502394305628, 1.6102402210235596, -0.1498802343951624]
        self.PreAttachAtlasFarJOINTPose = [0.08005275577306747, -0.09790021577943975, 1.3764376640319824, 1.7152074575424194, -1.6239331404315394, 0.36349916458129883]
        self.PreAttachLFToolFarJOINTPose = [0.48709583282470703, -0.4043362776385706, 1.8112993240356445, -1.4140384832965296, -1.0778668562518519, 0.41233524680137634]
        self.PreAttachOFToolFarJOINTPose = [0.30180624127388, -0.37626773515810186, 2.060617446899414, -1.704867188130514, -1.2735651175128382, 0.41501858830451965]
        self.AboveEngineJOINTPose = [0.08036433905363083, -1.4317691961871546, 2.296618700027466, 2.2488040924072266, -1.664436165486471, 0.36170265078544617]

        self.PreFindEngineJOINTPose = [0.045610461384058, -1.7011755148517054, 1.6724138259887695, -3.188571278248922, 1.467016339302063, 0.32854679226875305]
        self.FindEngineRightUpJOINTPose = [0.7311691641807556, -1.1983607451068323, 1.1075925827026367, -3.0703700224505823, 2.441229820251465, 0.41306591033935547]
        self.FindEngineLeftUpJOINTPose = [-0.4624560515033167, -1.0342209974872034, 0.8818979263305664, -2.8513134161578577, 0.4796641170978546, 1.461244821548462]
        self.FindEngineMidUpJOINTPose = [-0.06745511690248662, -2.8873093763934534, 2.1933436393737793, -2.5130417982684534, -1.5144537130938929, 1.4326841831207275]

        self.LFOperationMidpoint1JOINTPose = [0.41999128460884094, 0.1648104190826416, 1.1107091903686523, -1.2837541739093226, -1.2063325087176722, 0.48322567343711853]
        self.LFOperationMidpoint2JOINTPose = [-0.2440126577960413, 0.2491534948348999, 1.2506699562072754, -1.5690816084491175, -1.8519170920001429, 0.4272848069667816]
        self.LFOperationMidpoint3JOINTPose = [-1.3039653936969202, 0.11497437953948975, 0.14282608032226562, -0.4018920103656214, 3.024214029312134, -1.2089689413653772]
        self.LFOperationMidpoint4JOINTPose = [-2.557596508656637, 0.11800789833068848, -0.032685105000631154, -0.8492801825152796, 1.3278743028640747, -0.08603126207460576]
        self.LFOperationMidpoint5JOINTPose = [-2.6127448717700403, 0.18079030513763428, -1.8194859663592737, 0.09791183471679688, 1.534314513206482, -0.23677331606020147]
        self.AfterLFOperationJOINTPose = [0.18237031996250153, 0.3903089761734009, -1.5529182592975062, -0.4450719992267054, 1.6995275020599365, -1.2665088812457483]

        self.OFMidpoint1JOINTPose = [0.35248976945877075, -0.38250524202455694, 2.0415005683898926, 0.6728662252426147, -0.8877318541156214, 0.4149826467037201]
        self.OFMidpoint2JOINTPose = [0.41940370202064514, -0.8982942740069788, 1.4213886260986328, 1.0498987436294556, -1.5416081587420862, 0.4149826467037201]

        self.AboveUntightenedOF1JOINTPose = [0.47172942757606506, -1.187561337147848, 1.5113859176635742, 1.2485429048538208, -1.5618355909930628, -5.71590227285494]
        self.AtUntightenedOF1JOINTPose = [0.4717414379119873, -1.2069280783282679, 1.4626832008361816, 1.316667914390564, -1.5620277563678187, -5.716369632874624]
        self.AtTightenedOF1JOINTPose = [0.4716571569442749, -1.2069161573993128, 1.4626951217651367, 1.3166199922561646, -1.562171761189596, 0.6051022410392761]
        self.AboveTightenedOF1JOINTPose = [0.47168150544166565, -1.1898048559771937, 1.5067229270935059, 1.2552989721298218, -1.5617640654193323, 0.605629563331604]

        self.AboveUntightenedOF2JOINTPose = [0.40008997917175293, -1.0357564131366175, 1.278336524963379, 1.3280595541000366, -1.5612967650042933, -3.849708143864767]
        self.AtUntightenedOF2JOINTPose = [0.4001499116420746, -1.0482481161700647, 1.2359981536865234, 1.3829470872879028, -1.561512295399801, -3.850152079259054]
        self.AtTightenedOF2JOINTPose = [0.40016189217567444, -1.0482600370990198, 1.2359857559204102, 1.3828750848770142, -1.561500374470846, 2.4674112796783447]
        self.AboveTightenedOF2JOINTPose = [0.4001499116420746, -1.0345085302936, 1.2821359634399414, 1.3229809999465942, -1.5613206068622034, 2.4680707454681396]

        self.pose_names =["HomeJOINT",
            "PreHomeJOINT",
            "PreAttachAtlasFarJOINT",
            "PreAttachLFToolFarJOINT", 
            "PreAttachOFToolFarJOINT", 
            "AboveEngineJOINT",
            "PreFindEngineJOINT",
            "FindEngineRightUpJOINT",
            "FindEngineLeftUpJOINT",
            "FindEngineMidUpJOINT",
            "LFOperationMidpoint1JOINT",
            "LFOperationMidpoint2JOINT",
            "LFOperationMidpoint3JOINT",
            "LFOperationMidpoint4JOINT",
            "LFOperationMidpoint5JOINT",
            "AfterLFOperationJOINT",
            "OFMidpoint1JOINT",
            "OFMidpoint2JOINT",
            "AboveUntightenedOF1JOINT",
            "AtUntightenedOF1JOINT",
            "AtTightenedOF1JOINT",
            "AboveTightenedOF1JOINT",
            "AboveUntightenedOF2JOINT",
            "AtUntightenedOF2JOINT", 
            "AtTightenedOF2JOINT",
            "AboveTightenedOF2JOINT"]

        self.joint_rate = rospy.Rate(125)

        rospy.sleep(2)

        self.main()



    def moveJ(self, jointPose, a=0.5, v=0.5, t=0, r=0):

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

        if self.go_to_joint_pose == "reset":
            self.go_to_joint_pose_prev = "reset"
        
    
        # This is awesome...
        for pose in self.pose_names:
            if self.go_to_joint_pose == pose and self.go_to_joint_pose_prev != pose:
                self.go_to_joint_pose_prev = pose
                joint_pose = "self." + pose + "Pose"
                method = "self.moveJ(" + joint_pose + ", a=0.5, v=0.5, t=0, r=0)"
                eval(method)
            else:
                pass

    def jointCallback(self, joint):

        self.ResetJOINTPose = [joint.position[0], joint.position[1], joint.position[2], joint.position[3], joint.position[4], joint.position[5]]

        self.ur_joint_pose_name = []

        for pose in [self.HomeJOINTPose,
            self.PreHomeJOINTPose,
            self.PreAttachAtlasFarJOINTPose,
            self.PreAttachLFToolFarJOINTPose,
            self.PreAttachOFToolFarJOINTPose,
            self.AboveEngineJOINTPose,
            self.PreFindEngineJOINTPose,
            self.FindEngineRightUpJOINTPose, 
            self.FindEngineLeftUpJOINTPose,
            self.FindEngineMidUpJOINTPose,
            self.LFOperationMidpoint1JOINTPose,
            self.LFOperationMidpoint2JOINTPose,
            self.LFOperationMidpoint3JOINTPose,
            self.LFOperationMidpoint4JOINTPose,
            self.LFOperationMidpoint5JOINTPose,
            self.AfterLFOperationJOINTPose,
            self.OFMidpoint1JOINTPose,
            self.OFMidpoint2JOINTPose,
            self.AboveUntightenedOF1JOINTPose,
            self.AtUntightenedOF1JOINTPose,
            self.AtTightenedOF1JOINTPose,
            self.AboveTightenedOF1JOINTPose, 
            self.AboveUntightenedOF2JOINTPose,
            self.AtUntightenedOF2JOINTPose,
            self.AtTightenedOF2JOINTPose,
            self.AboveTightenedOF2JOINTPose]:

            if  abs((abs(joint.position[0]) - abs(pose[0]))) < 0.003 and\
                abs((abs(joint.position[1]) - abs(pose[1]))) < 0.003 and\
                abs((abs(joint.position[2]) - abs(pose[2]))) < 0.003 and\
                abs((abs(joint.position[3]) - abs(pose[3]))) < 0.003 and\
                abs((abs(joint.position[4]) - abs(pose[4]))) < 0.003 and\
                abs((abs(joint.position[5]) - abs(pose[5]))) < 0.003:
                self.ur_joint_pose_name = pose
                
            else:
                self.ur_joint_pose = "unknown"

        # This is also awesome...
        for pose2 in self.pose_names:
            joint_pose2 = "self." + pose2 + "Pose"
            if self.ur_joint_pose_name == eval(joint_pose2):
                ur_joint_pose_a = joint_pose2.replace("self.", "")
                self.ur_joint_pose = ur_joint_pose_a.replace("Pose", "")
            else:
                pass

        if (joint.velocity[0] or joint.velocity[1] or joint.velocity[2] or joint.velocity[3] or joint.velocity[4] or joint.velocity[5] != 0):
            self.ur_executing = True
        else:
            self.ur_executing = False

if __name__ == '__main__':
    try:
        ur_joint_pose_smaster()
    except rospy.ROSInterruptException:
        pass

    
