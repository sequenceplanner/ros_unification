#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import numpy
import math
import os
import socket
import atexit

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

HOST = "192.168.1.11"
PORT = 29999
#HOST = 'localhost'

#--------------------------------------------------------------------  

class Test1():

    def __init__(self):
        
        # Start listener:
        #self.listener = tf.TransformListener()
	
	
        self.init = rospy.Rate(30)
        self.io_service = rospy.ServiceProxy('/ur_driver/set_io', SetIO, self.cb5)


            # Wait for topics to start publishing
        rospy.Subscriber("/joint_states", JointState, self.cb1)
        while(True):
            try:
                self.joints
                rospy.loginfo("/joint_states found")
                break
            except (AttributeError):
                self.init.sleep()
                continue

        rospy.Subscriber("/wrench",WrenchStamped, self.cb2)
        while(True):
            try:
                self.force
                self.wrench
                rospy.loginfo("/wrench found")
                break
            except (AttributeError):
                self.init.sleep()
                continue


        rospy.Subscriber("/ethdaq_data",WrenchStamped, self.cb3)
        while(True):
            try:
                self.T
                self.F
                rospy.loginfo("/ethdaq_data found")
                break
            except (AttributeError):
                self.init.sleep()
                continue
            
        rospy.Subscriber("/ur_driver/io_states", IOStates, self.cb4)
        while(True):
            try:
                self.IO
                rospy.loginfo("/io_states found")
                break
            except (AttributeError):
                self.init.sleep()
                continue



        # Publish to robot
        self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

        # Go into spin, with rateOption!
        self.rate = rospy.Rate(10)          # 10hz


        IO = SetIO()
        IO.state = 0
        IO.pin = 0
        IO.fun = 1
        self.io_service.call(IO.fun,IO.pin,IO.state)
        self.set_IO_states(1, 0, 0)

        self.lifting = False
        self.freedrive = True

        self.loadNPlay('/programs/CollabForce_Freedrive.urp')

        IO = SetIO()
        IO.state = 1
        IO.pin = 7
        IO.fun = 1
        self.io_service.call(IO.fun,IO.pin,IO.state)

        atexit.register(self.exit_handler)
	

#CBs ----------------------------------------------------------------
#--------------------------------------------------------------------
    def cb1(self, data):
        # Get update from the manipulator:
        self.joints = data.position

    def cb2(self,data):
	# Get update from force sensors on manipulator
        self.force = data.wrench.force
        self.wrench = data.wrench.torque
    
    def cb3(self,data):
        #Get update from optoforce force sensor
        self.T = data.wrench.torque
        self.F = data.wrench.force
        #print self.F.z

    def cb4(self,data):
        self.IO = data      

    def cb5(self,data):
        self.IO = data
        print 'wrote IO cb5'
 
# spin --------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()):

            start_time = rospy.get_rostime() 
            end = time.time()

            if self.lifting and not self.freedrive and self.F.z < -100:
                #Release frame
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Release.urp')

                IO = SetIO()
                IO.state = 1
                IO.pin = 7
                IO.fun = 1
                self.io_service.call(IO.fun,IO.pin,IO.state)
                while not self.IO.digital_out_states[7].state:
                    self.init.sleep()
                while self.IO.digital_out_states[7].state:
                    self.init.sleep()


                #Go into freedrive
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Freedrive.urp')

                self.freedrive = True
                self.lifting = False
                print 'Going into freedrive mode!\n------------------------------------------------'
                time.sleep(3)


            elif not self.lifting and self.freedrive and self.F.z < -80:
                #Grip frame		
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Grip.urp')


                IO = SetIO()
                IO.state = 1
                IO.pin = 7
                IO.fun = 1
                self.io_service.call(IO.fun,IO.pin,IO.state)
                while not self.IO.digital_out_states[7].state:
                    self.init.sleep()
                while self.IO.digital_out_states[7].state:
                    self.init.sleep()


                #Go into lifting mode
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Follow_ZUnlockHalfForce.urp')
                self.freedrive = False
                self.lifting = True
                print 'Going into lifting mode!\n------------------------------------------------'
                time.sleep(3)

            
            # Go into spin, with rateOption!
            self.rate.sleep()        

#--------------------------------------------------------------------
    def speedl(self,fx,fy,fz,rx,ry,rz,a,tmin):
        return "speedl([" + str(fx) + "," + str(fy) + "," + str(fz) + "," + str(rx) + "," + str(ry) + "," + str(rz) + "],"+ str(a) +", "+ str(tmin) +")"

    def command_mode(self,mode):
        # cmd = "def command_mode():\n\n\twhile (True):\n\t\tspeedl([0,0,-1,0,0,0],1,0.1)sync()\n\tend\nend"
        #cmd = "def command_mode():\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\n\tcommand_mode()\nend\n"
        cmd = 'freedrive_mode()\nwhile True:\nsleep(1)\nend'
        print cmd
        return cmd


    def tcp(self, cmd):
        # https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        self.init.sleep()
        s.recv(1024)
        s.send (cmd + "\n")
        self.init.sleep()
        self.msg = s.recv(1024)
        print self.msg
        s.close()


    def loadNPlay(self,cmd):
        print '======= Begin Loading:'
        self.tcp('load ' + cmd)
        self.init.sleep()
        self.loaded = 0
        self.played = 0

        # check if the program is loaded and played!
        while self.loaded != 1 and self.played != 1:
            self.tcp('get loaded program')
            if cmd in self.msg:
                self.loaded = 1
                self.tcp('programState')
                if 'PLAYING' in self.msg:
                    self.played = 1
                else:   
                    # print 'message:' + self.msg
                    self.tcp('play')
                    print '! ->  Trying to play again!\n'
            else:
                self.tcp('load ' + cmd)
                self.init.sleep()
                print '! ->  Trying to load again!\n'

        print '======= End Loading'
        return True

    def exit_handler(self):
        print '\n'
        self.tcp('stop')
        print 'Program terminated by user!'

#--------------------------------------------------------------------
#--------------------------------------------------------------------
    def set_IO_states(self,fun, pin, state):
        IO = SetIO()
        IO.state = 0
        IO.pin = 0
        IO.fun = 1
        self.io_service.call(IO.fun,IO.pin,IO.state)

        rospy.wait_for_service('/ur_driver/set_io')
        # print "IO States"
        try:
            set_io = rospy.ServiceProxy('/ur_driver/set_io', SetIO)
            resp = set_io(fun, pin, state)
            # print "IO pin %d state %d" % (pin , state)
            return resp
        except rospy.ServiceException, e:
            print "Service call Failed: %s"%e
            
        self.io_service.call(fun,pin,state)
# #--------------------------------------------------------------------    
 

            
# Here is the main entry point
if __name__ == '__main__':
    try:
        # Init the node:
        rospy.init_node('pickNPlace')


        start = time.time()
        # Initializing and continue running the class Test1:

	a = Test1();
	a.spin()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass

