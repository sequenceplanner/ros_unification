#! /usr/bin/env python
# Make sure that python is used as the interpretor.
# Note! #! is not a comment and the line must ve the first in the file

###################################### Filter tool node file ###############################################

# Imports
import rospy                            # Import the rospy module with a lot of ros functions in python
import roslib                           # Contains some ROS stuff
import threading
from std_msgs.msg import Int16          # Use the std_msgs/Int16 message type
from std_msgs.msg import UInt16          # Use the std_msgs/UInt16 message type 
import RPi.GPIO as GPIO                 # Import Raspberry pi GPIO module and call it GPIO in this module
import time                             # Time related functions
#import os

#os.nice(-20)                           # Not allowed for some reason


# Set up I/O
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)                  # The RP connector functional designations are used in setup
GPIO.setup(4, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(5, GPIO.IN)
GPIO.setup(6, GPIO.IN)
GPIO.setup(12, GPIO.IN)
GPIO.setup(13, GPIO.IN)
GPIO.setup(16, GPIO.IN)
GPIO.setup(19, GPIO.IN)


# Init Global variables. Has to be done!
RPM = 0                                 
DO1 = 0
DO2 = 0
DO3 = 0
DO4 = 0
SW1 = 0
SW2 = 0
SW3 = 0
SW4 = 0
SW5 = 0
SW6 = 0


#functions

#Define what should be done when message has been read
def callback_speed_dem(data):
    global RPM                          # Has to de declared as global to enable update
    RPM = float(data.data)              # Read data from topic       
    print(RPM)
  	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_ft_con(data):
    global DO1, DO2, DO3, DO4           # Has to de declared as global to enable update
    DO1 = data.data & 0x0001            # Read data from topic       
    DO2 = data.data & 0x0002            # Read data from topic   
    DO3 = data.data & 0x0004            # Read data from topic   
    DO4 = data.data & 0x0008            # Read data from topic   

#Define the filter tool node. Node name is filtertool
def filtertool():
    rospy.init_node('filtertool', anonymous=True)    # Tells rospy the name of the node
    #anonymous = True ensures that node has a unique name by adding random numbers to the end of it.


    ##################  Subscribers ############################################
    #The node is subscribing to the topic CT_FTspeed_dem of the type Int16.
    #After receiving the function callback_speed_dem is called.
    rospy.Subscriber("CT_FTspeed_dem", Int16, callback_speed_dem)
    rospy.Subscriber("CT_FT_con", UInt16, callback_ft_con)

    #################### Publishers ############################################
    #The node is publishing a message of standard type UInt16 to a topic named FT_status.
    #Queue_size is the max length of the queue before message is dropped. (Short queue -> higher prio.)
    #The name rpmpub is just a define of the stuff to the right of the =.
    statuspub = rospy.Publisher('FT_status', UInt16, queue_size=10)
    #ratestat = rospy.Rate(1)

    #The node is publishing a message of standard type Int16 to a topic named FT_SOC.
    socpub = rospy.Publisher('FT_SOC', Int16, queue_size=10)
    #ratesoc = rospy.Rate(0.1)

    #The node is publishing a message of standard type Int16 to a topic named FT_DI.
    DIpub = rospy.Publisher('FT_DI', UInt16, queue_size=10)



    #Initialisations
    GPIO.output(4, False)                    # Set GPIO to output low
    GPIO.output(17, False)                    # Set GPIO to output low
    GPIO.output(18, False)                    # Set GPIO to output low
    GPIO.output(27, False)                    # Set GPIO to output low
    pulse = False
    pulses_per_rpm = 200                      # no of pulser per RPM for selected stepper motor  
    pulseref = time.time()                     # set pulse reference time to MHz timer
    statref = time.time()                  # set status pub reference time to MHz timer
    socref = time.time()                  # set SOC pub reference time to MHz timer
    DIref = time.time()                  # set DI pub reference time to MHz timer
    pubratestat = 0.1                         # FT status publish rate in sec
    pubrateDI = 1                         # FT status publish DI in sec
    pubratesoc = 10                         # FT SOC publish rate in sec
    ft_soc = 1
    ft_status = 1
    ft_di = 0

    #Run loop  
    while not rospy.is_shutdown():

        ########### Set Dicrete outouts ########
        GPIO.output(4, DO1)                    # Set GPIO as commanded from CT
        GPIO.output(17, DO2)                 # Set GPIO as commanded from CT
        GPIO.output(18, DO3)                 # Set GPIO as commanded from CT
        GPIO.output(27, DO4)                 # Set GPIO as commanded from CT


        #Set timing variables
        pulsedifftime = time.time()-pulseref
        statdifftime = time.time()-statref
        socdifftime = time.time()-socref
        DIdifftime = time.time()-DIref

        ##### Control pulses to stepper motor #####  
        if RPM>10:  
          toggletime = 60/(float(RPM) * pulses_per_rpm)
        else:
          toggletime = pulsedifftime
          
        if RPM<10 or RPM>2000:                  # Outside defined rpm
          GPIO.output(23, False)                # Set GPIO to output low
          pulseref = time.time()              # set pulse reference time to MHz timer
          pulse = False   
        elif pulsedifftime>toggletime and pulse==False and RPM>10:
          GPIO.output(23, True)                 # Set GPIO to output high
          pulseref = time.time()              # set pulse reference time to MHz timer
          pulse = True
        elif pulsedifftime>toggletime and pulse==True and RPM>10:
          GPIO.output(23, False)                # Set GPIO to output low
          pulseref = time.time()              # set pulse reference time to MHz timer
          pulse = False


        ############# Publish stuff ###########   
        if statdifftime>pubratestat:
          statref=time.time()
          ft_status = ft_status+1                     # Calculate FT status
          statuspub.publish(ft_status)          #Publish FT status

        elif DIdifftime>pubrateDI:
          DIref=time.time()
          ########### Read dicrete inputs ########
          SW1=GPIO.input(5)                    # Read GPIO
          SW2=GPIO.input(6)                    # Read GPIO
          SW3=GPIO.input(12)                    # Read GPIO
          SW4=GPIO.input(13)                    # Read GPIO
          SW5=GPIO.input(16)                    # Read GPIO
          SW6=GPIO.input(19)                    # Read GPIO

          ft_di = SW1+(2*SW2)+(4*SW3)+(8*SW4)+(16*SW5)+(32*SW6)   # Set DI status as bits
          #print(SW1)
          DIpub.publish(ft_di)                #Publish FT DI

        elif socdifftime>pubratesoc:
          socref=time.time()
          ft_soc = ft_soc+1                           # Calculate FT SOC
          socpub.publish(ft_soc)                #Publish FT SOC

  
        #ratestat.sleep()
        #ratesoc.sleep()

if __name__ == '__main__':

# The try except below means if indicated exeption errror ocurres when calling function then
    # do not act on it
    try:
      filtertool()
    except rospy.ROSInterruptException():
      pass
    


