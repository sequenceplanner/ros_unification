#! /usr/bin/env python

###################################### HECU node file ###############################################

# Make sure that python is used as the interpretor.
# Note! #! is not a comment and the line must ve the first in the file

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

# Assign I/O numbers
GPO1 = 4
GPO2 = 17
GPO3 = 18
GPO4 = 27
GPI1 = 5
GPI2 = 6
GPI3 = 12
GPI4 = 13
GPI5 = 16
GPI6 = 19
GPI7 = 22
GPI8 = 23


# Set up I/O on RP
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)                  # The RP connector functional designati$
GPIO.setup(GPO1, GPIO.OUT)
GPIO.setup(GPO2, GPIO.OUT)
GPIO.setup(GPO3, GPIO.OUT)
GPIO.setup(GPO4, GPIO.OUT)

GPIO.setup(GPI1, GPIO.IN)
GPIO.setup(GPI2, GPIO.IN)
GPIO.setup(GPI3, GPIO.IN)
GPIO.setup(GPI4, GPIO.IN)
GPIO.setup(GPI5, GPIO.IN)
GPIO.setup(GPI6, GPIO.IN)
GPIO.setup(GPI7, GPIO.IN)
GPIO.setup(GPI8, GPIO.IN)


# Init Global variables. Has to be done                             
do_old = 0xFF
hecustat_old = 0


############# Other Initialisations ###############################################
GPIO.output(GPO1, False)             # Set GPIO to output low
GPIO.output(GPO2, False)            # Set GPIO to output low
GPIO.output(GPO3, False)            # Set GPIO to output low
GPIO.output(GPO4, False)            # Set GPIO to output low
pubratestat = 0.1                 # Status publish rate in sec
pubrateDI = 4                     # HW input status publish rate in sec
hecu_status = 1
pi6_di = 0


################  Callbacks on CT published data ############################

###### Generic DO set command callback (For test)
def callback_pi_do(data):               # Nibble 2 in PI_DO topic used
    global DO1, DO2, DO3, DO4, do_old   # Has to de declared as global to enabl$
    DO1 = data.data & 0x0100            # Read data from topic       
    DO2 = data.data & 0x0200            # Read data from topic   
    DO3 = data.data & 0x0400            # Read data from topic   
    DO4 = data.data & 0x0800            # Read data from topic
    ########### Set Dicrete outouts ########
    GPIO.output(GPO1, DO1)                 # Set GPIO as commanded from CT
    GPIO.output(GPO2, DO2)                 # Set GPIO as commanded from CT
    GPIO.output(GPO3, DO3)                 # Set GPIO as commanded from CT
    GPIO.output(GPO4, DO4)                 # Set GPIO as commanded from CT

    if (data.data & 0x0F00) != do_old:
      do_old = data.data & 0x0F00          # Nibble no 2
      print("hecu_do = %s " % hex(do_old))





#########  Define the hecu node. Node name is hecu ##############################
def hecu():
    rospy.init_node('hecu', anonymous=True)    # Tells rospy the name of the node
    #anonymous = True ensures that node has a unique name by adding random numbers to the end of it.


    ##################  Subscriber topics ############################################
    #The node is subscribing to the topic PI_DO of the type UInt16.
    #After receiving the function callback_pi_do is called.
    rospy.Subscriber("PI_DO", UInt16, callback_pi_do)


    #################### Publish topics  ############################################
    #Queue_size is the max length of the queue before message is dropped. (Short queue -> higher prio.)

    #Publish HECU status
    statuspub = rospy.Publisher('HECU_status', UInt16, queue_size=10)

    #Publish of HW input data
    dipub = rospy.Publisher('PI6_DI', UInt16, queue_size=10)



    ############# Initialisations ###############################################
    statref = time.time()                  # set status pub reference time to MHz timer
    DIref = time.time()                  # set DI pub reference time to MHz timer


    ###################### Run loop ############################################### 
    while not rospy.is_shutdown():
        #Update timing variables
        statdifftime = time.time()-statref
        DIdifftime = time.time()-DIref


############# Check and Publish AECU status if time ########################   
        if statdifftime>pubratestat:
          statref=time.time()
 
          #read all  DI
          DI1=GPIO.input(GPI1)                    # Read GPIO
          DI2=GPIO.input(GPI2)                    # Read GPIO
          DI3=GPIO.input(GPI3)                    # Read GPIO
          DI4=GPIO.input(GPI4)                    # Read GPIO
          DI5=GPIO.input(GPI5)                    # Read GPIO
          DI6=GPIO.input(GPI6)                    # Read GPIO
          DI7=GPIO.input(GPI7)                    # Read GPIO
          DI8=GPIO.input(GPI8)  

          LFhome = DI1
          FThome = DI2  

          # Calculate status
          hecu_status = 1+(2*DI1)+(4*DI2)                
          # Publish it
          statuspub.publish(hecu_status)       


############# Read and Publish all HW input status if time ########################   
        elif DIdifftime>pubrateDI:
          DIref=time.time()
          # Set DI status for pi6 as bits
          pi6_di = DI1+(2*DI2)+(4*DI3)+(8*DI4)+(16*DI5)+(32*DI6)+(64*DI7)+(128*DI8)
          dipub.publish(pi6_di)                   # Publish DI status

 
       

if __name__ == '__main__':

# The try except below means if indicated exeption errror ocurres when calling function then
    # do not act on it
    try:
      hecu()
    except rospy.ROSInterruptException():
      pass
    


