#! /usr/bin/env python

###################################### Filter tool node file ###############################################

# Make sure that python is used as the interpretor.
# Note! #! is not a comment and the line must ve the first in the file


# Imports
import rospy                            # Import the rospy module with a lot of ros functions in python
import roslib                           # Contains some ROS stuff
import threading
from std_msgs.msg import Int16          # Use the std_msgs/Int16 message type
from std_msgs.msg import UInt16  
import GPIOEmu as GPIO        # Use the std_msgs/UInt16 message type 
#import RPi.GPIO as GPIO                 # Import Raspberry pi GPIO module and call it GPIO in this module
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
GPIO.setmode(GPIO.BCM)                  # The RP connector functional designations are used in setup
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
recu_con=recu_con_old=0
do_old = 0xFF
connect_imp=0

############# Other Initialisations ###############################################
GPIO.output(GPO1, False)             # Set GPIO to output low
GPIO.output(GPO2, False)            # Set GPIO to output low
GPIO.output(GPO3, False)            # Set GPIO to output low
GPIO.output(GPO4, False)            # Set GPIO to output low
pubratestat = 0.1                 # RECU status publish rate in sec
pubrateDI = 4                     # HW input status publish rate in sec
recu_status = 0                   # Init to not defined stataus
pi1_di = 0



####### CT_RECU_con callback #######
def callback_ct_recu_con(data):
    if data: print "asdfasdf"
    global recu_con, recu_con_old       # Has to de declared as global to enable update
    recu_con = data.data                # Read data from topic
    if recu_con & 0x0003 == 1:
      GPIO.output(GPO1, 1)
    #if recu_con == 1:
    #  GPIO.output(GPO1, 1)
                   # Connect to tool 
    elif recu_con & 0x0003 == 2:
      GPIO.output(GPO1, 0)             # Disconnect from tool
 
    if (recu_con & 0x000C)>>2 == 1:
      GPIO.output(GPO2, 0)                # Release  LF       
    elif (recu_con & 0x000C)>>2 == 2:
      GPIO.output(GPO2, 1)                # Grab LF

    if recu_con != recu_con_old:       
      print("CT_RECU_con = %s " % hex(recu_con))
      recu_con_old = recu_con

###### Generic DO set command callback (For test)
def callback_pi_do(data):               # Nibble 0 in PI_DO topic used
    global DO1, DO2, DO3, DO4, do_old   # Has to de declared as global to enable update
    DO1 = data.data & 0x0001            # Read data from topic       
    DO2 = data.data & 0x0002            # Read data from topic   
    DO3 = data.data & 0x0004            # Read data from topic   
    DO4 = data.data & 0x0008            # Read data from topic   
    ########### Set Discrete outouts ########
    GPIO.output(GPO1, DO1)                 # Set GPIO as commanded from CT
    GPIO.output(GPO2, DO2)                 # Set GPIO as commanded from CT
    GPIO.output(GPO3, DO3)                 # Set GPIO as commanded from CT
    GPIO.output(GPO4, DO4)                 # Set GPIO as commanded from CT

    if (data.data & 0x000F) != do_old:
      do_old = data.data & 0x000F
      print("recu_do = %s " % hex(do_old))



#########  Define the recu node. Node name is recu ##############################
def recu():
    rospy.init_node('recu', anonymous=True)    # Tells rospy the name of the node
    #anonymous = True ensures that node has a unique name by adding random numbers to the end of it.



    ##################  Subscribed topics ############################################
    #The node is subscribing to the topic CT_RECU_con of the type UInt16.
    #After receiving the function callback_ct_recu_con is called.
    rospy.Subscriber("CT_RECU_con", UInt16, callback_ct_recu_con)         # Control from CT
    rospy.Subscriber("PI_DO", UInt16, callback_pi_do)                     # DO test from CT



    #################### Published topics  ############################################
    #Queue_size is the max length of the queue before message is dropped. (Short queue -> higher prio.)

    #Publish RECU status
    statuspub = rospy.Publisher('RECU_status', UInt16, queue_size=10)

    #Publish of all discrete input data (for test)
    dipub = rospy.Publisher('PI1_DI', UInt16, queue_size=10)



    ############# Initialisations ###############################################
    statref = time.time()                  # set status pub reference time to MHz timer
    DIref = time.time()     
    #GPIO.output(GPO1, 1)
    #GPIO.output(GPO1, 1)               # set DI pub reference time to MHz timer



    ###################### Run loop ############################################### 
    while not rospy.is_shutdown():
        #Update timing variables
        statdifftime = time.time()-statref
        DIdifftime = time.time()-DIref


        ############# Check and Publish RECU status if time ########################   
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

          #Calculate robot_tool_conn
          if not(DI3) and not(DI4) and not(DI5):
            robot_tool_conn = 1                 # Nothing connected
          elif DI3 and not(DI4) and not(DI5):
            robot_tool_conn = 4                 # Filter tool connected
          elif not(DI3) and DI4 and not(DI5):              
            robot_tool_conn = 3                 # Atlas tool connected
          elif not(DI3) and not(DI4) and DI5:              
            robot_tool_conn = 2                 # LF tool connected
          else:              
            robot_tool_conn = 5                 # Not plausable or undefined connection

          LFgrip_inp = GPIO.input(GPI1)+2*GPIO.input(GPI2)
          if LFgrip_inp == 0:
            lf_grip_conn = 1                    # LF not connected
          elif LFgrip_inp == 3:
            lf_grip_conn = 2                    # LF connected
          else:
            lf_grip_conn = 3                    # LF failed connection

 	  pressure_inp = GPIO.input(GPI7)
          if pressure_inp == 1:
            pressure_stat = 1                    # Pressure OK
          else:
            pressure_stat = 2                    # Pressure Failure

##################################################################################
############################## Add diagnose etc. to recu_status here ############
#################################################################################

          # Calculate recu status
          recu_status = robot_tool_conn + (16*lf_grip_conn)+ (128*pressure_stat) 
#          print("Recu status = %s" % hex(recu_status))
          statuspub.publish(recu_status)        # Publish recu status



############# Read and Publish HW input status if time ########################   
        elif DIdifftime>pubrateDI:
          DIref=time.time()
          # Set DI status for pi1 as bits
          pi1_di = DI1+(2*DI2)+(4*DI3)+(8*DI4)+(16*DI5)+(32*DI6)+(64*DI7)+(128*DI8)  # Set DI status as bits
          dipub.publish(pi1_di)                   # Publish DI status
 
       

if __name__ == '__main__':

# The try except below means if indicated exeption errror ocurres when calling function then
    # do not act on it
    try:
      recu()
    except rospy.ROSInterruptException():
      pass
    


