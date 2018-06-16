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
import GPIOEmu as GPIO     # Use the std_msgs/UInt16 message type 
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
aecu_con=aecu_con_old=0
do_old = 0xFF
recustat_old = 0


############# Other Initialisations ###############################################
GPIO.output(GPO1, False)             # Set GPIO to output low
GPIO.output(GPO2, False)            # Set GPIO to output low
GPIO.output(GPO3, False)            # Set GPIO to output low
GPIO.output(GPO4, False)            # Set GPIO to output low
pubratestat = 0.1                 # RECU status publish rate in sec
pubrateDI = 4                     # HW input status publish rate in sec
aecu_status = 1
pi3_di = 0
ct_idle = True
ct_fwd = False
ct_rev = False
ct_inh = False
robot_has_atlas = False

################  Callbacks on CT published data ############################
def callback_ct_aecu_con(data):
    global aecu_con, aecu_con_old       # Has to de declared as global to enable update
    global ct_idle, ct_fwd, ct_rev, ct_inh

    aecu_con = data.data                # Read data from topic
    
    # Read command from CT 
    if aecu_con & 0x01C == 4:
      ct_idle = True
      ct_fwd = False
      ct_rev = False
      ct_inh = False
    elif aecu_con & 0x01C == 8:
      ct_idle = False
      ct_fwd = True
      ct_rev = False
      ct_inh = False
    elif aecu_con & 0x01C == 0x0C:
      ct_idle = False
      ct_fwd = False
      ct_rev = True
      ct_inh = False
    elif aecu_con & 0x01C == 0x010:
      ct_idle = False
      ct_fwd = False
      ct_rev = False
      ct_inh = True

    if aecu_con != aecu_con_old:       
      print("CT_AECU_con = %s " % hex(aecu_con))
      aecu_con_old = aecu_con

###### Generic DO set command callback (For test)
def callback_pi_do(data):               # Nibble 1 in PI_DO topic used
    global DO1, DO2, DO3, DO4, do_old   # Has to de declared as global to enable update
    DO1 = data.data & 0x0010            # Read data from topic       
    DO2 = data.data & 0x0020            # Read data from topic   
    DO3 = data.data & 0x0040            # Read data from topic   
    DO4 = data.data & 0x0080            # Read data from topic
    ########### Set Dicrete outouts ########
    GPIO.output(GPO1, DO1)                 # Set GPIO as commanded from CT
    GPIO.output(GPO2, DO2)                 # Set GPIO as commanded from CT
    GPIO.output(GPO3, DO3)                 # Set GPIO as commanded from CT
    GPIO.output(GPO4, DO4)                 # Set GPIO as commanded from CT

    if (data.data & 0x00F0) != do_old:
      do_old = data.data & 0x00F0
      print("aecu_do = %s " % hex(do_old))


################  Callbacks on RECU published data ############################$
def callback_recustat(data):            # Callback from RECU_status
    global recustat, recustat_old, robot_has_atlas       # Has to de declared as global to enable update
    recustat = data.data                # read data from topic

    if (recustat & 0x000F) == 3:
      robot_has_atlas = True
    else:
      robot_has_atlas = False

    if recustat != recustat_old:
      print("robot has atlas = %s " % robot_has_atlas)
      recustat_old = recustat



#########  Define the aecu node. Node name is aecu ##############################
def aecu():
    rospy.init_node('aecu', anonymous=True)    # Tells rospy the name of the node
    #anonymous = True ensures that node has a unique name by adding random numbers to the end of it.


    ##################  Subscriber topics ############################################
    #The node is subscribing to the topic CT_AECU_con of the type UInt16.
    #After receiving the function callback_ct_aecu_con is called.
    rospy.Subscriber("CT_AECU_con", UInt16, callback_ct_aecu_con)
    rospy.Subscriber("PI_DO", UInt16, callback_pi_do)
    rospy.Subscriber("RECU_status", UInt16, callback_recustat)
   # rospy.Subscriber("HECU_status", UInt16, callback_hecustat)


    #################### Publish topics  ############################################
    #Queue_size is the max length of the queue before message is dropped. (Short queue -> higher prio.)

    #Publish AECU status
    statuspub = rospy.Publisher('AECU_status', UInt16, queue_size=10)

    #Publish of HW discrete input data
    dipub = rospy.Publisher('PI3_DI', UInt16, queue_size=10)



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
          DI5=GPIO.input(GPI5          )                    # Read GPIO
          DI6=GPIO.input(GPI6)                    # Read GPIO
          DI7=GPIO.input(GPI7)                    # Read GPIO
          DI8=GPIO.input(GPI8)  

          #Read manual commands
          if DI1 and not(DI2):          
            hw_fwd = True
            hw_rev = False
            hw_idle = False
          elif DI2 and not(DI1):
            hw_fwd = False
            hw_rev = True
            hw_idle = False
          else:
            hw_fwd = False
            hw_rev = False
            hw_idle = True

          if DI3:                                 # Arm oor
            hw_aposhome = False
            hw_aposprehome = False
            hw_aposunclear = True
            hw_apos_other = False
          elif (DI4 and DI5):                     # Arm in home position
            hw_aposhome = True
            hw_aposprehome = False
            hw_aposunclear = False
            hw_apos_other = False
          elif DI4:                               # Arm in position to be lifted to home
            hw_aposhome = False
            hw_aposprehome = True
            hw_aposunclear = False
            hw_apos_other = False
          elif DI5:                               # Arm in unclear position
            hw_aposhome = False
            hw_aposprehome = False   
            hw_aposunclear = True
            hw_apos_other = False
          else:
            hw_aposhome = False
            hw_aposprehome = False
            hw_aposunclear = False
            hw_apos_other = True

          hw_torque_reached = DI6
          hw_sum_alarm = DI7

          # other inputs:
            # ct_idle
            # ct_fwd
            # ct_rev
            # ct_inh
	    # robot_has_atlas
            # 

          #Calculate tool status and set outputs
          if  ct_inh or hw_sum_alarm:
            run_status = 1
          elif hw_fwd and not(robot_has_atlas) and not(hw_torque_reached) and hw_apos_other:          
            run_status = 2                        # Manually forward 
          elif hw_rev and not(robot_has_atlas) and hw_apos_other:          
            run_status = 3                        # Manually reverse
          elif ct_fwd and (robot_has_atlas) and not(hw_torque_reached) and hw_apos_other:          
            run_status = 4                        # CT runs fwd
          elif ct_rev and (robot_has_atlas) and hw_apos_other:          
            run_status = 5                        # CT runs reverse
          else:          
            run_status= 1                        # Atlas is idle

          if hw_aposhome:
            pos_status = 1
          elif hw_aposprehome:
            pos_status = 5
          elif hw_aposunclear:
            pos_status = 6
          else: 
            pos_status = 4

          if hw_torque_reached:
            tq_status = 2
          else:
            tq_status = 1

          # Set outputs
          if (run_status == 2) or (run_status == 4):   # run fwd
            GPIO.output(GPO1, 1)                 
            GPIO.output(GPO2, 0)                 
          elif (run_status == 3) or (run_status == 5):   # run rev
            GPIO.output(GPO1, 1)
            GPIO.output(GPO2, 1)
          else:
            GPIO.output(GPO1, 0)
            GPIO.output(GPO2, 0)

          # Calculate status 
          aecu_status = run_status + 16*pos_status + 128*tq_status 

          # Publish it
          statuspub.publish(aecu_status)        # Publish recu status


############# Read and Publish all HW input status if time ########################   
        elif DIdifftime>pubrateDI:
          DIref=time.time()
          # Set DI status for pi1 as bits
          pi3_di = DI1+(2*DI2)+(4*DI3)+(8*DI4)+(16*DI5)+(32*DI6)+(64*DI7)+(128*DI8)  # Set DI sta$
          dipub.publish(pi3_di)                   # Publish DI status
 

 
       

if __name__ == '__main__':

# The try except below means if indicated exeption errror ocurres when calling function then
    # do not act on it
    try:
      aecu()
    except rospy.ROSInterruptException():
      pass
    


