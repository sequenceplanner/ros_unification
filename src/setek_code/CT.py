#! /usr/bin/env python


######################################## Node that emulates Control tower ##########################

# Make sure that python is used at the interpretor.
# Note! #! is not a comment and the line must ve the first in the file


#Imports
import rospy
import roslib                       #Base dependency of all ROS Client Libraries and tools.
import threading                    #Python std module. Enables working with threads
from std_msgs.msg import Int16      #Use the std_msgs/Int16 message
from std_msgs.msg import UInt16      #Use the std_msgs/Int16 message

#Global variables
pi1di = pi1di_old = 0
pi3di = pi3di_old = 0
pi6di = pi6di_old = 0

recustat = recustat_old = 0
aecustat = aecustat_old = 0
hecustat = hecustat_old = 0


################  Callbacks on RECU published data #################################
def callback_recustat(data):            # Callback from RECU_status
    global recustat, recustat_old       # Has to de declared as global to enable update
    recustat = data.data                # read data from topic
    if recustat != recustat_old:
      print("RECU status = %s" % bin(0x8000+recustat))
      print("Robot tool connection status = %s" % (recustat & 0x000F))
      print("LF grip status = %s" % ((recustat & 0x0070)>>4))
      print("pressure status = %s" % ((recustat & 0x0180)>>7))
      print(" ")
      recustat_old = recustat

def callback_pi1di(data):               # Callback from PI1_DI
    global pi1di,pi1di_old              # Has to de declared as global to enable update
    pi1di = data.data                   # read data from topic
    if (pi1di != pi1di_old):
      print("PI1 DI = %s" % bin(0x8000+pi1di))
      print(" ")
      pi1di_old = pi1di


###################  Callbacks on AECU published data #############################
def callback_aecustat(data):            # Callback from AECU_status
    global aecustat, aecustat_old       # Has to de declared as global to enable update
    aecustat = data.data                # read data from topic
    if aecustat != aecustat_old:
      print("AECU status = %s" % bin(aecustat))
      print("Atlas run status = %s" % (aecustat & 0x000F))
      print("Atlas arm position = %s" % ((aecustat & 0x0070)>>4))
      if ((aecustat & 0x0310)>>7) == 2:
        print("Atlas torque reached")
      else:
        print("Atlas torque not reached")
      print(" ")
      aecustat_old = aecustat

def callback_pi3di(data):               # Callback from PI3_DI
    global pi3di,pi3di_old              # Has to de declared as global to enable update
    pi3di = data.data                   # read data from topic
    if (pi3di != pi3di_old):
      print("PI3 DI = %s" % bin(0x8000+pi3di))
      print(" ")
      pi3di_old = pi3di


################  Callbacks on HECU published data #################################
def callback_hecustat(data):            # Callback from FECU_status
    global hecustat, hecustat_old       # Has to de declared as global to enable update
    hecustat = data.data                # read data from topic
    if hecustat != hecustat_old:
      print("HECU status = %s" % bin(hecustat))
      print(" ")
      hecustat_old = hecustat

def callback_pi6di(data):               # Callback from PI4_DI
    global pi6di,pi6di_old              # Has to de declared as global to enable update
    pi6di = data.data                   # read data from topic
    if (pi6di != pi6di_old):
      print("PI6 DI = %s" % hex(pi6di))
      print(" ")
      pi6di_old = pi6di


# Define the ct emulation node. Node name is ct_emul
def ct_emul():

    global recustat_old, pi1_old, aecustat_old, pi3_old, hecustat_old, pi6_old       
    # Has to de declared as global to enable update


    # Init the node name so that it can communicate with the ROS master.
    # anonymous=True guarantees that the name stalker is unique by automatically adding a no to the end of the name.
    rospy.init_node('ct_emul', anonymous=True)


    ################################## Published topics ###################################### 
    recuconpub = rospy.Publisher('CT_RECU_con', UInt16, queue_size=10)
    aecuconpub = rospy.Publisher('CT_AECU_con', UInt16, queue_size=10)
    pidopub = rospy.Publisher('PI_DO', UInt16, queue_size=10)
   
    # Set publishing rate to 1 Hz
    rate = rospy.Rate(1)


    ################################## Subscribed topics ###################################### 
    rospy.Subscriber("RECU_status", UInt16, callback_recustat)
    rospy.Subscriber("AECU_status", UInt16, callback_aecustat)
    rospy.Subscriber("HECU_status", UInt16, callback_hecustat)

    #The node is subscribing to the status of the discrete inputs of the diffrent units
    rospy.Subscriber("PI1_DI", UInt16, callback_pi1di)
    rospy.Subscriber("PI3_DI", UInt16, callback_pi3di)
    rospy.Subscriber("PI6_DI", UInt16, callback_pi6di)



    ############################# Initialisations ################################################
    print(" ")
    docon = 0
    recucon = 0
    aecucon = 0
    atlasrun = 0


    ############################## Loop ################################################# 
    while not rospy.is_shutdown():

        ################ CT emulation functions here
        nimp = 0
	kinp = raw_input()			# Wait for something to be written
        try:
          nimp = int(kinp)
        except ValueError:
          print("Not a number. 1:Set DO, 2: Set RECU con, 3: Set AECU con")

        if nimp == 1:
          # Set DO on all PI:es if input was 1
          # Nibble 0 for RECU, nibble 1 for RECU, nibble 2 for AECU, nibble 3 for HECU 
          nb = raw_input('Set DO: ')
          try:
            docon = int(nb)        # Has to be read as int. HEX not permitted!!???
          except ValueError:
            print("Not a valid DO")
          # Publish do topic 
          pidopub.publish(docon)

        elif nimp == 2:
          # Set RECU con if input was 2 
          nb = raw_input('Set RECU con: ')
          try:
            recucon = int(nb)
          except ValueError:
            print("Not a valid RECU_con")
          # Publish recucon topic 
          recuconpub.publish(recucon)

        elif nimp == 3:
          # Set Atlas if input was 3 
          nb = raw_input('Set Atlas run: ')
          try:
            atlasrun = int(nb)
          except ValueError:
            print("Not a valid Atlas command") 
          if atlasrun == 4:
            aecucon =0x0010
          elif atlasrun == 1:
            aecucon =0x0004
          elif atlasrun == 2:
            aecucon =0x0008
          elif atlasrun == 3:
            aecucon =0x000C
          else:
            aecucon =0x0004
        # Publish ecucon topic
          aecuconpub.publish(aecucon)

        elif nimp == 10:
          print ("Tool status:")
          # Print status from all tools once 
          recustat_old = 0xFF
          pi1di_old = 0xFF
          aecustat_old = 0xFF
          pi3do_old = 0xFF
          hecustat_old = 0xFF
          pi6do_old = 0xFF
          # Nothing is published         

        else:
          print("1:set do, 2:recu_con, 3:atlas con, 10:get status")


        # Sleep until next publishing time. Subscribing is working also during sleep
        rate.sleep()

if __name__ == '__main__':

    # The try except below means if indicated exeption errror ocurres when calling function then
    # do not act on it
    try:
        ct_emul()   
    except rospy.ROSInterruptException:
        pass
