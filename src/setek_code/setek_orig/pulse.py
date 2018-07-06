#! /usr/bin/env python
# Make sure that python is used as the interpretor.
# Note! #! is not a comment and the line must ve the first in the file

import rospy                            # Import the rospy module with a lot of ros functions in python
import roslib
from std_msgs.msg import String         # Reuse the std_msgs/String message type (a simple string container) for publishing.
import RPi.GPIO as GPIO                 # Import Raspberry pi GPIO module and call it GPIO in this module
import time
GPIO.setmode(GPIO.BCM)                  # The RP connector designations are used in setup
GPIO.setup(24, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)

# declare that the node is publishing to the chatter topic using the message type String.
# String here is actually the class std_msgs.msg.String.
# The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough. 
#pub = rospy.Publisher('chatter', String, queue_size=10)

def main():
#rospy.init_node('LEDblink', anonymous=True)    # Tells rospy the name of the node
# anonymous = True ensures that node has a unique name by adding random numbers to the end of NAME. 
#rate = rospy.Rate(10) # 10hz object rate
  GPIO.output(24, False)                  # Set GPIO to output low
  GPIO.output(23, False)                  # Set GPIO to output low
  pulse = False
  RPM = 500
  pulses_per_rpm = 200
  toggletime = 60/(float(RPM) * pulses_per_rpm)
  print(toggletime)
  pulsestart = time.time()                 # set reference time
  while not rospy.is_shutdown():
    difftime = time.time()-pulsestart
    #print(difftime)
    if difftime>toggletime and pulse==False:
      GPIO.output(23, True)                 # Set GPIO to output high
      pulsestart = time.time()                 # set reference time
      pulse = True
      #print("true")
    elif difftime>toggletime and pulse==True:
      GPIO.output(23, False)                 # Set GPIO to output high
      pulsestart = time.time()                 # set reference time
      pulse = False
      #print("false")
      
#   ub.publish(ledstate)                  # publishes a string to the chatter topic
#    time.sleep(1)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException():
    pass
    


