#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # MiR REST Unification Driver
    # Requests state of the MiR via REST to get the x, y and yaw of the MiR
    # Transforms them to UR10 base_frame coordinates and publishes on a topic
    # The scene updater is subscribed to this topic and updates the MiR's position in the scene
    # Also contains methods to give commands to MiR like move_to or similar... 
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import time
import random
import rospy
import roslib
import json
import threading
import requests
from std_msgs.msg import String

class mir_rest_unidriver():

    def __init__(self):

        rospy.init_node('mir_rest_unidriver', anonymous=False)
        self.main_pub = rospy.Publisher('mir_unidriver_state', String, queue_size=10)
        #rospy.Subscriber("from_kafka", String, self.from_kafka_callback)
        
        self.main_rate = rospy.Rate(10)

        #--------------------------------------------------------------------------------------
        # For thesting without Kafka
        #--------------------------------------------------------------------------------------
        #self.publish_on_dummy_topic_2()
        #self.subscribe_to_dummy_topic_2()

        self.main()


    #----------------------------------------------------------------------------------------
    # Main method
    #----------------------------------------------------------------------------------------
    def main(self):
        while not rospy.is_shutdown():
            try:
                self.resp = requests.get('http://192.168.1.140:8080/v1.0.0/status')
                self.state = self.resp.json()
                self.position = self.state['position']
                self.internal_pos_x = self.position['x']
                self.internal_pos_y = self.position['y']
                self.internal_pos_yaw = self.position['orientation']
                self.world_pos_x = self.internal_pos_x
                self.world_pos_y = self.internal_pos_y - 1
                self.mir_state = str({"x" : self.internal_pos_x, "y" : self.internal_pos_y})
                self.main_pub.publish(json.dumps(self.mir_state))
                time.sleep(0.1)
            except rospy.ROSInterruptException:
                pass
    
        rospy.spin()


    #----------------------------------------------------------------------------------------
    # Main callback for external communication
    #----------------------------------------------------------------------------------------
    def from_kafka_callback(self, from_kafka_data):
        self.from_kafka = json.loads(from_kafka_data.data)
        print self.from_kafka
        if self.from_kafka['receiver'] == "mir_rest_unidriver":
            if self.from_kafka['command'] == "mir_move_to_ur10":
                self.mir_move_to_ur10()
            elif self.from_kafka['command'] == "mir_move_home":
                self.mir_move_home()
            else:
                pass
        else:
            pass
    

    #----------------------------------------------------------------------------------------
    # Callbacks for in-ROS communication
    #----------------------------------------------------------------------------------------
    def dummy_topic_2_callback(self, dummy_topic_2_data):
        self.dummy_state_2 = dummy_topic_2_data.data


if __name__ == '__main__':
    try:
        mir_rest_unidriver()
    except rospy.ROSInterruptException:
        pass