#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Unification ROSBridge-Kafka multiclient: unidriver_bridge

    # The idea is to publish messages only to topics bridge_to_sp(kafka) and bridge_to_driver(ros) 
    # and receive only from topics sp_to_bridge(kafka) and driver_to_bridge(ros)

    # V.0.1.0.
#----------------------------------------------------------------------------------------


#----------------------------------------------------------------------------------------
# Libraries. You dont see rospy or roslib here because the idea is to keep this 
# multiclient independent of ROS. Instead, the multiclient is "subscribed" to
# ROS topics by receiving data from those topics via socket
#----------------------------------------------------------------------------------------
import time
import json
import threading
import socket
from std_msgs.msg import String
from kafka import KafkaProducer
from kafka import KafkaConsumer
from collections import OrderedDict


#----------------------------------------------------------------------------------------
# Keeping it simple   
#----------------------------------------------------------------------------------------
class unidriver_bridge():

    def __init__(self, sock=None):

        if sock == None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

        self.spin()


    #----------------------------------------------------------------------------------------
    # Initialization in this case means advertising the topic on which stuff
    # will be published. Also, we have to subscribe the multiclient to a certain topic 
    # in order to be able to receive data via socket from that topic
    #----------------------------------------------------------------------------------------
    def initialize_rosbridge_server(self):
        def initialize_rosbridge_server_callback():
            producer = KafkaProducer(bootstrap_servers='localhost:9092', value_serializer=lambda m: json.dumps(m).encode('ascii'))
            producer.send('sp_to_bridge', {"op": "advertise", "topic": "/bridge_to_driver", "type": "std_msgs/String"})
            time.sleep(1)
            producer.send('sp_to_bridge', {"op": "subscribe", "topic": "/driver_to_bridge", "type": "std_msgs/String"})
            time.sleep(1)
        t = threading.Thread(target=initialize_rosbridge_server_callback)
        t.daemon = True
        t.start()


    #----------------------------------------------------------------------------------------
    # This method is subscribing the multiclient to the "sp_to_bridge" kafka topic
    # and it is for receiving messages from the Kafka side that are meant for one-way
    # multiclient to ROS communication, so publish, advretise... 
    #----------------------------------------------------------------------------------------
    def consumer_to_ros(self):
        def consumer_to_ros_callback():
            consumer = KafkaConsumer('sp_to_bridge',
                                      bootstrap_servers='localhost:9092',
                                      value_deserializer=lambda m: json.loads(m.decode('utf-8'), object_pairs_hook=OrderedDict),
                                      auto_offset_reset='latest',
                                      consumer_timeout_ms=1000)

            HOST = "localhost"
            PORT = 9090 
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))

            while (1):
                for message in consumer:
                    if message.value['op'] != "subscribe":
                        s.send(json.dumps(message.value))
                    else:
                        pass
                
        t = threading.Thread(target=consumer_to_ros_callback)
        t.daemon = True
        t.start()


    #----------------------------------------------------------------------------------------
    # Initially, this method was made for the possibility that the multiclient would like
    # to "subscribe" to several ROS topics, in that case it would launch an instance
    # of a socket receiver and kafka producer for each topic subscribed, but that was 
    # dropped when it was decided that the communication is achieved with two one-way topics
    #----------------------------------------------------------------------------------------
    def producer_from_ros(self):
        def producer_from_ros_callback():
            consumer = KafkaConsumer('sp_to_bridge',
                                      bootstrap_servers='localhost:9092',
                                      value_deserializer=lambda m: json.loads(m.decode('utf-8'), object_pairs_hook=OrderedDict),
                                      auto_offset_reset='latest',
                                      consumer_timeout_ms=1000)

            while (1):
                for message in consumer:
                    if message.value['op'] == "subscribe" or message.value['op'] == "unsubscribe":
                        self.connect('localhost', 9090)
                        time.sleep(0.3)
                        self.send(message.value)
                        time.sleep(0.3)
                        self.receive()
                    

        t = threading.Thread(target=producer_from_ros_callback)
        t.daemon = True
        t.start()


    #----------------------------------------------------------------------------------------
    # Establishing a connection with the rosbridge_tcp server
    #----------------------------------------------------------------------------------------
    def connect(self, host, port):
        def connect_callback():
            TCP_IP = 'localhost'
            TCP_PORT = 9090
            self.sock.connect((TCP_IP, TCP_PORT))
        t = threading.Thread(target=connect_callback)
        t.daemon = True
        t.start()
   

    #----------------------------------------------------------------------------------------
    # "Publishing" jsonized messages
    #----------------------------------------------------------------------------------------
    def send(self, msg):
        def send_callback():
            self.sock.send(json.dumps(msg))
        t = threading.Thread(target=send_callback)
        t.daemon = True
        t.start()


    #----------------------------------------------------------------------------------------
    # "Subscribing" to ROS messages and producing them to a kafka topic
    #----------------------------------------------------------------------------------------
    def receive(self):
        def receive_callback():
            producer = KafkaProducer(bootstrap_servers='localhost:9092')
            while (1):
                data = self.sock.recv(4096)
                if not data: break
                print data
                producer.send('bridge_to_sp', data)
            self.sock.close()
        t = threading.Thread(target=receive_callback)
        t.daemon = True
        t.start()


    #----------------------------------------------------------------------------------------
    # Calls the methods in the right order and makes sure the multiclient doesn't die
    #----------------------------------------------------------------------------------------
    def spin(self):
        self.consumer_to_ros()
        self.producer_from_ros()
        time.sleep(1)
        self.initialize_rosbridge_server()
        while (1):
            pass
 
if __name__ == '__main__':
    try:
        unidriver_bridge()
    except KeyboardInterrupt:
        pass
