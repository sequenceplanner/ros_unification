#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Simple Unification dummy Kafka Producer-Consumer mutlithreaded example: unif_dummy_kafka_pro_con.py
    # V.0.2.0.
#----------------------------------------------------------------------------------------

import time
import json
import threading
from kafka import KafkaProducer
from kafka import KafkaConsumer

class uni_control_sim():

    def __init__(self, sock=None):
        
        self.A = True
        self.B = True
        time.sleep(2)
        self.main()

    def producer1(self):
        def producer1_callback():
            producer1 = KafkaProducer(bootstrap_servers='localhost:9092', value_serializer=lambda m: json.dumps(m).encode('ascii'))
            producer1.send('to_ros', {"op": "advertise", "topic": "/from_kafka", "type": "std_msgs/String"})
            time.sleep(1)
            while (self.A):
                producer1.send('sp_to_bridge', {"op": "publish", "topic": "/bridge_to_driver", "msg": {"data" : '{"receiver" : "recu_unidriver", "command" : "connect_to_tool"}'}})
                time.sleep(8)
                producer1.send('sp_to_bridge', {"op": "publish", "topic": "/bridge_to_driver", "msg": {"data" : '{"receiver" : "ur_pose_unidriver", "refPos" : "URPickPos"}'}})
                time.sleep(8)
                self.A = False
        t = threading.Thread(target=producer1_callback)
        t.daemon = True
        t.start()

    def producer3(self):
        def producer3_callback():
            producer3 = KafkaProducer(bootstrap_servers='localhost:9092', value_serializer=lambda m: json.dumps(m).encode('ascii'))
            producer3.send('to_ros', {"op": "subscribe", "topic": "/ros_to_sp", "type": "std_msgs/String"})
            time.sleep(1)
        t = threading.Thread(target=producer3_callback)
        t.daemon = True
        t.start()

    def consumer1(self):
        def consumer1_callback():
            consumer1 = KafkaConsumer('from_ros',
                                      bootstrap_servers='localhost:9092',
                                      value_deserializer=lambda m: json.loads(m.decode('ascii')),
                                      auto_offset_reset='latest',
                                      consumer_timeout_ms=1000)

            while (1):
                for message in consumer1:
                    print (message.value)
                    pass

        t = threading.Thread(target=consumer1_callback)
        t.daemon = True
        t.start()

    def main(self):
        self.producer1()
        #self.producer3()
        #self.consumer1()
        time.sleep(2)
        
        while(1):
            pass
        
if __name__ == '__main__':
    try:
        uni_control_sim()
    except KeyboardInterrupt:
        pass