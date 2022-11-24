import paho.mqtt.client as paho
import sys
import numpy as np
import struct

# @datatype
# class points:
#     a1: float


feedback_strut = struct.Struct('<8f')

def onMessage(client, userdata, msg):
    # print(msg.topic+": "+ msg.payload.decode('utf-8'))
    # print(msg.payload)
    # print(np.frombuffer(msg.payload,dtype=np.float32,count=1))
    
    unpacked_data = feedback_strut.unpack(msg.payload)
    print(unpacked_data)
    # print(np.array(msg.payload, dtype = np.float32))

client = paho.Client()
client.on_message = onMessage

if client.connect("192.168.4.2", 1883, 60) !=0:
    print("couldnt connect to the brker, exititng")
    sys.exit(-1)

client.subscribe("/RIGHT_FLEXION/feedback_data")
client.loop_forever()
try:
    print("press ctrl+c to stop the execution")
    client.loop_forever()
except:
    print("Disconnected from the server")

client.disconnect()


