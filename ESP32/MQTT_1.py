       
import network
import time
from umqtt.simple import MQTTClient
from machine import Pin

WiFi_SSID = "ESP32COMQ"  # Enter Wifi SSID
WiFi_PASS = "ohm123ohm"  # Enter Wifi Pasword

#SERVER = "mqtt-v2.thingspeak.com" # Enter Mqtt Broker Name

SERVER = "mqtt3.thingspeak.com" # Enter Mqtt Broker Name

PORT = 1883
CHANNEL_ID = "1875815" #Enter Channel Id here
USER = "AjoFBykJLigwIBsQCAERLDA"  # Enter User Id here
CLIENT_ID = "AjoFBykJLigwIBsQCAERLDA" #Enter Client Id here
PASSWORD = "v3Vsm28AU4MY4OU+WsFlsLhp" #Enter Password here


counter = 0  # counter value initialised
led = 0

#create topic to publish the message
topicOut = "channels/" + CHANNEL_ID + "/publish" 

#create topic to publish the message
#topicIn = "channels/" + CHANNEL_ID  + "/subscribe/fields/field2"

led = Pin(2, Pin.OUT)  #Led pin is initialise as Output
led.value(0)    # Led pin is set low

# Function to implement wifi connection
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(WiFi_SSID, WiFi_PASS)
        while not wlan.isconnected():
            pass
    print("Connected to Wifi Router")
    
   

#Call back function called when new message from the broker arrives

def calback(topic,msg):
    print("topic : " + topic)
    print("msg : " + msg)
    
    if msg == b'1':
        led.value(1)
    else:
        led.value(0)
    
    

#connect esp32 to wifi
wifi_connect()

#create a client and connect to the mqtt broker

#client = MQTTClient(CLIENT_ID, SERVER,PORT,USER,PASSWORD)

client = MQTTClient(CLIENT_ID, SERVER,PORT,USER,PASSWORD,60) 

client.set_callback(calback)

client.connect()

#client.subscribe(topicIn)

print("Mqtt connected")
    
while True:
    
    client.check_msg()
    
    if counter>100:
        counter = 0
        
    #Publish the topic meaasge to the broker
    client.publish(topicOut, "field1="+str(counter))
    client.subcribe(topicIn,"")
    print(counter)
    counter = counter + 10
    
    time.sleep(2)
    
 