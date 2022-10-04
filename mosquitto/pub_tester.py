from umqtt.simple import MQTTClient
from machine import Pin
from time import sleep

BROKER_ADDRESS = '192.168.0.195'
CLIENT_NAME = 'BLUE'

mqttc = MQTTClient(CLIENT_NAME, BROKER_ADDRESS, keepalive=60)
mqttc.connect()

btn = Pin(0)
BTN_TOPIC = CLIENT_NAME.encode() + b'/btn/0'

while True:
    mqttt.publsh(BTN_TOPIC, str(btn.value()).encode() )
    sleep(0.5)
    
#('192.168.0.40', '255.255.255.0', '192.168.0.195', '192.168.0.195')
 
 
