# Esp8266
<!-- https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html -->

# Esp32
<!-- https://docs.micropython.org/en/latest/esp32/quickref.html -->

# flash drive
    esptool.py --port /dev/ttyUSB0 erase_flash

# deploy new firmware
    cd /home/ohm/Documents/GitHub/Exo
`esp8266 -->` 

    esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash --flash_size=detect 0 ESP_config/esp8266-20220618-v1.19.1.bin     
`esp32 -->`

    esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 ESP_config/esp32-20220618-v1.19.1.bin

`communication with ESP8266 using picocom`

    picocom /dev/ttyUSB0 -b115200

`connecting to wifi`

    https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html

`mosquitto`

    https://www.vultr.com/docs/install-mosquitto-mqtt-broker-on-ubuntu-20-04-server/

`To start mosquitto server with custom config file (Wifi username and password configured) `
    
    mosquitto -c /etc/mosquitto/mosquitto.conf

`For ESP-Laptop interface download ESP-IDF`

    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation

# setup esp32
    get_idf
`path to project `

    cd ~/esp/hello_world
    idf.py set-target esp32

`to change default configure settings`

    idf.py menuconfig  
    idf.py build
    idf.py -p /dev/ttyUSB0 flash

`to moniter`

    idf.py -p /dev/ttyUSB0 monitor

`to add external components`

First make a folder named `components` and add components to it 

CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY

Then use this command for all newly added components in project folder

    idf.py create-manifest --component=my_component1
    idf.py create-manifest --component=my_component2
    idf.py create-manifest --component=my_component3
    .
    .
    .
    idf.py reconfigure

# Wifi connection using ESP32

    cd /home/ohm/esp-idf/wifi/getting_started/station
    idf.py set-target esp32
    idf.py menuconfig  (optional if you want to change wifi name and password)
    idf.py build
    idf.py -p /dev/ttyUSB0 flash
    idf.py -p /dev/ttyUSB0 monitor
    # once flashed just providing power source is enough

#
`Station means will connect to wifi and softAP means work as Access Point or say will work as wifi `
#
# Position control 2 code
    - Data recived types
        - command_payload
            - goal_position;
            - goal_velocity;
            - effort;
            - CONTROL_MODE mode;
                - STOP = 0,
                - POSITION = 1,
                - VELCOCITY = 2,
                - EFFORT = 3,
            - STOP_INTURRPT emergency_command;
                - NO_INTERRUPT = 0,
                - MANUAL_STOP = 1,
                - MASTER_STOP = 2,
                - LOCAL_STOP = 3,
        - pid (p,i,d)

`MQTT default broker` mqtt://mqtt.eclipseprojects.io 

`about MQTT esp-idf tcp code`

    - basic yet to be read
    - MQTT_EVENT_CONNECTED,
    - MQTT_EVENT_DISCONNECTED
    - MQTT_EVENT_SUBSCRIBED
    - MQTT_EVENT_UNSUBSCRIBED
    - MQTT_EVENT_DATA
    - MQTT_EVENT_ERROR



`Multi esp communication`

    - see code -> example/wifi/espnow
    - MQTT broker -> example/protocal/mqtt/tcp

for multiple esp use espnow in example/wifi
solve heating issue

160W
magnetic codrej 














