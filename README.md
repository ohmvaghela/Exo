# Esp8266
    https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html 

# Esp32
    https://docs.micropython.org/en/latest/esp32/quickref.html 

# erase flash drive
    esptool.py --port /dev/ttyUSB0 erase_flash

#
> Always make /dev/ttyUSB0 as Access point and use other ports for exo 
#

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

sudo cp -n /home/ohm/.espressif/tools/openocd-esp32/v0.11.0-esp32-20220411/openocd-esp32/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d

# while pip installing gdbgui following packages version were changed

`requirement.txt path`

    /home/ohm/esp/esp-idf/requirements.txt

``

    Installing collected packages: pygdbmi, brotli, werkzeug, six, python-engineio, pygments, markupsafe, itsdangerous, greenlet, dnspython, click, bidict, python-socketio, jinja2, eventlet, flask, flask-socketio, flask-compress, gdbgui
    Attempting uninstall: python-engineio
        Found existing installation: python-engineio 4.3.4
        Uninstalling python-engineio-4.3.4:
        Successfully uninstalled python-engineio-4.3.4
    Attempting uninstall: greenlet
        Found existing installation: greenlet 1.1.3
        Uninstalling greenlet-1.1.3:
        Successfully uninstalled greenlet-1.1.3
    Attempting uninstall: bidict
        Found existing installation: bidict 0.22.0
        Uninstalling bidict-0.22.0:
        Successfully uninstalled bidict-0.22.0
    Attempting uninstall: python-socketio
        Found existing installation: python-socketio 5.7.1
        Uninstalling python-socketio-5.7.1:
        Successfully uninstalled python-socketio-5.7.1
    ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
    launchpadlib 1.10.13 requires testresources, which is not installed.
    Successfully installed bidict-0.21.2 brotli-1.0.9 click-8.0.1 dnspython-2.2.1 eventlet-0.33.0 flask-2.0.1 flask-compress-1.10.1 flask-socketio-5.1.1 gdbgui-0.15.1.0 greenlet-1.1.2 itsdangerous-2.0.1 jinja2-3.0.1 markupsafe-2.0.1 pygdbmi-0.10.0.1 pygments-2.10.0 python-engineio-4.2.1 python-socketio-5.4.0 six-1.16.0 werkzeug-2.0.1



160W
magnetic codrej 














