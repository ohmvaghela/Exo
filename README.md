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

# Wifi connection using ESP32

    cd /home/ohm/esp-idf/wifi/getting_started/station
    idf.py set-target esp32
    idf.py menuconfig  (optional if you want to change wifi name and password)
    idf.py build
    idf.py -p /dev/ttyUSB0 flash
    idf.py -p /dev/ttyUSB0 monitor
    # once flashed just providing power source is enough




for multiple esp use espnow in example/wifi
solve heating issue