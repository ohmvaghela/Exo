<!-- Esp8266 -->
<!-- https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html -->

<!-- Esp32 -->
<!-- https://docs.micropython.org/en/latest/esp32/quickref.html -->

<!-- flash drive -->
esptool.py --port /dev/ttyUSB0 erase_flash

<!-- deploy new firmware -->
<!-- esp8266 -->    esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash --flash_size=detect 0 esp8266-20220618-v1.19.1.bin     
<!-- esp32 -->      esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 esp32-20220618-v1.19.1.bin

<!-- communication with ESP8266 using picocom -->
picocom /dev/ttyUSB0 -b115200

<!-- connecting to wifi -->
<!-- https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html -->

