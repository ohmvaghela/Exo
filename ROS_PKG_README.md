# ROS Package

## Raspberry pi

* takes input - process it - sent it to MQTT broker 
> edited line 58 exoskeletion_ui/src/exoskeletion_ui
> change the localhost to 192. ... in mqtt_endpoint_touch_screen.py

## for running 

    run mosquitto
    rosrun exoskeleton_ui exoskeleton_ui
    rosrun exoskeleton_ui mqtt_endpoint_touchscreen.py 

