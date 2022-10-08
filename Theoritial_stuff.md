`Access point`
- device that allows wireless device to connect to wired network
- in real life one Wifi routers are installed in big offices and many WAPs(Wireless Access Points) are connected to the main wifi router
- In this project WAP is my mobile and devices connected to mobile are ESPs

`MAC (Media Access Control)`
- Physical address of a device, embedded in chip by manufacturer
- Each ESP and mobile in our case has a unique physical address

`Web Sockets`
- Bi-directional commincation protcal
- Same as HTTP/HTTPs but these are unidirectional

`WebREPL`
- REPL - Read Evaluate Print Loop
- -> Read from prompt -> evaluate it -> print result -> Read from prompt ->  

`MQTT`
- Broker may be device like laptop, mobile, Rpy, or cloud brokers
- for my case my laptop will be broker
- ESP will publish sensor data and Subscribe motor data 
- There 3 things in broker to subscriber conncetion
    * Birth message - when broker and pub connection is established 
    * Death Message - when broker and pub connection is abloished on purpose
    * Last Will Testiment Message - when broker and pub connection is abloished due to error 


