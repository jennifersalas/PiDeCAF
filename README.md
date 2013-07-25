PiDeCAF
=======================================

This repository contains a package for ROS groovy that implements a decentralized collision avoidance framework for use on a Raspberry Pi.  The framework was developed as a part of the Auburn University ATTRACT program in the summer of 2013. Visit our website at https://sites.google.com/site/auburnuavreu2013team1/home for more details. The following documentation will be split up into two major components:

1. Hardware - Setting up the Pi, subsystems on the plane

2. Software - How our framework works, how to slot in another algorithm


Hardware
=======




Software
=======


Our decentralized collision avoidance framework makes use of three
nodes: the ardu, xbee and mover nodes. The ardu and xbee
nodes are responsible for handling communications between
the ArduPilot and XBee devices respectively, while the mover
node is responsible for forwarding waypoints to the ArduPilot
and running the collision avoidance algorithm.



![My image](https://0d9aa83c-a-62cb3a1a-s-sites.googlegroups.com/site/auburnuavreu2013team1/PiDeCAF.png?attachauth=ANoY7cplKmbZcYVVpBwpfJMWCUCy5ydfAghsI4wjWjYU3Uwm4khZ8q9-inrlYKmuPjDGuM5nFaXXuXm8w0ZLKmSXIH6D9YPEoyJII-g9FNV6azWnJHuqObGu4BsK7wwUFM3705k8lTw32SL1uQdXl07nLG9sRNr5l37N5BtTUljClmrxa3em3EdNXNFKoIOjQIpn47CUSJJRgi6V1iqYWz8e0sOQysAcxQ%3D%3D&attredirects=0)


###Ardu node
The ardu node is responsible for handling communication
between the Raspberry Pi and the ArduPilot system.
This entails receiving GPS information from the ArduPilot
and forwarding waypoint commands to the ArduPilot. The
ardu node accomplishes this by simultaneously listening to the
ArduPilot’s serial line and waiting for a message to be posted
to ca commands. When the ardu node receives a telemetry
update from the ArduPilot, it will publish this information to both the all telemetry topic
(which is used by the mover node)
and the my mav telemetry topic (which is used to send this
telemetry to other UAVs).
While the ardu node is listening for a telemetry update
from the ArduPilot, it will also be waiting for a command
to be published on the ca commands topic. If a command is
published on this topic, the ardu node will take the posted
information, pack it into a mavlink message and then send
it to the ArduPilot system using serial communication. The
ArduPilot will then set the new destination of the UAV to
reflect the command.

###Xbee node
The xbee node is responsible for handling communication
between the Raspberry Pi and the XBee module.
The XBee module communicates with other XBee modules
by either receiving messages from a ground station and other
UAVs or by sending out telemetry information to other systems.
Much like the ardu node, the xbee node simultaneously
listens to the XBee module for a message and writes to the
XBee once a message is posted to the my mav telemetry
topic.
The xbee node can currently read and decode two types of
mavlink messages: a telemetry update and a command. If a
telemetry update is read from the XBee module, the message is
decoded and posted to the all telemetry topic (which is used
by the mover node). If a command is read from the XBee
module, it is decoded and posted to the gcs commands topic
(which is also used by the mover node).
In addition to reading from the XBee module, the xbee node
also waits for a message to be posted to the my mav telem
topic. Once a message is posted to this topic, the xbee node
will take the information, pack it to a mavlink message and
perform a serial write to the XBee so that the telemetry
information is sent out by the XBee module.

###Mover node
The mover node is responsible for generating
waypoints to send to the ArduPilot. This node operates
by taking in telemetry messages and ground station control
commands, then periodically forwarding waypoints to
ca commands so that they can be sent to the ArduPilot. Input
to this node is taken from either the gcs commands topic or the
all telemetry topic. Messages from the gcs commands topic
specify a goal waypoint that the UAV should eventually aim
to reach while running collision avoidance. Messages from the
all telemetry topic, on the other hand, contain GPS updates
from all UAVs (including the plane running this framework).
Depending on the state that the mover node is in (see section
IV-D1), the forwarded waypoint will be either a collision
avoidance waypoint or the goal waypoint. The waypoint is
forwarded to the ardu node through the ca commands topic
at a preset rate so that the ArduPilot is not flooded with
commands.
















Terminal Interfacing
--------------------

###Helpful commands

These commands can be run on a laptop with gcs_router running to issue specific commands to a Raspberry Pi running
the PiDeCAF framework. Replace "255" with the planeID of the plane you would like to receive the command

CA_ON
```
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 555 1 1
```

CA_OFF
```
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 444 1 1
```
STOP

```
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 777 1 1
```


###Faking telemetry

The gcs_router node supports sending fake telemetry updates to other planes.  The following examples demonstrate how:

Publishing goal waypoint, X = desired latitude, Y = desired longitude, Z = desired altitude
```
rostopic pub /my_mav_telemetry au_uav_ros/Telemetry '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 X Y Z 200 200 200 3 3 4 5
```
Enter a "--" only once before your negative numbers to enter negative numbers as arguments
```
rostopic pub /my_mav_telemetry au_uav_ros/Telemetry '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 32.602597 -- -85.488859 100 200 200 200 3 3 4 5
```
