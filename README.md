PiDeCAF
=======================================

This repository contains a package for ROS groovy that implements a decentralized collision avoidance framework for use on a Raspberry Pi.  The framework was developed as a part of the Auburn University ATTRACT program in the summer of 2013. Visit our website at https://sites.google.com/site/auburnuavreu2013team1/home for more details. The following documentation will be split up into two major components:

1. Hardware - Setting up the Pi, subsystems on the plane

2. Software - How our framework works, how to slot in another algorithm


Hardware
=======

Power
-----




Controls
----

There are three main components that control the plane: the ardupilot chip, the raspberry Pi and the xbee radio.  
The ardupilot is responsible for driving the servos and the propellor of the plane.  It can take in waypoints as input
and it will direct the servos and motor to navigate to that waypoint.  The ardupilot will also output GPS information to the
raspberry Pi. 
The rapsberry Pi will run the collision avoidance algorithm by receiving telemetry updates from other planes and sending 
avoidance waypoints to the ardupilot. The raspberry Pi will also forward telemetry updates from the plane it is mounted on
to the xbee chip. The xbee chip which is responsible for sending and recieving telemetry information.





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
