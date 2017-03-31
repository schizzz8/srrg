# srrg_orazio_core
The core package with firmware and installation instructions for
the Orazio Robot (aka MARRtino)

## Detailed Description

Orazio is a simple and cheap differential drive robot based on Arduino.
It performs closed loop control of the wheels, odometry integration
at the mobile base level.

The PC can control the robot through a serial connection.
All parameters can be configured and are stored in the EEPROM of the
arduino.

This package contains

* Arduino firmware
* Calibration Utilities

## Prerequisites

### Software

For this package you need
* Ubuntu 16.04 or Ubuntu 14.04
* arduino-mk 
* libncurses5-dev
* libwebsockets-dev
ROS is not required

### HARDWARE

* Arduino MEga 2560
* Arduino motor shield OR any 5V controllable H Bridge
* two motors with encoders
* a 12V battery
* a frame to fix all these devices
* a laptop to connect to Arduino board
* wheels and other mechanical parts

For a detailed part list look at 
https://sites.google.com/dis.uniroma1.it/marrtino/part-list?authuser=0

## Quick setup guide

### Flashing Ardunio
The first step is compiling and flasking the arduino with the firmware
To this end:
* connect the arduino
* check where the system has placed the device, using
  the command "dmesg"
* make and upload the software

```
cd arduino
make upload
```

You should see something like this on your shell

```
    show things
```

### Building the Monitor Program
Here we build a set of programs needed to contol
the arduino running the firmware from the PC.

* If you are using UBUNTU 16.04, you will have 
  to edit src/Makefile and uncomment the following line

```
#uncomment the line below for ubuntu 16_04.
# CXXOPTS += -D__UBUNTU_16_04__

```

* enter in the src and issue make

This will build a bunch of programs.
The most important is orazio_robot_websocket_server

Start it from src/ with the following arguments

```
./orazio_robot_websocket_server -resource-path ../html -serial-device /dev/ttyACM0

```
Options
* -resource-path <html_folder>, should be followed by the html directory in the package.
* -serial-device /dev/ttyACM0 or /dev/ttyACM0 should be the serial port where the arduino
  is detected.

This will start  web server interface to control the robot.

### Building the Monitor Program (ROS instructions)

srrg_orazio_core is a Catkin package. If you have ROS
You can just build it by issuing "catkin make" from your catkin workspace.
In this case the programs are accessible via rosrun


```
rosrun srrg_orazio_core orazio_robot_websocket_server -resource-path <where is the html directory of the package>

```


Open a browser and type localhost:9000 in the url (with arduino attached)

Proceed on that and execute the instructions to assemble and tune the robot.
Do that incrementally.


Enjoy,
	G.
