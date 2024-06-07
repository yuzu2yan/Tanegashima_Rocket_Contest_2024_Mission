# Tanegashima_Rocket_Contest_2024_Mission
![release_date](https://img.shields.io/badge/release_date-Mar_2024-yellow)
[![python](https://img.shields.io/badge/python-v3.9.2-blue)](https://www.python.org/downloads/release/python-392/)
[![openCV](https://img.shields.io/badge/OpenCV-v4.7.0-blue)](https://docs.opencv.org/4.7.0/)
[![ROS](https://img.shields.io/badge/ROS-Humble-blue)](https://docs.ros.org/en/humble/index.html)  
[![python](https://img.shields.io/badge/-Python-F9DC3E.svg?logo=python&style=flat)](https://www.python.org/)
[![linux](https://img.shields.io/badge/-Linux-6C6694.svg?logo=linux&style=flat)](https://www.linux.org/)
[![ubuntu](https://img.shields.io/badge/-Ubuntu-6F52B5.svg?logo=ubuntu&style=flat)](https://releases.ubuntu.com/jammy/)
[![raspberrypi](https://img.shields.io/badge/-Raspberry%20Pi-C51A4A.svg?logo=raspberry-pi&style=flat)](https://www.raspberrypi.com/)
## 🥉3rd place in mission division of Tanegashima Rocket Contest 2024🚀!!

This is a project of Team Fukawani Express, CanSat and Mission Division of Tanegashima Rocket Contest 2024. 

<img width="400px" alt="intro" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/assets/89567103/ddd4bac6-31c1-4667-b668-c72122fdd9c8">


## Mission  
Simulate sample returns. After the truck is launched and lands, the arm loads the sample onto the truck. The truck then travels to the destination.

## Mission Sequence  
The truck is lifted by crane to 30 meters above the ground and dropped. After a soft landing, the truck heads for the recovery point. The arm also heads to the recovery point, collects the sample, and loads it onto the truck. Synchronization between the two vehicles via an access point and communicate via Wi-Fi, and they are controlled by GPS and geomagnetism, while sample collection and loading are guided by position estimation using markers. When approaching the goal point, image processing is used to achieve a zero-distance goal.

<img width="500px" alt="Mission_Overview" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/assets/89567103/78217d04-819f-4137-8059-ff5c0865a1dd">


## Success Criteria  

| | Statement | Methodology |
| ---- | ---- |---|
| Minimum Success |- Truck makes a soft landing <br> - Truck moves to sample|Confirmation by visual inspection and log|
| Full Success |- Arm retrieves the sample <br> - Successfully loaded onto a truck|Confirmation by visual inspection and log|
| Extra Success |Truck travels to destination|Confirmation by visual inspection and log|

## Feature
### Arm

<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/arm.gif">

- The arm is extended and retracted by a link mechanism
- Operation detection by microswitch

### Truck

<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/truck.gif">

- Equipped with ride height adjustment mechanism
- Can be used on both sides

### Loading & Sampling

<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/loading.gif">

Automatic self-location estimation, loading and sample retrieval

### Self-location Estimation

<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/marker.gif">

- Adopts ROS as middleware
- Self-location estimation with [ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) 
- Sample collection and loading with autonomous movement by tag detection

### Real-Time Object Detection

<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/detection.gif">

Real-time goal detection using the Tensorflow framework

## Software Configuration
Language : Python 3.9.2  
OS       : Ubuntu 22.04  
ROS      : ROS2 Humble   
OpenCV   : Ver.4.7.0   
Tensorflow: Ver.2.15.0  

## Hardware Configuration

Computer                   : Raspberry Pi4  
GPS                        : GYSFDMAXB  
9-axis sensor              : BNO055  
Barometric pressure sensor : BME280   
Vision Camera              : ELP 1080P Global Shutter USB Camera  
Access Point               : TP-Link EAP610-Outdoor


## Program Configuration
### Arm
- collect_sample.py  
  Detects tag on sample using the marker and automatically collects it
- load_sample.py  
  Detects truck tag using the marker and automatically loads the sample
- gnss.py  
  Obtains latitude and longitude from a GPS module every second. The acquisition program runs as a daemon.
- chat.py  
  Interact with the truck via ROS topic communication.
- motor.py  
  This class deals with motors, controlling tires and deployment motors.

### Truck
- main.py  
    Main program. Operates according to the flow of the mission sequence.
- logger.py  
    Define a log class. Create logs for each phase and error and output them in csv format.
- floating.py  
    Used to calculate altitude; obtains air pressure and temperature from the BME280 module and calculates the altitude relative to the initial altitude.
- ground.py  
    This program is used in the ground phase. It calculates the distance and angle to the goal based on geomagnetic and GPS information, and determines the control.
- bme280.py  
    Obtain air pressure and temperature data using BME280.
- gnss.py  
    Obtains latitude and longitude from a GPS module every second. The acquisition program runs as a daemon.
- bno055.py  
    Obtain barometric pressure and acceleration data from BNO055. Each value is automatically calibrated by the built-in microcomputer, and the degree of calibration can be checked.
- cone_detection.py    
    It is an image processing module that takes a picture and detects red pylons in real time from the image.
- chat.py  
    Interact with the arm via ROS topic communication.
- motor.py  
    This class deals with motors, controlling tires and deployment motors.

## Result
The truck happened to land at the sample collection point, and the arm also collected the sample and even approached the truck.

<img width="400px" alt="collect_sample" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/assets/89567103/e0cabdb5-2454-46f2-9fe7-a2cb77e8bbf0">

However, the vibration of the movement during loading caused the sample to drop, and the mission was retired. The truck also failed to make a soft landing, damaging a tire upon landing.

<img width="400px" alt="mission_retire" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/assets/89567103/03ec74c3-2cef-4ae0-b455-972aa5d4db9f">

**This mission won third place in the Tanegashima Rocket Contest2024!!**
