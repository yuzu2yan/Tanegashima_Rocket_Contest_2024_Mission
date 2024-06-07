# Tanegashima_Rocket_Contest_2024_Mission
![release_date](https://img.shields.io/badge/release_date-May_2024-yellow)
[![python](https://img.shields.io/badge/python-v3.9.2-blue)](https://www.python.org/downloads/release/python-392/)
[![openCV](https://img.shields.io/badge/OpenCV-v4.7.0-blue)](https://docs.opencv.org/4.7.0/)
![linux11](https://img.shields.io/badge/os-linux11-blue)  
[![python](https://img.shields.io/badge/-Python-F9DC3E.svg?logo=python&style=flat)](https://www.python.org/)
[![linux](https://img.shields.io/badge/-Linux-6C6694.svg?logo=linux&style=flat)](https://www.linux.org/)
[![raspberrypi](https://img.shields.io/badge/-Raspberry%20Pi-C51A4A.svg?logo=raspberry-pi&style=flat)](https://www.raspberrypi.com/)
## 🥉3rd place in mission division of Tanegashima Rocket Contest 2024🚀!!

This is a project of Team Fukawani Express, CanSat and Mission Division of Tanegashima Rocket Contest 2024.   

<img width="400px" alt="intro_img" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/assets/89567103/36a8ce0d-23ff-42c2-94db-d2784f731f34">

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


<img width="400px" alt="arm" src="https://raw.github.com/wiki/yuzu2yan/Tanegashima_Rocket_Contest_2024_Mission/images/detection.gif">



Bearings are mounted on the axles of the tires to absorb the shock of landing. In addition, rigidity is secured by using TPU for the tires and CFRP for the body.

### Truck



The TPU accelerator allows for 30 object detections per second and goal detection using the Tensorflow framework. 


## Software Configuration
Language : Python 3.9.2  
OS       : Raspberry Pi OS Lite (32-bit)   
Raspbian GNU/Linux 11 (bullseye)    
Kernel   : Ver.5.15    
OpenCV   : Ver.4.7.0   
Tensorflow: Ver.2.15.0

## Hardware Configuration

Computer                   : Raspberry pi4  
GPS                        : GYSFDMAXB  
9-axis sensor              : BNO055  
Barometric pressure sensor : BME280   
Vision Camera              : ELP 1080P Global Shutter USB Camera    
ToF Camera                 : Arducam ToF Camera  
TPU                        : Coral USB Accelerator  
Motor Driver               : BD6231F  


## Program Configuration

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
- tof_distance.py
      Measure the position and distance to the red cone using a ToF camera.
- motor.py  
    This class deals with motors, controlling tires and deployment motors.

## Result
It was retired due to a damaged circuit board before the drop.

<img width="400px" alt="result_img" src="https://github.com/yuzu2yan/Tanegashima_Rocket_Contest_2024_Runback/assets/89567103/8ef6777b-da15-459b-8ad7-a3177bff7803">

