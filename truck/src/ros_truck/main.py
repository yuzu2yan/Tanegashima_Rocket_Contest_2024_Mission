"""""""""""""""""""""""""""""""""""
    TANEGASHIMA ROCKET CONTEST 2024
    ASTRUM RUNBACK MAIN PROGRAM
    
    Author : Yuzu
    Language : Python Ver.3.9.2
    Last Update : 03/09/2024
"""""""""""""""""""""""""""""""""""


import os
import logger
import time
import datetime
import csv
import yaml
import cv2
import rclpy
from rclpy.node import Node
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
import gnss
import bno055
import motor
import ground
import floating
import cone_detection
import chat


print("Hello World!!")
now = datetime.datetime.now()
directory_path = "./../data/" + now.strftime('%Y%m%d %H:%M:%S')
if not os.path.exists(directory_path):
    os.makedirs(directory_path)
error_log = logger.ErrorLogger(directory_path)
drive = motor.Motor()
drive.stop()
# destination point(lon, lat)
with open('settings.yaml') as yml:
    settings = yaml.safe_load(yml)
des = [settings['destination']['longitude'], settings['destination']['latitude']]
rclpy.init()
node = chat.Truck_Node()
rclpy.spin(node)


"""
phase 1 : Floating
      2 : Load Sample
      3 : Head to Goal
      4 : Image Processing
      5 : Reach the goal
"""


"""
Floating Phase
"""
phase = 2 # TODO change
if phase == 1:
    print("phase : ", phase)
    floating_log = logger.FloatingLogger(directory_path)
    """
    state 
        Rising
        Falling
        Landing
        Error
    """
    state = 'Rising'
    floating_log.state = 'Rising'
    start = time.time()
    # The flag that identifies abnormalities in the barometric pressure sensor
    error_baro = 0
    init_altitude = 0
    data = floating.cal_altitude(init_altitude)
    init_altitude = data[2]
    altitude = init_altitude
    print("initial altitude : {}." .format(init_altitude))
    floating_log.floating_logger(data)
    print("Rising phase")
while phase == 1:
    while state == 'Rising':
        data = floating.cal_altitude(init_altitude)
        altitude = data[2]
        floating_log.floating_logger(data)
        print("Rising")
        # Incorrect sensor value
        if altitude < -5:
            error_baro += 1
            if error_baro >= 15:
                state = 'Error'
                floating_log.state = 'Error'
                error_log.baro_error_logger(phase, data)
                print("Error : Altitude value decreases during ascent")
            time.sleep(1.5)
            continue
        if altitude >= 25:
            state = 'Ascent Completed'
            floating_log.state = 'Ascent Completed'
        now = time.time()
        if now - start > 480:
            print('8 minutes passed')
            state = 'Landing'
            floating_log.state = 'Landing'
            floating_log.end_of_floating_phase('Landing judgment by passage of time.')
            break
        print("altitude : {}." .format(altitude))
        time.sleep(1.5)
    while state == 'Ascent Completed':
        data = floating.cal_altitude(init_altitude)
        altitude = data[2]
        floating_log.floating_logger(data)
        print("Falling")
        if altitude <= 2:
            state = 'Landing'
            floating_log.state = 'Landing'
            floating_log.end_of_floating_phase()
        now = time.time()
        if now - start > 480:
            print('8 minutes passed')
            state = 'Landing'
            floating_log.state = 'Landing'
            floating_log.end_of_floating_phase('Landing judgment by passage of time.')
            break
        print("altitude : {}." .format(altitude))
        time.sleep(0.2)
    while state == 'Error':
        data = floating.cal_altitude(init_altitude)
        altitude = data[2]
        floating_log.floating_logger(data)
        now = time.time()
        if now - start > 480:
            print('8 minutes passed')
            state = 'Landing'
            floating_log.state = 'Landing'
            floating_log.end_of_floating_phase('Landing judgment by passage of time.')
            break
        time.sleep(1)
    print("Landing")
    break


drive.descending() # Separation mechanism activate and descent detection
drive.stop()
if bno055.read_Mag_AccelData()[5] < 0:
    upside_down = True
else:
    upside_down = False
    drive.rising()


# Head to sample(arm) position
reach_sample = False
phase = 2
print("phase : ", phase)
node.state = 2 # landing and head to sample
ground_log = logger.GroundLogger(directory_path)
ground_log.state = 'Normal'
img_proc_log = logger.ImgProcLogger(directory_path)
    

while not reach_sample:
    while gnss.read_GPSData() == [0,0]:
            print("Waiting for GPS reception")
            time.sleep(5)
    gps = gnss.read_GPSData()
    data = ground.is_heading_goal(gps, [node.long, node.lat])
    distance = ground.cal_distance(gps[0], gps[1], node.long, node.lat)
    print("distance : ", distance)
    ground_log.ground_logger(data, distance)
    # Reach sample judgment
    if distance <= 3: # Reach the sample within 3m
        print("Close to the sample")
        ground_log.end_of_ground_phase()
        reach_sample = True
        phase = 3
        break
    count = 0
    while data[3] != True: # Not heading the sample
        if count > 7 or distance <= 3:
            break
        if data[4] == 'Turn Right':
            drive.turn_right()
        elif data[4] == 'Turn Left':
            drive.turn_left()
        time.sleep(0.3)
        drive.forward()
        gps = gnss.read_GPSData()
        distance = ground.cal_distance(gps[0], gps[1], node.long, node.lat)
        print("distance : ", distance)
        data = ground.is_heading_goal(gps, [node.long, node.lat])
        ground_log.ground_logger(data, distance)
        count += 1
    # End of Orientation Correction

# Reach the sample    
drive.stop()
node.state = 2

# Wait loading sample
while node.state != 3:
    print("waiting")
    time.sleep(3)


# Head to the goal
reach_goal = False
phase = 3
img_proc_log = logger.ImgProcLogger(directory_path) 
cap = cv2.VideoCapture(0) # /dev/video0
if cap.isOpened() == False:
    print("Error opening video stream or file")
interpreter = make_interpreter('../model/red_cone.tflite')
interpreter.allocate_tensors()
labels = read_label_file('../model/red_cone.txt')
inference_size = input_size(interpreter)   
            
while not reach_goal:
    """
    Ground Phase
    """
    print("phase : ", phase)
    while gnss.read_GPSData() == [0,0]:
            print("Waiting for GPS reception")
            time.sleep(5)
    while phase == 3:
        gps = gnss.read_GPSData()
        data = ground.is_heading_goal(gps, des)
        distance = ground.cal_distance(gps[0], gps[1], des[0], des[1])
        print("distance : ", distance)
        ground_log.ground_logger(data, distance)
        # Goal judgment
        if distance <= 7: # Reach the goal within 7m
            print("Close to the goal")
            ground_log.end_of_ground_phase()
            phase = 4
            break
        count = 0
        while data[3] != True: # Not heading the goal
            if count > 7 or distance <= 7:
                break
            if data[4] == 'Turn Right':
                drive.turn_right()
            elif data[4] == 'Turn Left':
                drive.turn_left()
            time.sleep(0.3)
            drive.forward()
            gps = gnss.read_GPSData()
            distance = ground.cal_distance(gps[0], gps[1], des[0], des[1])
            print("distance : ", distance)
            data = ground.is_heading_goal(gps, des)
            ground_log.ground_logger(data, distance)
            count += 1
        # End of Orientation Correction
        drive.forward()

            
    """
    Image Processing Phase
    """
    print("phase : ", phase)
    not_found = 0
    while phase == 4 and cap.isOpened():
        drive.forward()
        try:
            percent, cone_loc, ditected_img_name, area_p = cone_detection.detect_cone(cap, inference_size, interpreter, labels, directory_path)
            img_proc_log.img_proc_logger(cone_loc, percent, ditected_img_name, area_p)
            print("percent:", percent, "location:", cone_loc)
        except Exception as e:
                print("Error : Image processing failed")
                phase = 5
                reach_goal = True
                error_log.img_proc_error_logger(phase, distance=0)
                with open('sys_error.csv', 'a') as f:
                    now = datetime.datetime.now()
                    writer = csv.writer(f)
                    writer.writerow([now.strftime('%H:%M:%S'), 'Image processing failed', str(e)])
                    f.close()
                drive.stop()
                break
        # Goal judgment
        if area_p > 1.0:
            print("Reach the goal")
            phase = 5
            reach_goal = True
            img_proc_log.end_of_img_proc_phase()
            drive.forward()
            time.sleep(3)
            drive.stop()
            break
        if cone_loc == "right":
            drive.turn_right()
            time.sleep(0.3)
        elif cone_loc == "left":
            drive.turn_left()
            time.sleep(0.3)
        elif cone_loc == "not found":
            not_found += 1
        if not_found >= 10:
            print('Error : Cone not found')
            drive.stop()
            phase = 3
            break
    

cap.release()
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()