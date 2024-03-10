"""""""""""""""""""""""""""""""""""
    TANEGASHIMA ROCKET CONTEST 2024
    ASTRUM RUNBACK MAIN PROGRAM
    
    Author : Yuzu
    Language : Python Ver.3.9.2
    Last Update : 03/10/2024
"""""""""""""""""""""""""""""""""""


import os
import logger
import time
import datetime
import csv
import yaml
# import cv2
# import rclpy
# from rclpy.node import Node
# from pycoral.adapters.common import input_size
# from pycoral.adapters.detect import get_objects
# from pycoral.utils.dataset import read_label_file
# from pycoral.utils.edgetpu import make_interpreter
# from pycoral.utils.edgetpu import run_inference
import gnss
import bno055
import motor
import ground
import floating
# import cone_detection
import chat


print("Hello World!!")
now = datetime.datetime.now()
directory_path = "./data/" + now.strftime('%Y%m%d %H:%M:%S')
if not os.path.exists(directory_path):
    os.makedirs(directory_path)
error_log = logger.ErrorLogger(directory_path)
drive = motor.Motor()
drive.stop()
# destination point(lon, lat)
with open('settings.yaml') as yml:
    settings = yaml.safe_load(yml)
des = [settings['destination']['longitude'], settings['destination']['latitude']]
# rclpy.init()
# node = chat.Truck_Node()
# rclpy.spin(node)


"""
phase 1 : Floating
      2 : Load Sample
      3 : Head to Goal
      4 : Image Processing
      5 : Reach the goal
"""

# Head to the goal
reach_goal = False
phase = 3
ground_log = logger.GroundLogger(directory_path)
# img_proc_log = logger.ImgProcLogger(directory_path) 
# cap = cv2.VideoCapture(0) # /dev/video0
# if cap.isOpened() == False:
#     print("Error opening video stream or file")
# interpreter = make_interpreter('../model/red_cone.tflite')
# interpreter.allocate_tensors()
# labels = read_label_file('../model/red_cone.txt')
# inference_size = input_size(interpreter)   
            
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
        if distance <= 3: # Reach the goal within 3m
            print("Close to the goal")
            ground_log.end_of_ground_phase()
            phase = 4
            reach_goal = True
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

            
    # """
    # Image Processing Phase
    # """
    # print("phase : ", phase)
    # not_found = 0
    # while phase == 4 and cap.isOpened():
    #     drive.forward()
    #     try:
    #         percent, cone_loc, ditected_img_name, area_p = cone_detection.detect_cone(cap, inference_size, interpreter, labels, directory_path)
    #         img_proc_log.img_proc_logger(cone_loc, percent, ditected_img_name, area_p)
    #         print("percent:", percent, "location:", cone_loc)
    #     except Exception as e:
    #             print("Error : Image processing failed")
    #             phase = 5
    #             reach_goal = True
    #             error_log.img_proc_error_logger(phase, distance=0)
    #             with open('sys_error.csv', 'a') as f:
    #                 now = datetime.datetime.now()
    #                 writer = csv.writer(f)
    #                 writer.writerow([now.strftime('%H:%M:%S'), 'Image processing failed', str(e)])
    #                 f.close()
    #             drive.stop()
    #             break
    #     # Goal judgment
    #     if area_p > 1.0:
    #         print("Reach the goal")
    #         phase = 5
    #         reach_goal = True
    #         img_proc_log.end_of_img_proc_phase()
    #         drive.forward()
    #         time.sleep(3)
    #         drive.stop()
    #         break
    #     if cone_loc == "right":
    #         drive.turn_right()
    #         time.sleep(0.3)
    #     elif cone_loc == "left":
    #         drive.turn_left()
    #         time.sleep(0.3)
    #     elif cone_loc == "not found":
    #         not_found += 1
    #     if not_found >= 10:
    #         print('Error : Cone not found')
    #         drive.stop()
    #         phase = 3
    #         break
    

# cap.release()
# cv2.destroyAllWindows()
# node.destroy_node()
# rclpy.shutdown()