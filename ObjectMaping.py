# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 11:46:04 2024

@author: mohan
"""

import cv2
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np
import math

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#Align depth to color for better correspondence
align_to = rs.stream.color
align = rs.align(align_to)
COLORS = [(255, 0, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]

pipeline.start(config)

Frame_center_X = 320
Frame_center_Y = 240

#model download
model = YOLO('yolov8n.pt')
class_names = model.names

camera = cv2.VideoCapture(1)
W = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

#Camera Cordinates
xc = 99
yc = 446

#Map location
Map = r"D:\Academics\RAwork\ObjectMapping\tstlab_apr11(small).jpg"
map_image = cv2.imread(Map)

#Dimensions of Map
xm = 213
ym = 447

scale = 26 # 1meter = 26 pixels

def angle_estimater(center_x, center_y):
    HFOV = 86
    VFOV = 57
    H_theta = ((center_x - W/2)/(W/2))*(HFOV/2)
    V_theta = ((center_y - H/2)/(H/2))*(VFOV/2)
    return (int(H_theta), int(V_theta))

def horPlaneDist(V_theta, distance):
    # print("horDistance :", distance*(math.cos(V_theta / 180. * math.pi)))
    return distance*(math.cos(V_theta / 180. * math.pi))

def obj_cordinates(H_theta, Hpdistance):
    a = Hpdistance*(math.sin(H_theta / 180. * math.pi))
    b = Hpdistance*(math.cos(H_theta / 180. * math.pi))
    #print(a, b)
    # print('a:', a)
    # print('b :',b)
    x = xc + a
    y = yc - b
    # print("x:",x)
    # print('y', y)
    return (x, y)

try:
    while True:
        # Wait for a coherent pair of color frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        # Get aligned color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        results = model(source= color_image, classes = 0)
        
        boxes = results[0].boxes.xyxy.tolist()
        class_ids=results[0].boxes.cls.tolist()
        map_copy = map_image.copy()
        for box, class_id in zip(boxes, class_ids):
            object_Dict = dict()
            x1, y1, x2, y2 = box
            
            class_name = class_names[int(class_id)]
            
            center_x = int(x1+((x2-x1)/2))
            center_y = int(y1+((y2-y1)/2))            
            

            distance = depth_frame.get_distance(center_x,center_y)
            print('Distance :',distance)
            
            H_theta, V_theta = angle_estimater(center_x, center_y)
            print("H_theta :", H_theta, "V_theta :", V_theta)
            Hpdistance = int(horPlaneDist(V_theta, distance)) * scale #converting meters into pixels
            
            x, y = obj_cordinates(H_theta, Hpdistance)
            
            #image = np.zeros_like(image)
            cv2.circle(map_copy, (int(x),int(y)), radius=3, color=(0, 0, 255), thickness=-1)
            
            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), COLORS[int(class_id) % len(COLORS)], 2)
            cv2.putText(color_image, f"{class_name} : {distance :0.2f} m : {H_theta:0.2f} : {V_theta:0.2f}", (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        COLORS[int(class_id) % len(COLORS)], 2)
            #cv2.circle(color_image, (center_x, center_y), 5, (0,255,0), -1)
            
            # cv2.line(color_image, (int(W/2), 0), (int(W/2), H), (0, 255, 0), 1)
    
            # cv2.line(color_image, (0, int(H/2)), (W, int(H/2)), (0, 255, 0), 1)

        
        #cv2.imshow("Depth camera", depth_colormap)
        cv2.imshow("Color camera", color_image)
        cv2.imshow('map', map_copy)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    pipeline.stop()
    cv2.destroyAllWindows()