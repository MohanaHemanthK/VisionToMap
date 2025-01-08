# -*- coding: utf-8 -*-
"""
Created on Mon Apr 15 11:26:16 2024

@author: mohan
"""
## ---- For finding coordinates on the image, run the below code ----

# import cv2

# coordinates = (-1,-1)

# def click_event(event, x, y, flags, param):
#   if event == cv2.EVENT_LBUTTONDOWN:
#     coordinates = (x, y)
#     print(f"Clicked at: {coordinates}")
#     #return(x,y)

# Map = r"D:\Academics\RAwork\ObjectMapping\tstlab_apr11(small).jpg"

# image = cv2.imread(Map)

# if image is None:
#     print('Error to read the image!')

# else:
#     cv2.namedWindow('map')
#     cv2.setMouseCallback("map", click_event)
#     image = cv2.circle(image, (99,446), radius=3, color=(0, 0, 255), thickness=-1)
#     cv2.imshow('map', image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
    
## ---- For finding coordinates of the realtime feed run the below code ----

import cv2

# List to store clicked coordinates
coordinates_list = []

# Mouse callback function
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coordinates = (x, y)
        print(f"Clicked at: {coordinates}")
        coordinates_list.append(coordinates)

# Open camera
camera = cv2.VideoCapture(0)

while True:
    success, frame = camera.read()

    if not success:
        print("Camera not found!")
        break

    cv2.namedWindow('map')
    cv2.setMouseCallback("map", click_event)

    # Draw circles for all stored coordinates
    for coordinates in coordinates_list:
        frame = cv2.circle(frame, coordinates, radius=3, color=(0, 0, 255), thickness=-1)

    cv2.imshow('map', frame)
    
    # Press ESC to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

camera.release()
cv2.destroyAllWindows()

