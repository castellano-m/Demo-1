# Michael Castellano
# EENG 350
# Demo 1
# Marker detection and angle calculation

from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
from PIL import Image
from cv2 import aruco
import time
import numpy as np
import argparse
import cv2
import PIL
import math

# Gloval Variables
global quadrant, aruco_dict, parameters, frame, corners, ids, rejectedImgPoints, frame_markers
# Turn on video
video = cv2.VideoCapture(0)
        
def aruco_location():
    fov = .12 *1280*7 #3.05mm but in inches
    real_distance = 5.5 # 5.5 inch
    x_distance = (corners[0][0][1][0]- corners[0][0][0][0]) ** 2
    y_distance = (corners[0][0][1][1] - corners[0][0][0][1]) ** 2
    image_distance = corners[0][0][1][0] - corners[0][0][0][0]
    distance = fov * real_distance / image_distance
    # Calculate center of marker based on four corners
    reference1 = math.sqrt((corners[0][0][3][0] - corners[0][0][0][0])**2 +
                           (corners[0][0][3][1] - corners[0][0][0][1])**2) + corners[0][0][0][0]
                           
    reference2 = math.sqrt((corners[0][0][2][0] - corners[0][0][1][0])**2 +
                           (corners[0][0][2][1] - corners[0][0][1][1])**2) + corners[0][0][2][0]
                           
    imageCenter = (reference1 / 2) + (reference2 / 2) / 2
    print(corners[0][0][0][0])
    print(corners[0][0][0][1])
    
    x_cord = ((((corners[0][0][1][0] - corners[0][0][0][0])/2)
               + ((corners[0][0][3][0] - corners[0][0][2][0])/2)) / 2) + corners[0][0][0][0]
    
    y_cord = ((((corners[0][0][1][1] - corners[0][0][0][1])/2)
               + ((corners[0][0][3][1] - corners[0][0][2][1])/2)) / 2) + corners[0][0][0][1]
    # Quad 1
    if ((x_cord < 320.0) and (y_cord < 180.0)):
        quadrant = 1
    # Quad 2
    elif ((x_cord >= 320.0) and (y_cord < 180.0)):
        quadrant = 2
    # Quad 3
    elif ((x_cord < 320.0) and (y_cord >= 180.0)):
        quadrant = 3
    # Quad 4
    elif ((x_cord >= 320.0) and (y_cord >= 180.0)):
        quadrant = 4
    center = (1280/2) - imageCenter
    # Measurers angle from [30,0] from left side to right side of view
    angle = math.atan(center / fov) * 180 / 3.14
    # Convert angle value between [-15,15]
    angle = (angle - 15) * -1
    print("Real Distance: ", distance)
    print("Angle: " , angle)
    print(quadrant)
    
while True:
    # Get current frame and convert to grayscale. Print frame
    check,frame = video.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Capturing", frame)
    
    # Checks for aruco marker
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
    if ids != None :   
        cv2.imshow("Capturing",frame_markers)
        aruco_location()
    else:
        quadrant = 0
        print(quadrant)

# Stop recoring if 'q' is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    
# Stop video and close all windows
video.release()
cv2.destroyAllWindows()
