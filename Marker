# Michael Castellano
# EENG 350
# Demo 1
# Marker detection and angle calculation

# Initialize Libraries
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

# Global Variables
global quadrant, aruco_dict, parameters, frame, corners, ids, rejectedImgPoints, frame_markers
# Turn on video
video = cv2.VideoCapture(0)

# Calculates angle from center of camera
def aruco_location():
    # Adjust fov
    fov = .12 *1280*7                   #3.05mm but in inches
    real_distance = 6.0 /1.8            # 5.5 inch
    x_distance = (corners[0][0][1][0]- corners[0][0][0][0]) ** 2
    y_distance = (corners[0][0][1][1] - corners[0][0][0][1]) ** 2
    image_distance = corners[0][0][1][0] - corners[0][0][0][0]
    
    # Finds the distance from the center camera to the center of the aruco marker
    distance = fov * real_distance / image_distance
    
    reference1 = math.sqrt((corners[0][0][3][0] - corners[0][0][0][0])**2 +
                           (corners[0][0][3][1] - corners[0][0][0][1])**2) + corners[0][0][0][0]
                           
    reference2 = math.sqrt((corners[0][0][2][0] - corners[0][0][1][0])**2 +
                           (corners[0][0][2][1] - corners[0][0][1][1])**2) + corners[0][0][2][0]
                           
    imageCenter = (reference1 / 2) + (reference2 / 2) / 2
    
    # Calculate center of marker based on four corners
    x_cord = ((((corners[0][0][1][0] - corners[0][0][0][0])/2)
               + ((corners[0][0][3][0] - corners[0][0][2][0])/2)) / 2) + corners[0][0][0][0]
    
    y_cord = ((((corners[0][0][1][1] - corners[0][0][0][1])/2)
               + ((corners[0][0][3][1] - corners[0][0][2][1])/2)) / 2) + corners[0][0][0][1]
    
    # Having center of marker in inches, calculate corresponding angle
    center = ((1280/2) - (x_cord-300 * -1))
    X = center / (distance *128)
    
    # Measurers angle from [30,0] from left side to right side of view
    angle = math.asin(X)*1000
    print("Real Distance: ", distance)
    print("Angle: " , angle)

    
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
        

# Stop recoring if 'q' is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    
# Stop video and close all windows
video.release()
cv2.destroyAllWindows()
