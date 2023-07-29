"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

# Import Python libraries
import math
import copy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum

# Import Racecar library
import sys
sys.path.append("../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    # global ar_markers
    # image = rc.camera.get_color_image()
    # markers = rc_utils.get_ar_markers(image)
    # if len(markers) > 0:
    #     id = markers[0].get_id()
    # else:
    #     id = 1000000

    # if id == 1:
    #     rc.drive.set_speed_angle(1, 0)
    # elif id == 2:
    #     #state = wall following state
    # elif id == 3:
    #     #state = cone slalom state
    # elif id == 4:
    #     #state = stop state

    image = rc.camera.get_color_image_async()
    ar_markers = cv.aruco.detectMarkers(
        image,
        cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
        parameters=cv.aruco.DetectorParameters_create()
    )
    def show_image(image, size = 8) -> None:
        """
        Displays a color image in the Jupyter Notebook.
        """
        plt.figure(figsize=(size, size), dpi=100)
        plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
    # TODO: Make a deep copy of image
    image_copy = copy.deepcopy(image)

    color = (255, 255, 0)
    cv.aruco.drawDetectedMarkers(image_copy, ar_markers[0], ar_markers[1], color)

    show_image(image)
    show_image(image_copy)
  
  
#####################################################################################################################################################################
# Dark Past
#####################################################################################################################################################################

    # ar_markers = cv.aruco.detectMarkers(
    #     image,
    #     cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
    #     parameters=cv.aruco.DetectorParameters_create()
    # )
    
    # corners = ar_markers[0]
    # ids = ar_markers[1]

    # # Access the corners of the first AR marker detected, converting the data type from float32 to int32
    # first_corners = corners[0][0].astype(np.int32)

    # # TODO: Swap each corner from (col, row) to (row, col) format
    # new = []
    
    # for i in first_corners:
    #     temp = (i[1], i[0])
    #     new.append(temp)
    # first_corners = new

    # # TODO: Set first_id to the id of the first AR marker detected
    # print(ids[0][0])
    # first_id = ids[0][0]

    # print("first_corners: ", first_corners)
    # print("first_id: ", first_id)

    # class ARMarker:
    
    #     def __init__(self, marker_id, marker_corners):
    #         # TODO: Copy implementation of __id and __corners from your previous ARMarker class
    #         __id = marker_id
    #         __corners = marker_corners
                
    #         # TODO: Set the __orientation field based on the corners
    #         self.__orientation = Orientation.UP
    #         print("START")
    #         print(marker_corners)
    #         print("STOP")
                
    #     def get_id(self):
    #         # TODO: Copy implementation from your previous ARMarker class
    #         return id
    #         print("")
    #     def get_corners(self):
    #         # TODO: Copy implementation from your previous ARMarker class
    #         return corners
    #         print("")
    #     def get_orientation(self):
    #         # TODO: Return the orientation
    #         return self.__orientation

    #         print("")
    #     def __str__(self):
    #         # TODO: Update __str__ to include the ID, corners, and orientation
    #         ret = str(id) + ", " + str(corners) + ", " + str(self.__orientation)
    #         return ret
        
    # first_marker = ARMarker(first_id, first_corners)
    # print(first_marker)

    # def get_ar_markers(image):
    # # Gather raw AR marker data from ArUco
    #     aruco_data = cv.aruco.detectMarkers(
    #         image,
    #         cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
    #         parameters=cv.aruco.DetectorParameters_create()
    #     )
        
    #     # A list of ARMarker objects representing the AR markers found in aruco_data
    #     markers = []
    #     print((aruco_data[0]))
    #     for i in range(len((aruco_data[0]))):
    #         # TODO: For each marker in aruco_data, extract the corners and id, change the corners into (row, col) format,
    #         # and create an ARMarker object with this data (see section 3.1)
    #         arr = aruco_data[0][i]
    #         for j in range(len(arr)):
    #             temp = arr[j][0]
    #             arr[j][0]= arr[j][1]
    #             arr[j][1] = temp
    #         markers.append(ARMarker(ids[i][0], arr))
    #         # TODO: Add the new marker to markers
  
    # image = rc.camera.get_color_image_async()
    # markers = get_ar_markers(image)

    # for marker in markers:
    #     print(marker)
    #     print("\n----\n")  

    # Create an ARMarker object storing the id and corners of the first detected AR marker


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()