"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
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
    global markers
    image = rc.camera.get_color_image_async()
    ar_markers = cv.aruco.detectMarkers(
        image,
        cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
        parameters=cv.aruco.DetectorParameters_create()
    )

    # TODO: Make a deep copy of image
    image_copy = copy.deepcopy(image)

    color = (255, 255, 0)
    cv.aruco.drawDetectedMarkers(image_copy, ar_markers[0], ar_markers[1], color)
    
    corners = ar_markers[0]
    ids = ar_markers[1]

    # Access the corners of the first AR marker detected, converting the data type from float32 to int32
    first_corners = corners[0][0].astype(np.int32)

# TODO: Swap each corner from (col, row) to (row, col) format
    new = []
    for i in first_corners:
        temp = (i[1], i[0])
        new.append(temp)
    first_corners = new

    # TODO: Set first_id to the id of the first AR marker detected
    print(ids[0][0])
    first_id = ids[0][0]

    print("first_corners: ", first_corners)
    print("first_id: ", first_id)

    # TODO: Turn left if we see a marker with ID 0 and right for ID 1
    

    # TODO: If we see a marker with ID 199, turn left if the marker faces left and right
    # if the marker faces rights
       
    #  # TODO: If we see a marker with ID 2, follow the color line which matches the color
    # # border surrounding the marker (either blue or red). If neither color is found but
    # # we see a green line, follow that instead.

    #     if (not(contours is not None) ):

    #        # print("contours: " + str(contours))
            
    #         contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
    #         if(not(contours is not None)):
    #             contours= rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0],GREEN[1]))
            
        

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
