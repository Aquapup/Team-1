"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4A - LIDAR Safety Stop
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
global counter
global angle 
angle = 0 
counter = 0 
rc = racecar_core.create_racecar()

# >> Constants
# The (min, max) degrees to consider when measuring forward and rear distances
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)

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
    print(
        ">> Lab 4A - LIDAR Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override forward safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left bumper = override rear safety stop\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print forward and back distances"
    )


def update():
    global counter 
    global angle 
    #counter = 0 
    
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    scan = rc.lidar.get_samples()
    
    if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 150:
        
        counter += rc.get_delta_time()
        print("counter: " + str(counter)) 
        if counter < 2: 
            rc.drive.set_speed_angle(-1, angle)
            #rc.drive.set_speed_angle(-1, 0)
        else:
            counter = 0 
            rc.drive.stop()
    else:
        rc.drive.set_speed_angle(1, 0)

    rc.display.show_lidar(scan)
    # Use the left joystick to control the angle of the front wheels
  


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()