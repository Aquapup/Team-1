"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

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
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    # Allow the user to override safety stop by holding the right bumper.


    def get_closest_pixel(depth_image: NDArray[(Any, Any), np.float32], kernel_size: int = 5) -> NDArray[(Any, ...), Any]:
    
        depth_image = (depth_image - 0.01) % 10000
        depth_image = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
        minPixel = cv.minMaxLoc(depth_image)
        finalMinPixel = (minPixel[2])
        return (finalMinPixel[1], finalMinPixel[0])
    

    def crop(image: NDArray[(Any, ...), Any], top_left_inclusive: Tuple[float, float], bottom_right_exclusive: Tuple[float, float]) -> NDArray[(Any, ...), Any]:
        r_min, c_min = top_left_inclusive
        r_max, c_max = bottom_right_exclusive
        return image[r_min:r_max, c_min:c_max]
    

    top_left_inclusive = (0, 0)
    bottom_right_exclusive = (int((rc.camera.get_height()/3)*2), rc.camera.get_width())

    cropped_image = crop(depth_image, top_left_inclusive, bottom_right_exclusive)
    row = get_closest_pixel(cropped_image)[0]
    col = get_closest_pixel(cropped_image)[1]
    if depth_image[row][col] < 20:
        rc.drive.stop()


    
    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
