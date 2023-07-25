"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 29

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_ROOF = ((90, 0), (rc.camera.get_height(), rc.camera.get_width()))
# Colors, stored as a pair (hsv_min, hsv_max)
#BLUE = ((100, 0, 100), (150, 255, 170))  # The HSV range for the color blue
BLUE = ((90,50,50),(120,255,255))
# TODO (challenge 1): add HSV ranges for other colors
RED = ((160,180,150),(0,255,255)) 
PINK = ((130,0,0),(180,255,150))
GREEN = ((40,120,120),(100,255,255))
YELLOW = ((20,50,70), (32,240,255))
#YELLOW = ((20,50,50), (32,255,255))
#GREEN = ((40, 180, 140),(100, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
lasterr=0
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
pangle=0
dangle=0
########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()
    #print("image: " + str(image))
    #print(type(image))
    #rc.display.show_color_image(image)
    # threshold = 50
    # if rc_utils.get_contour_area(contours) < threshold: 
    #         contours = None 
    if image is None:
        contour_center = None
        contour_area = 0
        print("image is none")
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        image = rc_utils.crop(image, CROP_ROOF[0], CROP_ROOF[1])
        #image = image[100:320,::]
        if rc.camera.get_width() == 0:
            print("FAIL!")
        else:
            rc.display.show_color_image(image)
        # Find all of the blue contours
       # print("find contours: " + str(rc_utils.find_contours(image, GREEN[0], GREEN[1])))
        # if(np.size(rc_utils.find_contours(image, RED[0], RED[1]))!=0):
        #     contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
        # else:
        
       # print("found green")
        #print("contours: " + str(contours))
         
        contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),30)
        
        print("found yellow")
        if (np.size(contours)==0 ):

           # print("contours: " + str(contours))
            
            contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], GREEN[1]),30)
            # if rc_utils.get_contour_area(contours) < threshold: 
            #     contours = None 
            #print("contours: " + str(contours))
          #  if(contours.all()!=None):
           #     print("found blue")
       #     print(contours)
            if (contours.all() == None):
             #   print("found red")
                #print("contours: " + str(contours))
                contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], GREEN[1]),30)
                # if rc_utils.get_contour_area(contours) < threshold: 
                #     contours = None 
            if(contours.all()==None):
                contours= rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
                # if rc_utils.get_contour_area(contours) < threshold: 
                #     contours = None 
        # Select the largest contour
        #print("final counter",contours)
       # print(np.shape(contours))
        #contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        #check if image is None
        #print(contour_center)
       # print("contour:",contours)
        if contours is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contours)
            contour_area = rc_utils.get_contour_area(contours)
           # print("contour center: " + str(contour_center))
            # Draw contour onto the image
            rc_utils.draw_contour(image, contours)
            rc.display.show_color_image(image)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    print("starting program")
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )

    
def clamp(value: float, vmin: float, vmax: float) -> float:

    if value < vmin:
        return vmin
    elif value > vmax:
        return vmax
    else:
        return value

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global Kp
    global Kd
    global contour_center
    global pangle
    global dangle
    global lasterr
   # print("in update")
    # Search for contours in the current color image
    update_contour()
    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    print(contour_center)
    
    if contour_center is not None:
        # Current implementation: bang-bang control (very choppy)
        # TODO (warmup): Implement a smoother way to follow the line
        
        Kp =0.5
        Kd= 0.2
        setpoint = rc.camera.get_width()/2
        #angle = Kp*(contour_center[1]-(setpoint)) #tried
        #pangle=((angle/320)*2-1)
        # dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
        # lasterr=(contour_center[1])
        # angle=pangle+dangle*rc.get_delta_time()
        #angle = (contour_center[1]) #tried
        # print("contour center",contour_center[1])
        # print(rc.camera.get_width())
        # new_max = 1
        # new_min = -1
        # print("pangle: " + str(pangle))
        # print("dangle: " + str(dangle)) 
        #angle = (angle/(old_max-old_min) * (new_max-new_min)+new_min)
       # print("old angle: " + str(angle))
        #angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        #angle = rc_utils.remap_range(angle,  Kp*-setpoint, Kp*setpoint, -1, 1)
        # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        pangle = ((contour_center[1]/320)*2-1)
        angle=clamp(Kp*pangle,-1,1)
        # dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
        # lasterr=(contour_center[1])
        # angle=(pangle+dangle)*rc.get_delta_time()
        # print("first angle: ", angle)
        # angle = clamp(angle, 0, rc.camera.get_width())
        # print("angle after clamping: ", angle)
        # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        angle = Kp*pangle
        #angle=clamp(angle,0,1)
        #angle -=1
        print("final angle: ", angle)
        # if angle < -1:
        #     angle = -1
        # elif angle > 1:
        #     angle = 1
   
        # lasterr=(contour_center[1])
        # angle=pangle+dangle*rc.get_delta_time()
        #angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        # if contour_center[1] < (rc.camera.get_width()/ 2):
        #     angle = Kp*abs(contour_center[1]-rc.camera.get_width())
        # else:
        #     angle = 1

    # Use the triggers to control the car's speed
    # forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # # #forwardSpeed = rc.controller.is_down(rc.controller.)
    # # #backSpeed = rc.controller.is_down(rc.controller.Button.B)
    # speed = forwardSpeed - backSpeed
    speed = 0.15

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)
    if rc.controller.was_pressed(rc.controller.Button.X):
        rc.drive.stop()


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()