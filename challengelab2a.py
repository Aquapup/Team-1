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
import enum
from enum import Enum

class State(Enum):
    line_follow=0
    slalom=1
    parking=2

cur_state: State = State.slalom


counter=0
rcontour_area=0
blue2red2 =0
blue3red3=0
blue4red4=0
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 29

# A crop window for the floor directly in front of the car
# CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_ROOF = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))

CROP_WIDTHL = ((rc.camera.get_height(),rc.camera.get_width()),(0,160))
CROP_WIDTHR = ((rc.camera.get_height(),rc.camera.get_width()),(480,640))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90,50,50),(120,255,255))
# TODO (challenge 1): add HSV ranges for other colors
RED = ((160,180,150),(0,255,255)) 
PINK = ((130,0,0),(180,255,150))
GREEN = ((40,120,120),(100,255,255))
YELLOW = ((20,50,70), (32,240,255))
LIGHTPURPLE = ((116-15, 18-15, 215-15), (116+15, 18+15, 215+15))
ORANGE = ((0,160,190),(10,255,255))
PURPLE= ((100,60,80),(170,200,255))
#LIGHTPURPLE((130,),())
# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
s_phase=0
blue1red0= 0

########################################################################################
# Functions
########################################################################################


def slalom():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global red_contour
    global blue_contour
    global counter
    global rcontour_area
    global s_phase
    global blue1red0
    global straight_angle
    global bcontour_center
    global red_contour
    global rcontour_center
    global Kp
    global blue2red2
    global blue3red3
    global blue4red4
    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image_async()
    #print("image: " + str(image))
    #print(type(image))
    #rc.display.show_color_image(image)
    #blue is 1 red is 0
    image = rc_utils.crop(image, CROP_ROOF[0], CROP_ROOF[1])
    if image is None:
        contour_center = None
        contour_area = 0
        print("image is none")
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        image=image[:,:]
        #for sim
        #image=image[:,100:479]
        if rc.camera.get_width() == 0:
            print("FAIL!")
        else:
            rc.display.show_color_image(image)
        # Find all of the blue contours
       # print("find contours: " + str(rc_utils.find_contours(image, GREEN[0], GREEN[1])))
    rc.display.show_color_image(image)
    Kp=1
    area_to_turn=900
    area_to_straight=100
    area_notStraight=200
    areaToTurnBack=5

    area_to_turnb1=900
    area_to_straightb1=100
    area_notStraightb1=200
    areaToTurnBackb1=5

    area_to_turn3=900
    area_to_straight3=100
    area_notStraight3=200
    areaToTurnBack3=5
    
    area_to_turn4=900
    area_to_straight4=100
    area_notStraight4=200
    areaToTurnBack4=5


    if blue1red0==0:

        red_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, ORANGE[0], ORANGE[1]),300)
        #rc_utils.draw_contour(image, red_contour)
        rc.display.show_color_image(image)
    # rc_utils.draw_circle(image, contour_center)    
        #print(red_contour)
        
        if (red_contour is not None):
            rcontour_area = rc_utils.get_contour_area(red_contour)
            rcontour_center = rc_utils.get_contour_center(red_contour)
            rc_utils.draw_contour(image, red_contour)
            print('red contour')
        else:
            rcontour_area=0
        #rc.drive.set_speed_angle(1,0)
        # print(rc_utils.get_contour_area(red_contour))
    
    #CROP!!!!!
        print("rcontour area",rcontour_area)
        try:
            if(rcontour_area>area_to_turn) and (s_phase!=2):
                rc.drive.set_speed_angle(1,1)
                s_phase=2
            elif(area_to_straight<rcontour_area<area_notStraight and (s_phase!=2)):
                
                straight_angle= Kp*(((float)(rcontour_center[1] )/ 320) * 2 - 1)
                print(straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(rcontour_area<areaToTurnBack) and (s_phase==2):
                rc.drive.set_speed_angle(1,-1)
                blue1red0=1
                s_phase=0
            elif(s_phase==1):
                rc.drive.set_speed_angle(1,0)
            print('sphase',s_phase)
                # rc.drive.set_speed_angle(1,0)  

        except Exception as e:
            print(f"Error: {e}")
    elif(blue1red0==1):

        blue_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),300)
        #rc_utils.draw_contour(image, red_contour)
        rc.display.show_color_image(image)
        #rc_utils.draw_contour(image, red_contour)
    # rc_utils.draw_circle(image, contour_center)    
       # print(blue_contour)
        if (blue_contour is not None):
            bcontour_area = rc_utils.get_contour_area(blue_contour)
            bcontour_center = rc_utils.get_contour_center(blue_contour)
            rc_utils.draw_contour(image, blue_contour)
           # s_phase=0
        else:
            bcontour_area=0
    #rc.drive.set_speed_angle(1,0)
    # print(rc_utils.get_contour_area(red_contour))
   
#CROP!!!!!
        print('blue',bcontour_area)
        try:
            if(bcontour_area>area_to_turnb1) and (s_phase!=2):
                rc.drive.set_speed_angle(1,-1)
                s_phase=2
            elif(area_to_straightb1<bcontour_area<area_notStraightb1 and (s_phase!=2)):
                straight_angle= Kp*(((float)(bcontour_center[1] )/ rc.camera.get_width()) * 2 - 1)                
                print("blue straight angle",straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(bcontour_area<areaToTurnBackb1) and (s_phase==2):
                rc.drive.set_speed_angle(1,1)
                blue2red2=0
                blue1red0=2
                s_phase=0
            elif(s_phase==1):
                rc.drive.set_speed_angle(1,0)
            print('sphase',s_phase)
                # rc.drive.set_speed_angle(1,0)  

        except Exception as e:
            print(f"Error: {e}")  

    elif blue2red2==0:

        red_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, ORANGE[0], ORANGE[1]),200)
        #rc_utils.draw_contour(image, red_contour)
        rc.display.show_color_image(image)
    # rc_utils.draw_circle(image, contour_center)    
        #print(red_contour)
        
        if (red_contour is not None):
            rcontour_area = rc_utils.get_contour_area(red_contour)
            rcontour_center = rc_utils.get_contour_center(red_contour)
            rc_utils.draw_contour(image, red_contour)
            print('red contour')
        else:
            rcontour_area=0
        #rc.drive.set_speed_angle(1,0)
        # print(rc_utils.get_contour_area(red`_contour))
    
    #CROP!!!!!
        print(rcontour_area)
        try:
            if(rcontour_area>area_to_turn3) and (s_phase!=2):
                rc.drive.set_speed_angle(1,1)
                s_phase=2
            elif(area_to_straight3<rcontour_area<area_notStraight3 and (s_phase!=2)):
                
                straight_angle= Kp*(((float)(rcontour_center[1] )/ rc.camera.get_width()) * 2 - 1)
                print(straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(rcontour_area<areaToTurnBack3) and (s_phase==2):
                rc.drive.set_speed_angle(1,-1)
                blue1red0=1
                s_phase=0
            elif(s_phase==1):
                rc.drive.set_speed_angle(1,0)
            print('sphase',s_phase)
                # rc.drive.set_speed_angle(1,0)  

        except Exception as e:
            print(f"Error: {e}")
    elif(blue2red2==1):

        blue_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),300)
        #rc_utils.draw_contour(image, red_contour)
        rc.display.show_color_image(image)
        #rc_utils.draw_contour(image, red_contour)
    # rc_utils.draw_circle(image, contour_center)    
       # print(blue_contour)
        if (blue_contour is not None):
            bcontour_area = rc_utils.get_contour_area(blue_contour)
            bcontour_center = rc_utils.get_contour_center(blue_contour)
            rc_utils.draw_contour(image, blue_contour)
           # s_phase=0
        else:
            bcontour_area=0
    #rc.drive.set_speed_angle(1,0)
    # print(rc_utils.get_contour_area(red_contour))
   
#CROP!!!!!
        print('blue',bcontour_area)
        try:
            if(bcontour_area>area_to_turn4) and (s_phase!=2):
                rc.drive.set_speed_angle(1,-1)
                s_phase=2
            elif(area_to_straight4<bcontour_area<area_notStraight4 and (s_phase!=2)):
                straight_angle= Kp*(((float)(bcontour_center[1] )/ rc.camera.get_width()) * 2 - 1)                
                print("blue straight angle",straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(bcontour_area<areaToTurnBack4) and (s_phase==2):
                rc.drive.set_speed_angle(1,1)
                blue1red0=0
                s_phase=0
            elif(s_phase==1):
                rc.drive.set_speed_angle(1,0)
            print('sphase',s_phase)
                # rc.drive.set_speed_angle(1,0)  

        except Exception as e:
            print(f"Error: {e}")  

###########################################################################################################################################
#                               UPDATE CONTOUR                                                                                            #
###########################################################################################################################################
def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global image 
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

def line_follow():
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
        
        Kp =0.6
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
        # dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
        # lasterr=(contour_center[1])
        # angle=(pangle+dangle)*rc.get_delta_time()
        # print("first angle: ", angle)
        # angle = clamp(angle, 0, rc.camera.get_width())
        # print("angle after clamping: ", angle)
        # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        angle = Kp*pangle
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

def cone_parking():
    pass
def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_state
    global cone_identified
    global contour_area
    # Initialize variables
    speed = 0
    angle = 0
    #cur_state = State.search
    cone_identified = False
    contour_area = 0 

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

    


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    
    global speed
    global angle
    global Kp
    global contour_center
    global cur_state
    global cone_identified
    global contour_area
    global purple_contour
    global red1_contour
    global image 
   # print("in update")
    # Search for contours in the current color image
    update_contour()
    slalom()
    #global image 
    # print(image)
    # purple_contour=rc_utils.get_largest_contour(rc_utils.find_contours(image, LIGHTPURPLE[0], LIGHTPURPLE[1]),30)
    # print("current state", cur_state)
    # if(purple_contour is not None):
    #     purple_contour= rc_utils.get_contour_area(purple_contour)
    #     if(purple_contour>400):
    #         cur_state= State.slalom 
    #         slalom()
    #     red1_contour=rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
    #     if(red1_contour is not None):
    #         red1_area= rc_utils.get_contour_area(red1_contour)
    #         if(red1_area is not None and red1_area>6000):
    #             rc.drive.stop()
    #     else:
    #         red1_area=0
    # else:
    #     cur_state=State.line_follow
    #     line_follow()
    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    #contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
    # if purple_contour > 301:
    #     cone_identified = True
    # print(contour_center)
    # if contour_center is not None:
    #     # Current implementation: bang-bang control (very choppy)
    #     # TODO (warmup): Implement a smoother way to follow the line
        
    #     # Kp = 0.4
    #     # angle = Kp*(contour_center[1]-(rc.camera.get_width()))
    #     #angle = (contour_center[1])
    #     print(contour_center[1])
    #     new_max = 1
    #     new_min = -1
    #     #angle = (angle/(old_max-old_min) * (new_max-new_min)+new_min)
    #    # print("old angle: " + str(angle))
    #    # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), new_min, new_max)
    #     # angle = rc_utils.remap_range(angle, -rc.camera.get_width()/2, rc.camera.get_width()/2, new_min, new_max)
    #     angle = ((contour_center[1]/320)*2-1)
    #     #angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
    #     print("new:",angle)
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
    speed = 0.14

    # if cur_state == State.straight:
    #     if cone_identified:
    #         cur_state = State.stop
    # else:
    #     speed = 0
    #     angle = 0
    #     rc.drive.stop
   # rc.drive.set_speed_angle(speed, angle)

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