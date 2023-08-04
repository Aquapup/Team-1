"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

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
    overpass = 1
    graveyardLineFollow = 2 
    canyonMazeWall = 3
    goFast = 4
    slalom = 5
    Search = 6
    orangeCurve = 7
    purpleCurve = 8
    rampLane = 9
    hazardWall = 10
    brickWall = 11

    # line_follow=0
    # wall_follow=1
    # right_wall_follow=2
    # slalom=3
    # parking=4


cur_state: State = State.line_follow
#variables 
global counter 
counter = 0 
global rcontour_area
rcotour_area=0
global blue2red2 
blue2red2 =0
global blue3red3
blue3red3 =0
global blue4red4
blue4red4 =0



rc = racecar_core.create_racecar()


distance=0
scan=[]
scan=np.array(scan)
lastd=0
total_lidar_pts=rc.lidar.get_num_samples()


CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_WIDTHL = ((rc.camera.get_height(),rc.camera.get_width()),(0,160))
CROP_WIDTHR = ((rc.camera.get_height(),rc.camera.get_width()),(480,640))
CROP_FLOOR = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_ROOF = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))

#HSV

ORANGE = ((0,160,160),(16,255,255))
PURPLE = ((127, 83, 100), (160, 255, 255))

GREEN = ((89.5, 180, 50),(179, 245, 245))
BLUE = ((90, 90, 140),(95, 255, 255))
RED = ((130,40,160),(179,255,255)) 
YELLOW = ((20,70,50), (32,255,255))

speed = 0.0  
angle = 0.0  
contour_center = None 
contour_area = 0
s_phase=0
blue1red0= 0
coneindex=0
numscan=360
lasterr=0
coneColor=None
MIN_CONTOUR_AREA = 40

coneColor = None
#functions 
def clamp(value: float, vmin: float, vmax: float) -> float:

    if value < vmin:
        return vmin
    elif value > vmax:
        return vmax
    else:
        return value
def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0
    coneColor=None
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
def ramp():
    scan= rc.lidar.get_samples()
    if(rc.get_lidar_average_distance(scan,0,20)):
        rc.drive.set_speed_angle(0.3, 0)
    else:
        right_wall_follow()

def line_follow():

    global speed
    global angle
    global Kp
    global contour_center
    global lasterr
    global dangle

    update_contour()
   
    if contour_center is not None:
        Kp =0.225
        Kd= 0.2
        setpoint = rc.camera.get_width()/2
       
        error = (contour_center[1]/320)*2-1
        pangle = Kp*error
        dangle= Kd*((error-lasterr)/rc.get_delta_time())
        lasterr=error

        angle=clamp((pangle +dangle),-1,1)
       
        print("final angle: ", angle)

    speed = 0.138
    rc.drive.set_speed_angle(speed, angle)

def right_wall_follow(setpoint=40,straight_speedp=0.165):
    global angle
    global scan
    global sharp
    global lastd
    global total_lidar_pts
   
    scan= rc.lidar.get_samples()

    Kp=0.1
    Kd=0.3
    dset=setpoint
    sharp_front_distance=125
    straight_speed=straight_speedp
    turn_speed =0.183
    #
    remapvalue= dset
    max_dis_gap=150
    rightdist= rc_utils.get_lidar_average_distance(scan,90, 40)
    frontdist = rc_utils.get_lidar_average_distance(scan,0, 40)
    print('right distance',rightdist)
    print('front distance',frontdist)

    sharp=False
    if(frontdist<sharp_front_distance):
        print("min",(np.argmin(scan)))
       
        shift_scan=np.roll(scan,total_lidar_pts//4)
        maxangle=np.argmax(shift_scan[0:total_lidar_pts//2]) #only look at front
        print("max",(maxangle))

        if (maxangle<(rc.lidar.get_num_samples()//4)):
            rc.drive.set_speed_angle(turn_speed,-1)  
        else:
            rc.drive.set_speed_angle(turn_speed,1)  
        sharp = True                     
   
    if(not sharp):
        angle= Kp * (rightdist - dset)+Kd*(rightdist-lastd)
        lastd= rightdist
        angle/=remapvalue
        print('old angle',angle)
        angle=clamp(angle,-1,1)
        print('new angle:',angle)    

        rc.drive.set_speed_angle(straight_speed,angle)

def left_wall_follow(setpoint=40,straight_speedp=0.165):
    global angle
    global scan
    global sharp
    global lastd
    global total_lidar_pts
   
    scan= rc.lidar.get_samples()

    Kp=0.1
    Kd=0.3
    dset=setpoint
    sharp_front_distance=125
    straight_speed=straight_speedp
    turn_speed =0.183

    remapvalue= dset
    max_dis_gap=150
    rightdist= rc_utils.get_lidar_average_distance(scan,270, 40)
    frontdist = rc_utils.get_lidar_average_distance(scan,0, 40)
    print('right distance',rightdist)
    print('front distance',frontdist)

    sharp=False
    if(frontdist<sharp_front_distance):
        print("min",(np.argmin(scan)))
       
        shift_scan=np.roll(scan,total_lidar_pts//4)
        maxangle=np.argmax(shift_scan[0:total_lidar_pts//2]) #only look at front
        print("max",(maxangle))

        if (maxangle<(rc.lidar.get_num_samples()//4)):
            rc.drive.set_speed_angle(turn_speed,-1)  
        else:
            rc.drive.set_speed_angle(turn_speed,1)  
        sharp = True                     
   
    if(not sharp):
        angle= Kp * (rightdist - dset)+Kd*(rightdist-lastd)
        lastd= rightdist
        angle/=remapvalue
        print('old angle',angle)
        angle=clamp(angle,-1,1)
        print('new angle:',angle)    

        rc.drive.set_speed_angle(straight_speed,angle)

  

def safety_stop():
    global counter 
    global angle 
    
    scan = rc.lidar.get_samples()
    
    if (rc_utils.get_lidar_average_distance(scan, 0, 20)) < 20:
        
        counter += rc.get_delta_time()
        print("counter: " + str(counter)) 
        if counter < 2: 
            rc.drive.set_speed_angle(-1, angle)
        else:
            counter = 0 
            rc.drive.stop()
    else:
        rc.drive.set_speed_angle(1, 0)

    rc.display.show_lidar(scan)

def lane_follow(speedVar = 0.2):
    global speed
    global angle
    global Kp
    global contour_center
    global lasterr
    speed = speedVar
    def update_contourlanefollow():
        """
        Finds contours in the current color image and uses them to update contour_center
        and contour_area
        """
        global contour_center
        global contour_area
        global cropped_image

        image = rc.camera.get_color_image()
        cropped_image = image[100:480, 0:400]
        #print("image: " + str(image))
        #print(type(image))
        #rc.display.show_color_image(image)
        # threshold = 50
        # if rc_utils.get_contour_area(contours) < threshold: 
        #         contours = None 
        if cropped_image is None:
            contour_center = None
            contour_area = 0
            print("image is none")
        else:
            # TODO (challenge 1): Search for multiple tape colors with a priority order
            # (currently we only search for blue)

            # Crop the image to the floor directly in front of the car
            # image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
            # image = rc_utils.crop(image, CROP_ROOF[0], CROP_ROOF[1])
            #image = image[100:320,::]
            if rc.camera.get_width() == 0:
                print("FAIL!")
            else:
                rc.display.show_color_image(cropped_image)
            # Find all of the blue contours
        # print("find contours: " + str(rc_utils.find_contours(image, GREEN[0], GREEN[1])))
            # if(np.size(rc_utils.find_contours(image, RED[0], RED[1]))!=0):
            #     contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
            # else:
            
        # print("found green")
            #print("contours: " + str(contours))
            
            contours = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image, BLUE[0], BLUE[1]),50)
            
            print(contours)
            #if (np.size(contours)==0 ):
            if (not (contours is not None)):
            # print("contours: " + str(contours))
                
                contours = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image, RED[0], RED[1]),50)
                print("found red")
                # if rc_utils.get_contour_area(contours) < threshold: 
                #     contours = None 
                #print("contours: " + str(contours))
            #  if(contours.all()!=None):
            #     print("found blue")
        #     print(contours)
                if (not (contours is not None)):
                #   print("found red")
                    #print("contours: " + str(contours))
                    contours = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image, GREEN[0], GREEN[1]),50)
                    # if rc_utils.get_contour_area(contours) < threshold: 
                    #     contours = None 
                    print("found green")
                if(not (contours is not None)):
                    contours= rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image, YELLOW[0], YELLOW[1]),50)
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
                rc_utils.draw_contour(cropped_image, contours)
                rc.display.show_color_image(cropped_image)
                rc_utils.draw_circle(cropped_image, contour_center)

            else:
                contour_center = None
                contour_area = 0

            # Display the image to the screen
            rc.display.show_color_image(cropped_image)
   # print("in update")
    # Search for contours in the current color image
    update_contourlanefollow()
    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
   # print(contour_center)
    scan= rc.lidar.get_samples()
    if(rc.get_lidar_average_distance(scan,0,20)):
        rc.drive.set_speed_angle(0.3, 0)
    else:

        if contour_center is not None:
                # Current implementation: bang-bang control (very choppy)
            # TODO (warmup): Implement a smoother way to follow the line
            #Kp = 0.4
            #angle = Kp*(contour_center[1]-(rc.camera.get_width()))
            #angle = (contour_center[1])
            Kp =0.3
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
            #rc_utils.get_lidar_average_distance(scan, 180,20) >= (leftDistance-2) and rc_utils.get_lidar_average_distance(scan, 180,20) <= (leftDistance+2)
            #angle = rc_utils.remap_range(angle,  Kp*-setpoint, Kp*setpoint, -1, 1)
            # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
            pangle = ((contour_center[1]/100)*2-1)
            #angle=clamp(Kp*pangle,-1,1)
            dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
            lasterr=(contour_center[1])
            print("angle before clamp: " + str(angle))
            angle = Kp*pangle+Kd*dangle
            print("angle before clamp: " + str(angle))
            angle= (clamp(Kp*pangle+Kd*dangle,-1,1))

          # print("first angle: ", angle)
            # angle = clamp(angle, 0, rc.camera.get_width())
            # print("angle after clamping: ", angle)
            # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
            #angle = Kp*pangle
            #angle=clamp(angle,0,1)
            #angle -=1
            print("final angle: ", angle)
        else:
             rc.drive.set_speed_angle(0.1,0)
    #speed = 1



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

def slalom():
    def update_contour():
        """
        Finds contours in the current color image and uses them to update contour_center
        and contour_area
        """
        global contour_center
        global contour_area
        global cur_state
        global coneColor
        global contour_center
        
        global coneColor

        image = rc.camera.get_color_image()

        if image is None:
            contour_center = None
            contour_area = 0
        else:

            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

            # Find all of the orange and purple contours
            orangeContours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])      
            purpleContours = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])

            # Select the largest orange and purple contour
            orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
            purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)
            

            if orangeContour is None and purpleContour is None:
                contour = None
            elif orangeContour is None:
                contour = purpleContour
                coneColor = "purple"
            elif purpleContour is None:
                contour = orangeContour
                coneColor = "orange"
            else:
                if rc_utils.get_contour_area(orangeContour) > rc_utils.get_contour_area(purpleContour):
                    contour = orangeContour
                    coneColor = "orange"
                else:
                    contour = purpleContour
                    coneColor = "purple"
            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the imagex
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)

            else:
                contour_center = None
                contour_area = 0
            # Display the image to the screen
            rc.display.show_color_image(image)
            #Speed and SPEEDA

            #Add start funtion
    def purpleCurve(contour_center, contour_area):
        if contour_center is None:
            rc.drive.set_speed_angle(speeda*0.9, 0.146)
            print("autoPurpturn")
        else:
            TURN_ANGLE = ((contour_center[1] - 1500) / 320) - 0.08
            TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -maxa, maxa)
            
            if -maxa < TURN_ANGLE < maxa:
                TURN_ANGLE = 0
            rc.drive.set_speed_angle(speeda,TURN_ANGLE)

    def orangeCurve(contour_center, contour_area):
        if contour_center is None:
            rc.drive.set_speed_angle(speeda*0.9, -0.146)
            print("autoOrangturn")
        else:
            TURN_ANGLE = ((contour_center[1] - 100) / 320) + 0.08
            TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -maxa, maxa)
            TURN_ANGLE = rc_utils.remap_range(TURN_ANGLE,1,-1,-1,1)
            if -maxa < TURN_ANGLE < maxa:
                TURN_ANGLE = 0
            rc.drive.set_speed_angle(speeda,TURN_ANGLE)
    global cur_state
    global coneColor
    global contour_center
    global contour_area

    update_contour()
    update_slow()
    print(cur_state)
    speeda=.15
    maxa=0.128
    if coneColor == "orange":
        cur_state = State.orangeCurve
    elif coneColor == "purple":
        cur_state = State.purpleCurve
    
    if cur_state == State.orangeCurve:
        orangeCurve(contour_center, contour_area)
    elif cur_state == State.purpleCurve:
        purpleCurve(contour_center, contour_area)
    elif cur_state == State.Search:
        rc.drive.set_speed_angle(0.148,0)
def update_contour():
    
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()
  
    if image is None:
        contour_center = None
        contour_area = 0
        print("image is none")
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        image = rc_utils.crop(image, CROP_ROOF[0], CROP_ROOF[1])
        #image = image[100:320,::]
        if rc.camera.get_width() == 0:
            print("FAIL!")
        else:
            rc.display.show_color_image(image)
      
        contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, BLUE[0], BLUE[1]),50)
        
        print(contours)
        if (not (contours is not None)):            
            contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),50)
            if (not (contours is not None)):
                contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], GREEN[1]),50)
                print("found green")
            if(not (contours is not None)):
                contours= rc_utils.get_largest_contour(rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),50)

        if contours is not None:
            contour_center = rc_utils.get_contour_center(contours)
            contour_area = rc_utils.get_contour_area(contours)
            rc_utils.draw_contour(image, contours)
            rc.display.show_color_image(image)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0
        rc.display.show_color_image(image)

def update():

    global speed
    global angle
    global Kp
    global contour_center
    global cur_state
    global counter
    global cone_identified
    global contour_area
    global purple_contour
    global red1_contour
    global ar_markers
    global angle 
    global follow
   
    counter+= rc.get_delta_time()
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    follow = False

    

    if len(markers) > 0:
        id = markers[0].get_id()
    else:
        id = 1000000

    cur_state = State.goFast
    if cur_state == State.goFast:
        print("in gofast")
        right_wall_follow()
        if id == 1:
            cur_state = State.overpass

    elif cur_state == State.overpass:
        print("in overpass")
        ramp()

        # scan = rc.lidar.get_samples()
        # if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 50:
            
        #     counter += rc.get_delta_time()
        #     #print("counter: " + str(counter)) 
        #     if counter < 2: 
        #         rc.drive.set_speed_angle(-1, angle)
        #         #rc.drive.set_speed_angle(-1, 0)
        #     else:
        #         counter = 0 
        #         rc.drive.stop()
        # else:
        #     wall_follow(0.3)

        if id == 2:
            cur_state = State.graveyardLineFollow

  
    elif cur_state == State.graveyardLineFollow: 
        safety_stop()
        print("in graveward")
        line_follow()

        if id == 3:
            cur_state = State.canyonMazeWall

    elif cur_state == State.canyonMazeWall: 
        print("in canyon maze")
        right_wall_follow(40,0.138)
        if id == 4:
            cur_state = State.goFast

    elif cur_state == State.goFast: 
        safety_stop()
        print("in go fast")
        right_wall_follow(40,0.17)

        if id == 5:
            cur_state = State.slalom
    elif cur_state == State.slalom: 
        print("in cone slalom")
        slalom()

        if id == 6:
            cur_state = State.rampLane
    
    elif cur_state == State.rampLane: 
        safety_stop()
        cropped_image = image[100:480, 0:400]
        print("in ramp lanes")
        
        lane_follow(0.14)
        if id == 7:
            cur_state = State.hazardWall
    
    elif cur_state == State.hazardWall: 
        print("in hazard valley")
        cur_state = State.hazardWall
        right_wall_follow(40,0.138)
    elif cur_state == State.brickWall:
        print("in brick wall")
        right_wall_follow(27,0.138)
        
    
    

    

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

    

    
    


