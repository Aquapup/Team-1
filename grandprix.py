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
    overpass = 1
    graveyardLineFollow = 2
    canyonMazeWall = 3
    goFast = 4
    slalom = 5
    rampLane = 6
    hazardWall = 7
    brickWall = 8

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


CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_WIDTHL = ((rc.camera.get_height(),rc.camera.get_width()),(0,160))
CROP_WIDTHR = ((rc.camera.get_height(),rc.camera.get_width()),(480,640))
CROP_FLOOR = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_ROOF = ((60, 0), (rc.camera.get_height(), rc.camera.get_width()))
# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 80, 80), (119, 255, 255))  # The HSV range for the color blue

# TODO (challenge 1): add HSV ranges for other colors
#RED = ((165, 121, 60),(179,255,255))
GREEN = ((89.5, 180, 50),(179, 245, 245))
PURPLE = ((65, 140, 140), (108.5, 255, 255))
ORANGE= ((0,209,220),(28,255,255))
# YELLOW = (())
BLUE = ((90, 90, 140),(95, 255, 255))
# TODO (challenge 1): add HSV ranges for other colors
RED = ((130,40,160),(179,255,255))
# PINK = ((130,0,0),(180,255,150))
# GREEN = ((40,100,120),(70,200,255))
#GREEN = ((26,56,119),(81,130,213))

YELLOW = ((20,70,50), (32,255,255))

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
s_phase=0
blue1red0= 0
coneindex=0
numscan=360
lasterr=0

#functions
def clamp(value: float, vmin: float, vmax: float) -> float:

    if value < vmin:
        return vmin
    elif value > vmax:
        return vmax
    else:
        return value

def line_follow(speedVar = 0.15):

    global speed
    global angle
    global Kp
    global contour_center
    global lasterr
    speed = speedVar
   # print("in update")
    # Search for contours in the current color image
    update_contour()
    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
   # print(contour_center)
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
        pangle = ((contour_center[1]/320)*2-1)
       #angle=clamp(Kp*pangle,-1,1)
        dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
        lasterr=(contour_center[1])
        angle=clamp(Kp*pangle+Kd*dangle)
        # print("first angle: ", angle)
        # angle = clamp(angle, 0, rc.camera.get_width())
        # print("angle after clamping: ", angle)
        # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        #angle = Kp*pangle
        #angle=clamp(angle,0,1)
        #angle -=1
        print("final angle: ", angle)
    #speed = 0.15

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

def wall_follow(speedVar = 0.2):
    global angle
    global scan
    global speed
    speed = speedVar
    # TODO: Follow the wall to the right of the car without hitting anything.
    #scan=[]
    scan= rc.lidar.get_samples()
 #   scan=np.array(scan)
    # TBD=0
    #print(distance)
    # if(rc_utils.get_lidar_average_distance(scan, 180,20) == distance):
    # if(np.argmin(scan))
    #     angle= 0
    # else:
    ### ONE SIDE OF WALL
    Kp=5
    angle=np.argmin(scan[0:359])/2-90
    # print("angle b4 clamp",np.argmin(scan[0:359]))
    # angle/=90
# if front too close, turn

    #angle=np.argmax(np.concatenate((scan[540:719],scan[0:179])))/2 -90
    print('old angle',angle)
    angle/=90
    angle*=Kp
    angle=clamp(angle,-1,1)
    print(angle)
    # distance = rc_utils.get_lidar_average_distance(scan,180,20)


    rc.drive.set_speed_angle(speed,angle)


    # p= scan
    # #k= [-1,-2,-3,5,-3,-2,-1]
    # k=[-3,-3,5,-3,-3]
    # d=[]

    # stdev= np.std(p)
    # clusters={}
    # cluster_index=np.zeros(len(scan))
    # #cluster=[i+j]
    # cluster_num=0
    # #Ci=[]

    # m=len(k)
    # print(len(p))
    # for l in range(1,len(scan)):
    #     d.append(p[l]-p[l-1])
    # print(len(d))
    # for i in range(1,len(p)-1):
    #     total=0
    #     for j in range(-(m-1)//2 , (m-1)//2):
    #     #  print(i+j)
    #         if not ((i+j)<0 or (i+j)>len(scan)-2):
    #             total=total+ d[i+j]*k[(j+(int)((m+1)/2))]



    #     if total>stdev:
    #         clusters[total]=[p[i]]
    #         C=total
    #     else:
    #         clusters[C]+=[scan[i]]
    #     print(clusters)

def safety_stop():
    global counter
    global angle
    #counter = 0

    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    scan = rc.lidar.get_samples()

    if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 50:

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

def lane_follow(speedVar = 0.16):
    global speed
    global angle
    global Kp
    global contour_center
    global lasterr
    speed = speedVar
   # print("in update")
    # Search for contours in the current color image
    update_contour()
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
            pangle = ((contour_center[1]/160)*2-1)
           #angle=clamp(Kp*pangle,-1,1)
            dangle= Kd*((contour_center[1]-lasterr)/rc.get_delta_time())
            lasterr=(contour_center[1])
            angle= (clamp(Kp*pangle+Kd*dangle,-1,1))
            rc.drive.set_speed_angle(speed, angle)
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

#def slalom():

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

        contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, BLUE[0], BLUE[1]),50)

        print(contours)
        #if (np.size(contours)==0 ):
        if (not (contours is not None)):
           # print("contours: " + str(contours))

            contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),50)
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
                contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], GREEN[1]),50)
                # if rc_utils.get_contour_area(contours) < threshold:
                #     contours = None
                print("found green")
            if(not (contours is not None)):
                contours= rc_utils.get_largest_contour(rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),50)
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
def ramp():
    scan= rc.lidar.get_samples()
    if(rc.get_lidar_average_distance(scan,0,20)):
        rc.drive.set_speed_angle(0.3, 0)
    else:
        rc.drive.set_speed_angle(0.1,0)
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
    global counter
    global cone_identified
    global contour_area
    global purple_contour
    global red1_contour
   # print("in update")
    # Search for contours in the current color image
    counter+= rc.get_delta_time()




    global ar_markers
    global angle
    image = rc.camera.get_color_image()
    #slalom()
    markers = rc_utils.get_ar_markers(image)
    global follow
    follow = False
    if len(markers) > 0:
        id = markers[0].get_id()
    else:
        id = 1000000

    if id == 1:
        print("in overpass")
        cur_state = State.overpass
        scan = rc.lidar.get_samples()

        if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 50:

            counter += rc.get_delta_time()
            #print("counter: " + str(counter))
            if counter < 2:
                rc.drive.set_speed_angle(-1, angle)
                #rc.drive.set_speed_angle(-1, 0)
            else:
                counter = 0
                rc.drive.stop()
        else:
            wall_follow(0.3)

    #rc.display.show_lidar(scan)
        # wall_follow()
        # safety_stop()
    elif id == 2:
        print("in graveward")
        cur_state = State.graveyardLineFollow
        scan = rc.lidar.get_samples()

        if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 50:

            counter += rc.get_delta_time()
            #print("counter: " + str(counter))
            if counter < 2:
                rc.drive.set_speed_angle(-1, angle)
                #rc.drive.set_speed_angle(-1, 0)
            else:
                counter = 0
                rc.drive.stop()
        else:
            line_follow()

        #rc.display.show_lidar(scan)
    elif id == 3:
        print("in canyon maze")
        cur_state = State.canyonMazeWall
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
        #     wall_follow()

        # rc.display.show_lidar(scan)
        wall_follow()
    elif id == 4:
        print("in go fast")
        cur_state = State.goFast
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

        # rc.display.show_lidar(scan)
        wall_follow(0.3)
    elif id == 5:
        print("in cone slalom")
        cur_state = State.slalom
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

        # rc.display.show_lidar(scan)
        wall_follow(0.3)

    elif id == 6:
        print("in ramp lanes")
        cur_state = State.rampLane
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

        # rc.display.show_lidar(scan)
        lane_follow(0.3)

    elif id == 7:
        print("in hazard valley")
        cur_state = State.hazardWall
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

        # rc.display.show_lidar(scan)

        #need to update with safety stop !!!!!
        line_follow(0.3)

    elif id == 8:
        print("in brick wall")
        cur_state = State.brickWall
        scan = rc.lidar.get_samples()

        if (rc_utils.get_lidar_average_distance(scan, 0, 1)) < 50:

            counter += rc.get_delta_time()
            #print("counter: " + str(counter))
            if counter < 2:
                rc.drive.set_speed_angle(-1, angle)
                #rc.drive.set_speed_angle(-1, 0)
            else:
                counter = 0
                rc.drive.stop()
        else:
            wall_follow(0.2)

        rc.display.show_lidar(scan)

    update_contour()
    scan= rc.lidar.get_samples()
    if(rc_utils.get_lidar_average_distance(scan, 0, 1))<200:

    purple_contour =rc_utils.get_largest_contour(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),30)
    if(purple_contour is not None):
        cur_state= 1
        slalom()
        red1_contour=rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
        red1_area= rc_utils.get_contour_area(red1_contour)
        if(red1_area>6000):
            rc.drive.stop()
    else:
        cur_state=0
    #Choose an angle based on contour_center
    #If we could not find a contour, keep the previous angle
    contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
    if purple_contour > 301:
        cone_identified = True
    print(contour_center)
    if contour_center is not None:
        # Current implementation: bang-bang control (very choppy)
        # TODO (warmup): Implement a smoother way to follow the line

        # Kp = 0.4
        # angle = Kp*(contour_center[1]-(rc.camera.get_width()))
        #angle = (contour_center[1])
        print(contour_center[1])
        new_max = 1
        new_min = -1
        #angle = (angle/(old_max-old_min) * (new_max-new_min)+new_min)
       # print("old angle: " + str(angle))
       # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), new_min, new_max)
        # angle = rc_utils.remap_range(angle, -rc.camera.get_width()/2, rc.camera.get_width()/2, new_min, new_max)
        angle = ((contour_center[1]/320)*2-1)
        #angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        print("new:",angle)
        if contour_center[1] < (rc.camera.get_width()/ 2):
            angle = Kp*abs(contour_center[1]-rc.camera.get_width())
        else:
            angle = 1

    # Use the triggers to control the car's speed
    # forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # # #forwardSpeed = rc.controller.is_down(rc.controller.)
    # # #backSpeed = rc.controller.is_down(rc.controller.Button.B)
    # speed = forwardSpeed - backSpeed
    speed = 0.2

    # if cur_state == State.straight:
    #     if cone_identified:
    #         cur_state = State.stop
    # else:
    #     speed = 0
    #     angle = 0
    #     rc.drive.stop
   # rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    update_contour()
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
