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
    wall_follow=1
    right_wall_follow=2
    slalom=3
    parking=4

cur_state: State = State.line_follow



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
#YELLOW = ((20,50,50), (32,255,255))
#GREEN = ((40, 180, 140),(100, 255, 255))

#LIGHTPURPLE((130,),())
# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
s_phase=0
blue1red0= 0
coneindex=0
numscan=360
lasterr=0
########################################################################################
# Functions
########################################################################################
def clamp(value: float, vmin: float, vmax: float) -> float:

    if value < vmin:
        return vmin
    elif value > vmax:
        return vmax
    else:
        return value

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
    global coneindex
    global blue3red3
    global blue4red4
    global numscan
    image = rc.camera.get_color_image()
    scan= rc.lidar.get_samples()
    #depth_image = rc.camera.get_depth_image_async()
    #print("image: " + str(image))
    #print(type(image))
    #rc.display.show_color_image(image)
    #blue is 1 red is 0

    if image is None:
        contour_center = None
        contour_area = 0
        print("image is none")
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        image=image[160:480,:]
        #for sim
        #image=image[:,100:479]
        if rc.camera.get_width() == 0:
            print("FAIL!")
        else:
            rc.display.show_color_image(image)
        # Find all of the blue contours
       # print("find contours: " + str(rc_utils.find_contours(image, GREEN[0], GREEN[1])))
    #rc.display.show_color_image(image)
    Kp=1
    area_to_turn=8000
    counter_turn_to_lidar =0.5
    turn_speed =0.3
    lidar_speed = 0.3
    counter_lidar_to_next= 0.8 #including counter_turn_to_lidar


    area_to_turn_b1=5000
    counter_turn_to_lidar_b1 =0.8
    turn_speed_b1 =0.3
    lidar_speed_b1 = 0.3
    counter_lidar_to_next_b1 = 0.9


    red_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image,ORANGE[0], ORANGE[1]),300)
    #rc_utils.draw_contour(image, red_contour)
    #rc.display.show_color_image(image)
# rc_utils.draw_circle(image, contour_center)    
    #print(red_contour)
    
    if (red_contour is not None):
        rcontour_area = rc_utils.get_contour_area(red_contour)
        rcontour_center = rc_utils.get_contour_center(red_contour)
        rc_utils.draw_contour(image, red_contour)
        print('red contour')
    else:
        rcontour_area=0
        rcontour_center = None   
    blue_contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),300)
    #rc_utils.draw_contour(image, red_contour)
    #rc.display.show_color_image(image)
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
        bcontour_center = None

    if coneindex==0:
    #if blue1red0==0:


        #rc.drive.set_speed_angle(1,0)
        # print(rc_utils.get_contour_area(red_contour))
    
    #CROP!!!!!
        print('red', rcontour_area)
        try:
            if(0<rcontour_area<area_to_turn and ((s_phase==0) or (s_phase==1))):            
                straight_angle= Kp*(((float)(rcontour_center[1] )/ rc.camera.get_width()) * 2 - 1)
                print(straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(rcontour_area>=area_to_turn) and (s_phase==1):
                rc.drive.set_speed_angle(turn_speed,1)
                if (s_phase==1): counter =0
                s_phase=2
                print('counter:',counter)
            elif(counter_lidar_to_next>counter>=counter_turn_to_lidar) and ((s_phase==2) or (s_phase==3)):
                mindis,minangle= rc_utils.get_lidar_closest_point(scan)
                langle=clamp(((float)((minangle-90))) /90,-1,1) #may need chnage - to +
                print('lidar counter:',counter)
                print('lidar minangle:',minangle)
                print('lidar minangle:',mindis)
                #print('lidar scan:',scan)
                print('lidar argmin:',np.argmin(scan))
                print('lidar angle:',langle)
                rc.drive.set_speed_angle(lidar_speed,langle)
                #if (s_phase==2): counter =0
                s_phase=3            
            elif(counter>=counter_lidar_to_next) and (s_phase==3):


                s_phase=0
                coneindex+=1             

            print('sphase',s_phase)
                # rc.drive.set_speed_angle(1,0)  

        except Exception as e:
            print(f"Error: {e}")
    elif(coneindex==1):


    #rc.drive.set_speed_angle(1,0)
    # print(rc_utils.get_contour_area(red_contour))
   
#CROP!!!!!
    
        print('blue',bcontour_area)
        try:
            if(0<bcontour_area<area_to_turn_b1 and ((s_phase==0) or (s_phase==1))):            
                straight_angle= Kp*(((float)(bcontour_center[1] )/ rc.camera.get_width()) * 2 - 1)
                print(straight_angle)
                rc.drive.set_speed_angle(1,straight_angle)
                s_phase=1
            elif(bcontour_area>=area_to_turn_b1) and (s_phase==1):
                rc.drive.set_speed_angle(turn_speed_b1,-1)
                if (s_phase==1): counter =0
                s_phase=2
                print('counter:',counter)
            elif(counter_lidar_to_next_b1>counter>=counter_turn_to_lidar_b1) and ((s_phase==2) or (s_phase==3)):
                mindis,minangle= rc_utils.get_lidar_closest_point(scan)
                langle=clamp(((float)((minangle+90))) /90,-1,1) #may need chnage - to +
                print('lidar counter:',counter)
                print('lidar minangle:',minangle)
                print('lidar minangle:',mindis)
                #print('lidar scan:',scan)
                print('lidar argmin:',np.argmin(scan))
                print('lidar angle:',langle)
                rc.drive.set_speed_angle(lidar_speed_b1,langle)
                #if (s_phase==2): counter =0
                s_phase=3            
            elif(counter>=counter_lidar_to_next_b1) and (s_phase==3):


                s_phase=0
                coneindex-=1             

            print('sphase',s_phase)
            print('counter',counter)
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
         
        contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, BLUE[0], GREEN[1]),50)
        
        print(contours)
        #if (np.size(contours)==0 ):
        if (not (contours is not None)):
           # print("contours: " + str(contours))
            
            contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], BLUE[1]),50)
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
                contours = rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),50)
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


def line_follow():
    global speed
    global angle
    global Kp
    global contour_center
    global lasterr
    global dangle
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
        Kp =0.225
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
        error = (contour_center[1]/320)*2-1
        pangle = Kp*error
       #angle=clamp(Kp*pangle,-1,1)

        dangle= Kd*((error-lasterr)/rc.get_delta_time())
        lasterr=error
        angle=clamp((pangle +dangle),-1,1)
        # print("first angle: ", angle)
        # angle = clamp(angle, 0, rc.camera.get_width())
        # print("angle after clamping: ", angle)
        # angle = rc_utils.remap_range(angle, 0, rc.camera.get_width(), -1, 1)
        #angle = Kp*pangle
        #angle=clamp(angle,0,1)
        #angle -=1
        print("final angle: ", angle)
    speed = 0.138

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
def wall_follow():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global angle
    global scan
    global sharp
    global distance
    global rightdist
    global leftdist
    # TODO: Follow the wall to the right of the car without hitting anything.
    #scan=[]
    scan= rc.lidar.get_samples()
 #   scan=np.array(scan)
    TBD=0
    #print(distance1
    Kp=1
    rightdist= rc_utils.get_lidar_average_distance(scan, 90,40)
    leftdist = rc_utils.get_lidar_average_distance(scan, 270,40)
    print("rightdist", rightdist)
    print("leftdist", leftdist)

    if( rightdist <leftdist+10 and rightdist>leftdist-10):
        angle=0
    else:
        angle= rightdist-leftdist

        #angle= Kp*(2*angle-1010)/990
        angle = angle/(rightdist+leftdist+150)
        # if(angle > 0):
        #     angle = 1
        # else:
        #     angle = -1
        angle = clamp(angle, -1,1)
        print("angle",angle)
        # distance=newdist
        # lidar (10,1000)
    # if(np.argmin(scan))

    #     angle= 0
    # else:
    ### ONE SIDE OF WALL
    Kp=5

    # dist= rc_utils.get_lidar_average_distance(scan,0, 20)
    # sharp=False
    # if(dist<150):
    #     print("hello")
    #     print("min",(np.argmin(scan)))
    #     min= np.argmin(scan)
    #     if(160>min and min<200):
    #         print("hello1")
    #         sharp = True
    #         rc.drive.set_speed_angle(0.1,1)
    #     if(min>520  and min<560):
    #         rc.drive.set_speed_angle(0.1,-1)
    #         sharp = True
        
    # print("angle b4 clamp",np.argmin(scan[0:359]))
    # angle/=90
# if front too close, turn

    # #angle=np.argmax(np.concatenate((scan[540:719],scan[0:179])))/2 -90
    # if(not sharp):
    #     angle=np.argmin(scan[0:359])/2-90
    #     print('old angle',angle)
    #     angle/=90
    #     angle*=Kp
    #     angle=clamp(angle,-1,1)
    #     print(angle)
    # # distance = rc_utils.get_lidar_average_distance(scan,180,20)
    

    rc.drive.set_speed_angle(0.15,angle)



def right_wall_follow():
    global angle
    global scan
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
    

    rc.drive.set_speed_angle(0.2,angle)


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
    global numscan
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
    numscan= rc.lidar.get_num_samples()

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
    #line_follow()

    


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
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    global follow
    follow = False
    if len(markers) > 0:
        id = markers[0].get_id()
    else:
        id = 1000000
    if id!= 1000000:
        print('WALLFOLLOW*********************',id)

    if id == 4 or follow:
        #state = wall following state
        cur_state = State.wall_follow
        follow=True
        wall_follow()
        print('WALLFOLLOW*********************')
    elif id == 0:
        #state = cone slalom state
        cur_state = State.right_wall_follow
        right_wall_follow()
        green_contour= rc_utils.get_largest_contour(rc_utils.find_contours(image, GREEN[0], GREEN[1]),300)
        if ( green_contour is not None):
            cur_state = State.line_follow
            line_follow()
    elif id == 1:
        #state = stop state
        #cur_state: State = State.slalom
        slalom()
    elif id==3:
        #cur_state: State = State.parking
        rc.drive.stop()
    else:
        cur_state== State.line_follow
        line_follow()

    #update_contour()
    # scan= rc.lidar.get_samples()
    # if(rc_utils.get_lidar_average_distance(scan, 0, 1))<200:

    # purple_contour=rc_utils.get_largest_contour(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),30)
    # if(purple_contour is not None):
    #     cur_state= 1 
    #     slalom()
    #     red1_contour=rc_utils.get_largest_contour(rc_utils.find_contours(image, RED[0], RED[1]),30)
    #     red1_area= rc_utils.get_contour_area(red1_contour)
    #     if(red1_area>6000):
    #         rc.drive.stop()
    # else:
    #     cur_state=0
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