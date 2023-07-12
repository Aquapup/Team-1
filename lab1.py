"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
counter=0
isDrivingCircle=False
isDrivingSquare=False
isDrivingFigure8=False
isDrivingTri= False
rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    #rc.drive.set_speed_angle(0, 0)
    global counter
    global isDrivingCircle
    global isDrivingSquare
    global isDrivingFigure8
    global isDrivingTri
    if rc.controller.was_pressed(rc.controller.Button.A):

        print("Driving in a circle...")
        # TODO (main challenge): Drive in a circle
    # First go straight for 2 seconds and then take 1.2 second to make a 90 degree turn and repeated this 4 times to draw a square.  
        isDrivingCircle = True

    if isDrivingCircle:
        counter += rc.get_delta_time()
        print(counter)
        if(counter<11.6):
            rc.drive.set_speed_angle(0.5,1)
        else:
            rc.drive.stop()
            isDriving = False

                #rc.drive.set_speed_angle(1,0)
            # else:
         # Otherwise, stop the car

    # TODO (main challenge): Drive in a square when the B button is pressed
    if(rc.controller.was_pressed(rc.controller.Button.B)):
        print("Driving in a Square... ")
        isDrivingSquare=True
    
    if isDrivingSquare:
        counter+= rc.get_delta_time()
    #     print(counter)
        ts=2
        tt=1.65

    #     global i
    #     for i in range(0,3):

    #         rc.drive.set_speed_angle(0.1,1)
    #         rc.drive.stop()
    #         rc.drive.set_speed_angle(1,0)
        if(counter<ts):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<ts+tt):
            rc.drive.set_speed_angle(1,1)
        elif(counter<2*ts+tt):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<2*ts+2*tt):
            rc.drive.set_speed_angle(1,1)
        elif(counter<3*ts+2*tt):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<3*ts+3*tt):
            rc.drive.set_speed_angle(1,1)
        elif(counter<4*ts+3*tt):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<4*ts+4*tt):
            rc.drive.set_speed_angle(1,1)
        else:
            rc.drive.stop()
    # TODO (main challenge): Drive in a figure eight when the X button is pressed
    if(rc.controller.was_pressed(rc.controller.Button.X)):
        print("Driving in a Figure 8")
        isDrivingFigure8= True
    if isDrivingFigure8:
        counter+= rc.get_delta_time()
        if(counter<11.6):
            rc.drive.set_speed_angle(0.5,1)
        elif(counter<23.2):
            rc.drive.set_speed_angle(0.5,-1)
    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed

    if(rc.controller.was_pressed(rc.controller.Button.Y)):
        print("Driving in a Triangle")
        isDrivingTri=True
        global trs
        global trts
        trs=2
        trts=2


    if isDrivingTri:
        counter+= rc.get_delta_time()
        if(counter<trs):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<trs+trts):
            rc.drive.set_speed_angle(1,1)
        elif(counter<2*trs+trts):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<2*trs+2*trts):
            rc.drive.set_speed_angle(1,1)
        elif(counter<3*trs+2*trts):
            rc.drive.set_speed_angle(0.5,0)
        elif(counter<3*trs+3*trts):
            rc.drive.set_speed_angle(1,1)
        else:
            rc.drive.stop()
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
