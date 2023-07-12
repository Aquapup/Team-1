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
driveCircle = False
driveSquare = False
driveEight = False
driveHeart = False
counter = 0
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
    global driveCircle
    global counter
    if rc.controller.was_pressed(rc.controller.Button.A):
        counter = 0
        driveCircle = True
        print("Driving in a circle...")
        # TODO (main challenge): Drive in a circle
    if driveCircle:
        counter += rc.get_delta_time()

        if counter < 7:
            #turn continously for 7 seconds
            rc.drive.set_speed_angle(1,1)
        else:
            #stop the car
            rc.drive.stop()
            driveCircle = False
                

    # TODO (main challenge): Drive in a square when the B button is pressed
    global driveSquare
    if rc.controller.was_pressed(rc.controller.Button.B):
        counter = 0
        driveSquare = True
        print("Driving in a square...")
    
    if driveSquare:
        counter += rc.get_delta_time()

        if counter < 1: #first line
            rc.drive.set_speed_angle(1,0)
        elif counter < 3:
            rc.drive.set_speed_angle(1,1) # ~right turn
        elif counter < 4: #second line
            rc.drive.set_speed_angle(1,0)
        elif counter < 6:
            rc.drive.set_speed_angle(1,1) # ~right turn
        elif counter <7: #third line
            rc.drive.set_speed_angle(1,0)
        elif counter < 9:
            rc.drive.set_speed_angle(1,1) # ~right turn
        elif counter < 10: #fourth line 
           rc.drive.set_speed_angle(1,0)
        elif counter < 12:
            rc.drive.set_speed_angle(1,1) # ~right turn
        else:
            #finally, stop the car
            rc.drive.stop()
            driveSquare = False 

    # TODO (main ch allenge): Drive in a figure eight when the X button is pressed
    global driveEight
    if rc.controller.was_pressed(rc.controller.Button.X):
        counter = 0
        driveEight = True
        print("Driving in a Figure Eight")

    if driveEight: 
        counter += rc.get_delta_time()

        if counter < 1:
            #move forward
            rc.drive.set_speed_angle(1,0)
        elif counter < 4:
            #turn for the first 8 curve
            rc.drive.set_speed_angle(1,1)
        elif counter < 5:
            #move forward slightly
            rc.drive.set_speed_angle(1,0)
        elif counter < 9:
            #turn for second 8 curve
            rc.drive.set_speed_angle(1,1)
        else:
            #finally, stop the car
            rc.drive.stop()
            driveSquare = False     




    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed

    global driveHeart
    if rc.controller.was_pressed(rc.controller.Button.Y):
        counter = 0
        driveHeart = True
        print("Driving in a heart")
    
    if driveHeart:
        counter += rc.get_delta_time()

        if counter < 0.5:
            #slight turn
            rc.drive.set_speed_angle(1, -1)
        elif counter < 2.5:
            #move forward
            rc.drive.set_speed_angle(1,0)
        elif counter < 3.5: 
            #turn for first heart curve 
            rc.drive.set_speed_angle(1,1)
        elif counter < 4:
            #slight turn
            rc.drive.set_speed_angle(1, -1)
        elif counter < 6.5:
            #second heart curve
            rc.drive.set_speed_angle(1,1)
        elif counter < 8.5:
            #move forward
            rc.drive.set_speed_angle(1,0)
        else:
            #finally, stop the car
            rc.drive.stop()
            driveSquare = False 


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
