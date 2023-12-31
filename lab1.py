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
    rc.drive.set_speed_angle(0, 0)
    global queue

    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        # TODO (main challenge): Drive in a circle
        drive_circle()
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)

    rc.drive.set_speed_angle(speed, angle)



    # TODO (main challenge): Drive in a square when the B button is pressed

    if rc.controller.was_pressed(rc.controller.Button.B):
         print("Driving in a square...")
        # TODO (main challenge): Drive in a circle
         drive_square()
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)

    rc.drive.set_speed_angle(speed, angle)
    # TODO (main challenge): Drive in a figure eight when the X button is pressed

    if rc.controller.was_pressed(rc.controller.Button.X):
        rc.drive.set_speed_angle(speed, angle)

    # TODO (main challenge): Drive in a shape of your choice when the Y button

    if rc.controller.was_pressed(rc.controller.Button.Y):
        rc.drive.set_speed_angle(speed, angle)

def drive_circle():
    global queue

    CIRCLE_TIME = 6
    BRAKE_TIME = 0.5

    queue.clear()

    queue.append([CIRCLE_TIME, 1, 1])
    queue.append([BRAKE_TIME, -1, 1])

def drive_square():
    global queue

    SQUARE_TIME = 2
    BRAKE_TIME = 0.5
    TURN_TIME = 0.5

    queue.clear()
    for x in range(4):
        queue.append([SQUARE_TIME, 0.5, 0])
        queue.append([BRAKE_TIME, -1, 1])
        queue.append([TURN_TIME, 1, 1])
    
    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
