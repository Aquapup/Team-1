"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
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
from nptyping import NDArray
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
distance=0
scan=np.array()
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
    distance =rc_utils.get_lidar_average_distance(scan, 180)
    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")



# def show_lidar(
#     scan: NDArray[np.float32, np.float32],
#     radius: int = 128,
#     max_range: int = 400,
#     highlighted_samples: List[Tuple[int, int]] = []
# ) -> None:
#     """
#     Displays a visual representation of a LIDAR scan in Jupyter Notebook.
    
#     Args:
#         scan: The LIDAR scan to show.
#         radius: Half of the width and height (in pixels) of the generated image.
#         max_range: The farthest distance to show in the image in cm. Any sample past this range is not shown.
#         highlighted_samples: A list of samples in (angle, distance) format to show as a blue dot.
#     """    
#     # Create a square black image with the requested radius
#     image = np.zeros((2 * radius, 2 * radius, 3), np.uint8, "C")
#     num_samples: int = len(scan)

#     # TODO: Draw a green dot at the center of the image to denote the car
#     # Hint: Use rc_utils.draw_circle
#     CAR_DOT_RADIUS = 2


#     rc_utils.draw_circle(image, (radius, radius), (0, 255, 0), CAR_DOT_RADIUS)
        
#     # TODO: Draw a red pixel for each non-zero sample less than max_range
#     for i in range(len(scan)):
#         if scan[i] < max_range:
#             x= radius+int(math.cos(i/360*math.pi)*scan[i]/3)
#             y= radius+int(math.sin(i/360*math.pi)*scan[i]/3)
#             rc_utils.draw_circle(image, (x, y), (0, 0, 255), 2)
#     # TODO: Draw a light blue dot for each point in highlighted_samples
#     # Hint: Use rc_utils.draw_circle
#     HIGHLIGHT_DOT_RADIUS = 2
#     for i in highlighted_samples:
#         x = int(i[0]/3)
#         y = int(i[1]/3)
#         rc_utils.draw_circle(image, (x, y), (230, 216, 173), 2)

#     # Show the image with Matplotlib
#     plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
#     plt.show()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global angle
    global scan
    # TODO: Follow the wall to the right of the car without hitting anything.
    scan= rc.lidar.get_samples()
    # TBD=0
    print(distance)
    if(rc_utils.get_lidar_average_distance(scan, 180,20) == distance):
        angle= 0
    else:
        angle=scan[0]
        distance = rc_utils.get_lidar_average_distance(scan,180,20)
    

    rc.drive.set_speed_angle(1,angle)


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

        



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
