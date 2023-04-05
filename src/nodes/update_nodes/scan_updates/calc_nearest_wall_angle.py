#!/usr/bin/env python3

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan

from ...nodes.update import Update


'''
Calculates the angle (in degrees) of the nearest wall to the robot according to the
LIDAR scanner.
'''
class CalcNearestWallAngle(Update):


    def __init__(self, scan_var_name, angle_var_name, degrees=False):

        super().__init__()

        self.angle_var_name = angle_var_name

        self.scan_var_name = scan_var_name

        self.degrees = degrees

    
    def update_blackboard(self, blackboard:dict) -> str:

        try:

            ranges = np.array(blackboard[self.scan_var_name].ranges)

            ranges[ranges <= 0.07] = 999

            min_ind = np.argmin(ranges)

            ratio = 360/ranges.size

            min_angle = min_ind*ratio

            if self.degrees:

                blackboard[self.angle_var_name] = min_angle
            
            else:

                blackboard[self.angle_var_name] = min_angle * (3.1415/180)

            return "success"

        except:

            return "failure"




    