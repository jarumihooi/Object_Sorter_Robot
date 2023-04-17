#!/usr/bin/env python3

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan

from ...nodes.update import Update



'''
Calculates the closest distance to the robot according to the
LIDAR scanner.
'''
class CalcNearestDist(Update):


    def __init__(self, scan_var_name, dist_var_name):

        super().__init__()

        self.dist_var_name = dist_var_name

        self.scan_var_name = scan_var_name

    
    def update_blackboard(self, blackboard:dict) -> str:

        try:

            ranges = np.array(blackboard[self.scan_var_name].ranges)

            ranges[ranges <= 0.07] = 999

            min_dist = np.min(ranges)

            blackboard[self.dist_var_name] = min_dist

            return "success"

        except:

            return "failure"

    