#!/usr/bin/env python3

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan

from ...nodes.update import Update


class CalcAvgFrontDist(Update):


    def __init__(self, scan_var_name, dist_var_name, fov):

        super().__init__()

        self.scan_var_name = scan_var_name

        self.dist_var_name = dist_var_name

        self.view_frac = fov/720


    def update_blackboard(self, blackboard:dict) -> str:

        try:

            ranges = np.array(blackboard['/scan'].ranges)

            ranges[ranges <= 0.1] = 0

            n = ranges.size 

            wall_to_left = ranges[0:int(n*self.view_frac)]
            wall_to_right = ranges[n-int(n*self.view_frac):]

            total_avg = (np.average(wall_to_left) + np.average(wall_to_right))/2

            blackboard[self.dist_var_name] = total_avg

            return "success"

        except:

            return "failure"


    