#!/usr/bin/env python3

import rospy
import numpy as np 
import math
from nav_msgs.msg import Odometry

from ...nodes.update import Update





class GetRotation(Update):


    def __init__(self, odom_var_name, rotation_var_name, degrees=True):

        super().__init__()

        self.odom_var_name = odom_var_name
        self.rotation_var_name = rotation_var_name

        if degrees:
            # Conversion between radians and degrees
            self.mult = 180/3.1415
        else:
            self.mult = 1


    def update_blackboard(self, blackboard:dict) -> str:

        try:

            # Getting orientation from msg
            ori = blackboard[self.odom_var_name].pose.pose.orientation
            x = ori.x
            y = ori.y
            z = ori.z
            w = ori.w

            # Converting the quaternion values to radians, and then to degrees if the user specifies
            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            rot = math.atan2(t3, t4)

            if rot < 0:
                rot += 2*3.1415

            blackboard[self.rotation_var_name] = rot * self.mult

            return "success"

        except:

            return "failure"
