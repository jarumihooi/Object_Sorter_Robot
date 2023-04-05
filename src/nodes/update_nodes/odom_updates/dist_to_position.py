#!/usr/bin/env python3

import rospy
import numpy as np 
import math
from nav_msgs.msg import Odometry

from ...nodes.update import Update


class DistToPosition(Update):

    def __init__(self, goal_position_var_name, curr_position_var_name, dist_var_name):

        super().__init__()

        self.goal_position_var_name = goal_position_var_name

        self.curr_position_var_name = curr_position_var_name

        self.dist_var_name = dist_var_name


    def update_blackboard(self, blackboard:dict) -> str:


        curr_pos = blackboard[self.curr_position_var_name]

        goal_pos = blackboard[self.goal_position_var_name]

        dist = np.sqrt((goal_pos[0]-curr_pos[0])**2 + (goal_pos[1]-curr_pos[1])**2)

        blackboard[self.dist_var_name] = dist

        return "success"