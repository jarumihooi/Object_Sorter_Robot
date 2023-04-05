#!/usr/bin/env python3

import rospy
import numpy as np 
import math
from nav_msgs.msg import Odometry

from ...nodes.update import Update


class AngleToPosition(Update):

    def __init__(self, goal_position_var_name, curr_position_var_name, goal_rotation_var_name, rotation_var_name):

        super().__init__()

        self.goal_position_var_name = goal_position_var_name
        self.curr_position_var_name = curr_position_var_name
        self.goal_rotation_var_name = goal_rotation_var_name
        self.rotation_var_name = rotation_var_name

    def update_blackboard(self, blackboard:dict) -> str:

        goal_pos = blackboard[self.goal_position_var_name]
        curr_pos = blackboard[self.curr_position_var_name]
        rot = blackboard[self.rotation_var_name]

        curr_vect = [np.cos(rot), np.sin(rot)]

        goal_vect = [goal_pos[0]-(0-curr_pos[0]), goal_pos[1]-(0-curr_pos[1])]

        goal_angle = np.arctan(goal_vect[1]/goal_vect[0])

        if goal_vect[1] < 0:
            goal_angle = np.pi + goal_angle

        

        blackboard[self.goal_rotation_var_name] = goal_angle

        return 'success'

