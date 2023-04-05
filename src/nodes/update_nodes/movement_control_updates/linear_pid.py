#!/usr/bin/env python3

import rospy
import numpy as np 

from ...nodes.update import Update



class LinearPid(Update):

    def __init__(self, linear_pid_var_name, diff_var_name, max_vel, offset=0):

        super().__init__()

        self.linear_pid_var_name = linear_pid_var_name
        self.diff_var_name = diff_var_name
        self.max_vel = max_vel
        self.offset = offset



    def update_blackboard(self, blackboard:dict) -> str:

        try:

            diff = blackboard[self.diff_var_name] - self.offset

            x = 10 * diff

            y = self.max_vel * np.tanh(x)

            blackboard[self.linear_pid_var_name] = y

            return "success"
        
        except:

            return "failure"





