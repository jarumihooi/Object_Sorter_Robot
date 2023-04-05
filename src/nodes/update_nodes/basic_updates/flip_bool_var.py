#!/usr/bin/env python3

import rospy
import numpy as np
import time

from ...nodes.update import Update


'''
Performs a "not" operation on a boolean variable in the blackboard.
'''
class FlipBoolVar(Update):


    def __init__(self, var_name: str):

        super().__init__()

        self.var_name = var_name

        self.var_name = var_name


    def update_blackboard(self, blackboard: dict) -> str:
        try:
            blackboard[self.var_name] = not blackboard[self.var_name]
            return "success"
        except:
            return "failure"


