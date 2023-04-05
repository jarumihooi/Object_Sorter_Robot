#!/usr/bin/env python3

import rospy
import numpy as np
import time

from ...nodes.update import Update



'''
Increments a number variable in the blackboard by an amount specified at initialization.
'''
class IncrementVar(Update):


    def __init__(self, var_name, increment):

        super().__init__()

        self.var_name = var_name
        self.increment = increment

    
    def update_blackboard(self, blackboard:dict) -> str:

        try:

            blackboard[self.var_name] += self.increment

            return "success"

        except:

            return "failure"

