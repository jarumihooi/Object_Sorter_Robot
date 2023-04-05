#!/usr/bin/env python3

import numpy as np 
from ...nodes.conditional import Conditional


class ReachedPosition(Conditional):

    def __init__(self, goal_pos_var_name: str, error: float):

        super().__init__()

        self.goal_pos_var_name = goal_pos_var_name
        self.error = error
    
    def condition(self, blackboard: dict) -> bool:
        pos = [blackboard["/odom"].pose.pose.position.x, blackboard["/odom"].pose.pose.position.y]
        goal = blackboard[self.goal_pos_var_name]
        dist = np.sqrt((goal[0]-pos[0])**2 + (goal[1]-pos[1])**2)

        return dist < self.error