#!/usr/bin/env python3

from typing import Union
import rospy 
import numpy as np 
from ...nodes.conditional import Conditional


class WallAtAngleRange(Conditional):


    def __init__(self, start_angle: Union[float,int], end_angle: Union[float,int], dist: Union[float, int]):

        super().__init__()

        self.start_angle = start_angle
        self.end_angle = end_angle
        self.dist = dist

    def condition(self, blackboard: dict) -> bool:
        ranges = np.array(blackboard['/scan'].ranges)
        ranges[ranges == 0] = 999
        ranges[ranges < 0.1] = 0

        n = ranges.size
        if n == 0:
            return False

        scaled_start = int(self.start_angle)
        scaled_end = int(self.end_angle)
        return np.min(ranges[scaled_start:scaled_end]) <= self.dist

