#!/usr/bin/env python3

from typing import Union
import rospy 
import numpy as np 
from ...nodes.conditional import Conditional


class WallAtAngleDynamic(Conditional):


    def __init__(self, angle_var_name: str, scan_var_name: str, dist: Union[float,int], fov: Union[float,int]):

        super().__init__()

        self.angle_var_name = angle_var_name
        self.scan_var_name = scan_var_name
        self.dist = dist
        self.fov = fov

    def condition(self, blackboard: dict) -> bool:
        ranges = blackboard[self.scan_var_name].ranges
        angle =  blackboard[self.angle_var_name]
        ranges[ranges == 0] = 999
        ratio = int(ranges.size/360)

        return ranges[ratio*angle] >= self.dist