#!/usr/bin/env python3

from typing import Union
import rospy 
import numpy as np 
from ...nodes.conditional import Conditional
from .wall_ahead import WallAhead


'''
Inverse of WallAhead conditional.
'''
class ClearAhead(WallAhead):


    def __init__(self, dist: float, fov: Union[float, int]):

        super().__init__(dist=dist, fov=fov)


    def condition(self, blackboard: dict) -> bool:

        wall_ahead = super().condition(blackboard)

        return not wall_ahead
