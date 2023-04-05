#!/usr/bin/env python3

from abc import ABC, abstractmethod

from .child_node import ChildNode
from .node import Node


class Update(ChildNode, ABC):
    '''
    The Update class is a leaf node in the behavior tree which performs some calculation/algorithm
    on information in the blackboard and updates the blackboard with new information.

    Each update should be tick based, so during each tick the .tick() method of an update
    will either return "failure", or "success" depending whether or not the update is successful

    This class is not meant to be initialized, but serves as an abstract parent class for users
    to construct their own updates with custom methods.
    '''

    @abstractmethod
    def update_blackboard(self, blackboard:dict) -> str:
        return

    
    def tick(self, blackboard:dict) -> tuple([str, dict]):
        try:

            status = self.update_blackboard(blackboard)
        except:
            status = "failure"
        return status, {self.id : status}