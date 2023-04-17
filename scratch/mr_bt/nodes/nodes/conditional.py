#!/usr/bin/env python3

from abc import ABC, abstractmethod

from .child_node import ChildNode
from .node import Node

class Conditional(ChildNode, ABC):
    '''
    The Conditional class is a leaf node in the behavior tree which returns either
    "success" or "failure" based on the boolean output from the condition function.
    Note that unlike other types of behavior tree nodes, a Conditional node will never
    return "running".

    The condition functon should be user defined and return a boolean value. 

    This class is not meant to be initialized, but serves as an abstract parent class for
    users to construct their own Conditional nodes with custom conditional functions.
    '''
    
    @abstractmethod
    def condition(self, blackboard:dict) -> bool:

        return True


    def tick(self, blackboard:dict) -> dict:

        status = 'failure'

        try:
            condition_met = self.condition(blackboard)
            if condition_met:  
                status = 'success'
        except AttributeError:
            status = "running"

        return status, {self.id : status}   