#!/usr/bin/env python3

from abc import ABC, abstractmethod

from .child_node import ChildNode
from .node import Node


class Action(ChildNode, ABC):
    '''
    The Action class is a leaf node in the behavior tree which completes an action
    specified in the __init__ method. The user is required to customize their action 
    methods using this blueprint as a guide.

    Each action should be tick based, so during each tick the .tick() method of an action
    will either return "running", "failure", or "success" depending on the state of the action.

    This class is not meant to be initialized, but serves as an abstract parent class for users
    to construct their own actions with custom methods.
    '''
    
    def __init__(self):
        super(Action, self).__init__()



    @abstractmethod
    def execute(self, blackboard:dict) -> str:
        return
    

    def tick(self, blackboard:dict) -> tuple([str, dict]):

        try:

            status = self.execute(blackboard)
        except:
            
            status = "failure"

        return status, {self.id : status}