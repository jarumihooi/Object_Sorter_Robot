#!/usr/bin/env python3

from abc import ABC, abstractmethod
from .node import Node



class ParentNode(Node, ABC):
    '''
    This class is a blueprint for different types of parent nodes in the behavior tree. 
    All parents will take in a list of child nodes as a parameter when initialized.
    The child nodes can either be action/conditional nodes, sequencers, or other selectors.
    '''

    def __init__(self, children:list, blackbox: bool = False):

        super().__init__()

        self.num_children = len(children)
        
        self.children = children

        self.blackbox = blackbox


    "Implements the main control flow for each of the children and returns the status"
    @abstractmethod
    def control_flow(self, blackboard:dict) -> tuple([str, dict]):
        return "", {}


    def tick(self, blackboard:dict) -> tuple([str, dict]):

        status, status_dict = self.control_flow(blackboard)

        status_dict[self.id] = status

        return status, status_dict

