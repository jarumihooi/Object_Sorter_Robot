#!/usr/bin/env python3

from abc import ABC, abstractmethod
import random
from sys import stdout


class Node(ABC):
    '''
    The Node class is an abstract class for every type of node in the behavior tree.
    This class is not meant to be initialized and instead used as a blueprint for other types
    of nodes.
    '''

    def __init__(self, blackbox: bool = False):
        self.id = str(id(self))
        self.name = self.__class__.__name__
        self.label = None




    '''
    When tick() is called on a node it will return either "failure" if the action(s) associated with 
    the node have failed, "success" if they have completed, or "running" if they are still in progress.
    '''
    @abstractmethod
    def tick(self, blackboard:dict) -> tuple([str, dict]):
        return "", {}

