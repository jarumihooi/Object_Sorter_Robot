from abc import ABC, abstractmethod
from .node import Node


class ChildNode(Node, ABC):

    def __init__(self):
        super(ChildNode, self).__init__()

    
    @abstractmethod
    def tick(self, blackboard:dict) -> tuple([str, dict]):
        return