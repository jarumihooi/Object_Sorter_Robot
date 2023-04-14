#!/usr/bin/env python3

from .parent_node import ParentNode

        

class Selector(ParentNode):
    '''
    The Selector class is a parent node in the behavior tree which ticks each of its children nodes
    in left-right order until one of them returns "success" or "running", and then returns the
    status back up the tree. If each child returns "failure", then the Selector will return 
    "failure" back up the tree.
    '''

    def __init__(self, children:list, blackbox: bool = False):
        super().__init__(children, blackbox)
        
    def control_flow(self, blackboard:dict) -> tuple([str, dict]):

        status = 'failure'
        status_dict = {}
        i = 0
        while (status == 'failure') and (i < self.num_children):

            status, child_status_dict = self.children[i].tick(blackboard)
            status_dict = {**status_dict, **child_status_dict}
            i += 1
        
        return status, status_dict