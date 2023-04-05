#!/usr/bin/env python3

from .parent_node import ParentNode
from concurrent.futures import ThreadPoolExecutor


class Multitasker(ParentNode):
    '''
    The Multitasker class is a parent node in the behavior tree which utilizes multithreading in Python
    to simultaneously tick each of it's children nodes at the same time. Each of it's children nodes will
    run in their own threads and the results are only gathered once all of the children have returned a
    status. If one or more of the children nodes returns the "failure" status, then the Multitasker will return
    "failure". Otherwise, if one or more of the children nodes returns "running", then the Multitasker will
    return "running". If all of the children nodes return "success", then the Multitasker will also return "success".
    '''

    def __init__(self, children:list, blackbox: bool = False):
        super().__init__(children, blackbox)

    

    def control_flow(self, blackboard:dict) -> tuple([str, dict]):

        statuses = {}

        with ThreadPoolExecutor() as executor:

            threads = [executor.submit(child.tick, blackboard) for child in self.children]
            results = [thread.result() for thread in threads]

        status_dict = {}
        statuses = []
        for result in results:
            statuses.append(result[0])
            status_dict.update(result[1])
        
        status = "running"

        if "failure" in statuses:

            return "failure", status_dict

        elif "running" in statuses:
            
            return "running", status_dict

        return "success", status_dict