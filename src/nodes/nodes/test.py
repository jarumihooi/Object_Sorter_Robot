#!/usr/bin/env python3

from .node import Node

class Test(Node):

    def __init__(self, print_statement):

        super().__init__()

        self.print_statement = print_statement

    
    def tick(self, blackboard:dict) -> dict:

        print(self.print_statement)

        return 'success'