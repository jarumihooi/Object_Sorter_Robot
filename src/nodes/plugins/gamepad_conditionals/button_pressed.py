#!/usr/bin/env python3
from ...nodes.conditional import Conditional

class ButtonPressed(Conditional):

    def __init__(self, button: int = 0):

        super().__init__()

        self.button = button


    def condition(self, blackboard:dict):
        try:
            joy_button = blackboard['/joy'].buttons[self.button]
            if joy_button == 1:
                return True
            else:
                return False
            
        except:
            return False
