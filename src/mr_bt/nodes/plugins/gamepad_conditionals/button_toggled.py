#!/usr/bin/env python3
from ...nodes.conditional import Conditional

class ButtonToggled(Conditional):

    def __init__(self, button: int = 0):

        super().__init__()

        self.button = button
        self.toggle = False
        self.prev_state = 0


            


    def condition(self, blackboard:dict):
        try:
            joy_button = blackboard['/joy'].buttons[self.button]

            if joy_button == 1 and self.prev_state == 0:
                self.toggle = not self.toggle

            self.prev_state = joy_button

            return self.toggle
        except:
            return False
