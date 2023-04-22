# jeremy attempt at simple fsm
# Make sure the package is installed: pip install transitions; pip install transitions[diagrams]  
#
# In ipython:
#     from fsm_example import MoveBaseRecovery
#     t = MoveBaseRecovery("base")
#     t.state
#     t.stuck() # etc
#     t.get_graph().draw('fsm_example.png', prog='dot')

from transitions import Machine
import random

class Matter(object):
    # i think this creates a super class
    def chg_state(): print("changing state")

# =====
def main():
    lump = Matter()
    transitions = [
        {"trigger":"melt", 'source':'solid', 'dest':'liquid'},
        { 'trigger': 'evaporate', 'source': 'liquid', 'dest': 'gas' },
        { 'trigger': 'sublimate', 'source': 'solid', 'dest': 'gas' },
        { 'trigger': 'ionize', 'source': 'gas', 'dest': 'plasma' }
    ]
    machine = Machine(model=lump, states=['solid', 'liquid', 'gas', 'plasma'], 
        transitions=transitions, initial="solid")
    # bf: was missing transitions=transitions, this creates the methods
    


    # do section ==============
    print(lump.state)
    lump.melt()
    print(lump.state)

main()