#!/usr/local/bin/python
#!/usr/bin/python

'''Info: jeremy attempt at simple fsm
    info: https://github.com/pytransitions/transitions
    Make sure the package is installed: 
    $ pip install transitions
    $ pip install transitions[diagrams]  
    $ sudo apt-get install graphviz graphviz-dev
    

    In ipython:
    from fsm_example import MoveBaseRecovery
    t = MoveBaseRecovery("base")
    t.state
    t.stuck() # etc
    t.get_graph().draw('fsm_example.png', prog='dot')'''

from transitions import Machine, State
import random
from transitions.extensions import GraphMachine

class Matter(object):
    # i think this creates a super class

    # these are the callbacks for when a state changes. they must be applied to the states directly. 
    def chg_state(self): print("changing state")
    def on_enter_gas(self):print("callback for entering gas")
    def make_hissing_noises(self): print("HISSSSSSSSSSSSSSSS")
    def evap(self): print("all liquid successfully evap'd")

# =====
def main():
    lump = Matter()
    states = [State(name='solid', on_enter="chg_state"), 'liquid', 'gas', 'plasma']
    transitions = [
        {"trigger":"melt", 'source':'solid', 'dest':'liquid', 'before': 'make_hissing_noises'},
        { 'trigger': 'evaporate', 'source': 'liquid', 'dest': 'gas', 'after': 'evap' },
        { 'trigger': 'sublimate', 'source': 'solid', 'dest': 'gas' },
        { 'trigger': 'ionize', 'source': 'gas', 'dest': 'plasma' },

        { 'trigger': 'de-ionize', 'source': 'plasma', 'dest': 'gas' },
        { 'trigger': 'condensate', 'source': 'gas', 'dest': 'liquid' },
        { 'trigger': 'freeze', 'source': 'liquid', 'dest': 'solid' },
        { 'trigger': 'deposit', 'source': 'gas', 'dest': 'solid' }
    ]
    machine = GraphMachine(model=lump, states=states, 
        transitions=transitions, initial="solid")
    # bf: was missing transitions=transitions, this creates the methods
    


    # do section ==============
    print(lump.state)
    lump.melt()
    print(lump.state)
    lump.evaporate()
    print(lump.state)
    lump.deposit()
    print(lump.state)
    lump.get_graph().draw('my_state_diagram.png', prog='dot')

main()
