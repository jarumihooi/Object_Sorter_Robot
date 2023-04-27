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

import rospy
from transitions import Machine, State
import random
from transitions.extensions import GraphMachine
from std_msgs.msg import Bool, String

class FSM(object):
    def __init__(self):
        rospy.init_node('fsm')
        self.rate = rospy.Rate(2)
        self.objects_left = 1

        # pubs
        self.state_pub = rospy.Publisher("/state", String, queue_size=1)

        # init the fsm 
        states = [
            State(name='Goto_loading', on_enter="chg_state"),
            'Cv_items', 
            'Pickup',
            'Goto_drop', 
            State(name='Drop', on_exit="dropped_off_cb"),
            'Shutdown'
        ]
        transitions = [
            {"trigger":"at_loading", 'source':'Goto_loading', 'dest':'Cv_items', 'before': 'remaining_cb'},
            { 'trigger': 'inpos_pickup', 'source': 'Cv_items', 'dest': 'Pickup', 'after': 'pickedup' },
            { 'trigger': 'deliver_goto', 'source': 'Pickup', 'dest': 'Goto_drop' },
            { 'trigger': 'inpos_drop', 'source': 'Goto_drop', 'dest': 'Drop' },
            { 'trigger': 'reload', 'source': 'Drop', 'dest': 'Goto_loading' },
            { 'trigger': 'end', 'source': 'Drop', 'dest': 'Shutdown' }
        ]
        machine = GraphMachine(model=self, states=states, 
            transitions=transitions, initial="Goto_loading")
        # end init the fsm ====
   
    # end init()

    # these are the callbacks for when a state changes. they must be applied to the states directly. 
    def remaining_cb(self): print("objects_left", self.objects_left)
    def pickedup(self):print("after: we picked up a can")
    def dropped_off_cb(self): self.objects_left -= 1; print("objects_left", self.objects_left)

    def run(self):
        print("starting fsm")
        # print("fsm thinks state is", self.state)
        while not rospy.is_shutdown():
            self.state_pub.publish(self.state)
            self.rate.sleep()

    
if __name__ == '__main__':
    fsm = FSM()
    fsm.run()

# =====
# if __name__ == '__main__':
#     fsm = FSM()
   
    # bf: was missing transitions=transitions, this creates the methods
    


    # do section ==============
    # fsm.get_graph().draw('fsm_state_diag.png', prog='dot')
    # print(fsm.state)
    # fsm.at_loading()
    # print(fsm.state)
    # fsm.evaporate()
    # print(fsm.state)
    # fsm.deposit()
    # print(fsm.state)
    

