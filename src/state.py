# Example of finite state machines
# First we show how to do the example using the pytransitions package
# Make sure the package is installed: pip install transitions; pip install transitions[diagrams]  
#
# In ipython:
#     from fsm_example import MoveBaseRecovery
#     t = MoveBaseRecovery("base")
#     t.state
#     t.stuck() # etc
#     t.get_graph().draw('fsm_example.png', prog='dot')

from transitions.extensions import GraphMachine

class MoveBaseRecovery(object):
    states = ['navigating', 'reset', 'rotate1', 'reset_aggressive', 'rotate2', 'abort']

    def __init__(self, name):
        self.name = name
        self.machine = GraphMachine(model=self, states=MoveBaseRecovery.states, initial='navigating')

        self.machine.add_transition(trigger='stuck', source='navigating', dest='reset')
        self.machine.add_transition(trigger='clear', source='reset', dest='navigating')
        self.machine.add_transition(trigger='clear', source='rotate1', dest='navigating')
        self.machine.add_transition(trigger='clear', source='reset_aggressive', dest='navigating')
        self.machine.add_transition(trigger='clear', source='rotate2', dest='navigating')
        self.machine.add_transition(trigger='stuck', source='reset', dest='rotate1')
        self.machine.add_transition(trigger='stuck', source='rotate1', dest='reset_aggressive')
        self.machine.add_transition(trigger='stuck', source='reset_aggressive', dest='rotate2')
        self.machine.add_transition(trigger='stuck', source='rotate2', dest='abort')






# ========================= other options
    '''This method figures out what the state command is and does that. 
    Following wall on right of robot. '''
    # 'g' = go wall, 'a' = align, "p" = parallel
    def do_states(self, state):
        # stopping states ==========================================
        if (self.state ==  "h" ):
            self.vel_vec = [0,0]
        elif (self.state == "d"):
            print("Go to shutdown state")
            self.vel_vec = [0,0]
        # automation states ===================================
        elif(self.state == "g"):
            if ( self.wall_sensed["F"] and (self.wall_sensed["FR"] or self.wall_sensed["FR"]) ): #found wall
                print("found wall, aligning")
                self.state = "a"
                self.vel_vec = [0,0]
            else: #not found, continue

                self.vel_vec = [1,0] #go forward
        elif(self.state == "a"):
            if (self.wall_sensed["F"]): #still walled, rotate left
                self.vel_vec = [-0.1,1] #rot l
                print("rot l, aligning")
            elif ((not self.wall_sensed["F"]) and self.wall_sensed["R"]): # okay, now parallel.
                self.vel_vec = [0,0]
                self.state = "p"
                print("paralleled, moving along wall")
            else: 
                self.vel_vec = [0,1] #rot l
                print("rot l, still aligning")                   
                
        elif(self.state == "p"):
            if ((not self.wall_sensed["F"]) and self.wall_sensed["R"]): #still walled, rotate left
                self.vel_vec = [1,0]#self.t_pid.angular.z] #paralleling wall on right. 
                #self.vel_vec = [1,self.t_pid.angular.z]; print("self.t_pid.angular.z", self.t_pid.angular.z) # PID SECTION
            elif (self.wall_sensed["F"] ): #and self.wall_sensed["R"]): # hit R corner
                #self.vel_vec = [0,1]
                self.state = "a"
                print("cornered, move to align[a]")
            # elif (self.wall_sensed["BR"] and (not self.wall_sensed["F"]) and (not self.wall_sensed["FR"]) ):
            #     self.vel_vec = [0,-1] #rot r.
            #     print("Returning towards wall")
            # elif (all(value == False for value in self.wall_sensed.values()) ):
            #     self.last_pressed = rospy.Time.now().to_sec() #must reset last pressed time. 
            #     self.state = "s"
            #     print("Lost wall, spiralling.")
            elif ((not self.wall_sensed["F"]) and (not self.wall_sensed["R"])): #if we dont have the wall.
                self.vel_vec = [0.4,-0.75] #rot r.
                print("Returning towards wall")
            else: 
                print("I'm confused, halting. ")
                # BUG: THIS CAN OCCUR IF WE HIT THE WALL AT A WEIRD ANGLE OR ON THE LEFT
                self.state = "h"
        elif (self.state == "s"):
            if (all(value == False for value in self.wall_sensed.values()) ):
                self.spiral_turn = max(0.1, 2.0 + 0.05 * (self.last_pressed - self.curT)) #gradually reduces the spiraling. 
                # BUG: It might never find the wall and start to circle. 
                self.vel_vec = [0.75,-self.spiral_turn];# print(self.spiral_turn)
                print("Lost wall, spiralling right in search. Spiral =",self.spiral_turn)
            elif ( not all(value == False for value in self.wall_sensed.values()) ): #found wall
                print("found wall, aligning")
                #self.spiral_turn = 2.5 #reset??
                #self.vel_vec = [0,0]
                self.state = "a"
            else: 
                print("I'm confused in spiralling, halting. ")
                self.state = "h"

        #these are movement options
        elif (self.state == "l"):
            self.vel_vec = [0,1]
        elif (self.state == "r"):
            self.vel_vec = [0,-1]
        elif (self.state == "f"):
            self.vel_vec = [1,0]
        elif (self.state == "b"):
            self.vel_vec = [-1,0]
        # elif(self.vel_vec == "s"):
        #     self.spiral_turn = max(0.5, 2.5 + 0.1 * (self.last_pressed - self.curT)) #gradually reduces the spiraling. 
        #     self.vel_vec = [0.75,self.spiral_turn];# print(self.spiral_turn)
        
        #elif (self.state == "g" ):
            #Calculate and set appropriate t_pub values
        else:
            print("STATE NOT FOUND, setting to halt")
            self.state = "h"

        # if in command state:
        self.compute_pub_twist() #convert vel_vec to actual linear and angular twist commands. 
    #end m()

        '''convert vel_vec to actual linear and angular twist commands. '''
    def compute_pub_twist(self):
        self.t_pub.linear.x = self.LINEAR_SPEED * self.vel_vec[0] #bf forgot self. on everything
        self.t_pub.angular.z = self.ANGULAR_SPEED * self.vel_vec[1]