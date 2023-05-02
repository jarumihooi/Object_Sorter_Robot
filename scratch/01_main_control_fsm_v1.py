#!/usr/local/bin/python
#!/usr/bin/python
'''Main control for sorter
bf: means bugfix
'''

import rospy
from geometry_msgs.msg import TransformStamped, Twist
# from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from std_msgs.msg import Bool, String # for the Bool/claw, String/state
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time
import sys
from fsm_state import FSM

def degrees(r):
    return 180.0 * r / math.pi


class Sorter:
    """Constructor for our class"""
    
    def __init__(self):
        rospy.init_node('main')
        print("Starting main_control in object_Sorter project.")
        # basic params ====
        # self.fsm = FSM() // we connect with this via pub sub only. 
        self.rate = rospy.Rate(2)
        self.state = "waiting"
        self.LinSpd = 0.0
        self.AngSpd = 0.0

        # extended params ====
        self.remaining = 3 # cans

        # CORE pub subs ====
        # self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # A publisher for robot motion commands
        self.state_sub = rospy.Subscriber("/state", String, self.state_cb)

    # callbacks =====
    def state_cb(self, msg):
        self.state = msg.data #bf: needed .data


    def print_state(self):
        ps = "State: "+self.state
        ps += "\n Pos X undef, Pos Y undef, LinSpd:"+str(self.LinSpd)+" AngSpd:"+str(self.AngSpd)
        ps += "\n Other vars here. Remaining: "+str(self.remaining)
        print(ps)

    '''This method ends the program'''
    def shutdown(self):
        print("shutting down")
        shut_twist = Twist()
        shut_twist.linear.x = 0.0
        shut_twist.angular.z = 0.0
        self.cmd_pub.publish(shut_twist)
        #self.rate.sleep()
        #rospy.sleep(1)
        # self.vel_vec = [0,0]
        # self.compute_pub_twist() #convert vel_vec
        # self.pub_vel.publish(self.t_pub)
        sys.exit(0)

    def run(self):
        print("starting run")
        print(self.state)
        while not rospy.is_shutdown():
            self.print_state()
            self.rate.sleep()
        # self.shutdown()

if __name__ == '__main__':
    s = Sorter()
    s.run()

