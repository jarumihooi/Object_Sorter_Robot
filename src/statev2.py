#!/usr/bin/env python3
# needs shebang!

import os
import sys
# paath = os.path.abspath(r"/my_ros_data/catkin_ws/src/COSI119-final-objectSorter/src")
# sys.path.append(paath)
import rospy
from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree
import time

class state_pusher(object):
    
    def initt(self):
        print("Started init")
        rospy.init_node('btree')

        self.start_time = rospy.Time.now()
        self.start_time_pub = rospy.Publisher('/start_time', String, queue_size=10)
        pub.publish(self.start_time)

        self.curT = rospy.Time.now()
        self.curT_pub = rospy.Publisher('/curT', String, queue_size=10)
        pub.publish(self.curT)

        tree = rospy.get_param("tree")
        log = rospy.get_param("log")

        

        tb = TreeBuilder(tree)
        root, blackboard = tb.build_tree()        

        tree = ROSBehaviorTree(root, blackboard, log)
        print("Ended init")
        rospy.spin()

    def __init__(self, name):
        self.name = name
        self.initt()
    
    

if __name__ == '__main__':
    stater = state_pusher("main")