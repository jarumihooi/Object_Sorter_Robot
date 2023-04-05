from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree
import os

class state_pusher(object):
    
    def __init__(self, name):
        self.name = name
        initt()
    
    def initt():
        print("Started init")
        tree = rospy.get_param("tree")
        log = rospy.get_param("log")

        rospy.init_node('btree')

        tb = TreeBuilder(tree)
        root, blackboard = tb.build_tree()        

        tree = ROSBehaviorTree(root, blackboard, log)
        print("Ended init")
        rospy.spin()

if __name__ == '__main__':
    stater = state_pusher("main")