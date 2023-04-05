#!/usr/bin/env python3

'''

RULES FOR THE JSON FORMATTING:

    NODES:

        For each node in a tree, you must provide a "name" parameter and a "type" parameter.

            The "name" field is a string that will be displayed in the Graphviz tree along with the "type" for each node. Each node in a tree must have
            a unique "name" in order for the tree to be displayed properly, but other than that the "name" of a node is arbitrary.

            The "type" parameter is used to specify which type of node you are instantiating. You must use one of the currently available
            node types listed above in the master_node_dict.

        When you are declaring a parent node, you will have a "children" parameter that will ask for a list of other nodes. You must provide
        a list of newly specified nodes in the same format as you would provide information for a regular node. This will give your .json file
        a nested structure.

    REFERENCES:

        You may pass in a reference to another json file node/tree structure as a child of another node. To do this, when declaring the node you must
        pass in an argument "ref" and assign it to the path of the referenced file relative to the interpreter.

    BLACKBOARD:

        You will need to provide a blackboard with the necessary variables to keep track of inside of your .json file. You will put this blackboard in
        as a parameter of the parent node and name it "blackboard".

        There are two types of blackboard variables that can be used in the blackboard.

            The "generic" variables which can be any kind of object or primitive data type supported by python. These types can have any name. They can be
            specified to initially have a null value, or start with a value of a data type supported by json.

            The ROS message variables will have the names of the topic which they are published to. Their name must start with a "/" or they will not be recognized
            as a ROS message and a subscriber will not be instantiated for them. They must initially have a value which is a key for one of the ROS message types specified
            above in the master_msg_dict.

    EXAMPLE:

        {
            "name":"parent",
            "type":"Selector",
            "children":[
                {
                    "name":"child1",
                    "type":"SomeConditionalNode",
                    "some_param1":"foo"
                },
                {
                    "name":"child2",
                    "type":"SomeActionNode",
                    "random_param1":"bar"
                },
                {
                    "ref":"path/to/other/node.json"
                }
            ],
            "blackboard":{
                "/scan":"LaserScan",
                "some_var":null
            }
        }

'''

import json
import os

from loader import import_node
from nodes.nodes.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from fiducial_msgs.msg import *


class TreeBuilder:


    def __init__(self, btree_folder:str):

        self.path = os.path.dirname(os.path.abspath(__file__)) + "/tree_jsons/"
        self.btree_folder = btree_folder
        with open(self.path + btree_folder +  "/root.json") as f:
            self.tree_dict = json.load(f)

        self.blackboard = {}



    def build_tree(self) -> tuple([Node, dict]):
        '''
        The recursive function attach_node() is called on the root of the tree, then the 
        ROS behavior tree root and the blackboard are returned.
        '''
        self.root = self.attach_node(self.tree_dict)

        return self.root, self.blackboard


    def attach_node(self, node: dict) -> Node:

        parameters = {}

        specials = ['name', 'type', 'blackboard']

        for parameter in node: # Each parameter provided in the json is interpreted and used to initialize the node

            if parameter == 'children': # Initializes all children recursively and appends them to a list which is then
                                        # passed as another parameter in the node
                parameters["children"] = []
                
                for child in node['children']:
                    if 'ref' in child: # Handles the case where the child is a reference to another json file
                        f = open(self.path + child['ref'])
                        child = json.load(f)
                        f.close()
                    parameters["children"].append(self.attach_node(child))

            elif parameter not in specials:
                
                parameters[parameter] = node[parameter]

        if 'blackboard' in node: # If the blackboard is passed as a parameter its contents are added to the tree blackboard

            for var in node['blackboard']:
                
                if var[0] == '/':
                    self.blackboard[var] = eval(node['blackboard'][var])
                else:
                    self.blackboard[var] = node['blackboard'][var]

        node_class = import_node(node['type'])(**parameters)
        node_class.label = node['name']
        return node_class
    
    # def link_blackboard(self, root_name, blackboard):
    #     '''
    #     Links the blackboard defined by any of the nodes in the tree and attaches it
    #     to the passed node in the graph and displays its contents.
    #     '''
        
    #     blackboard_string = 'BLACKBOARD\n\n'
    #     for key in blackboard:
    #         blackboard_string += key + '  :  ' + str(blackboard[key]) + '\n'
    #     self.dot.node('Blackboard', blackboard_string, shape='rectangle')
    #     self.dot.edge('Blackboard', root_name)


