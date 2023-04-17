#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import pydot

from nodes.nodes.node import Node
from nodes.nodes.parent_node import ParentNode


color_map = {"success":"green", "failure":"red", "running":"yellow"}
node_styles = ["filled", ""]
edge_styles = ["bold", "dotted"]

def build_dot(root: Node) -> pydot.Dot:
    graph = pydot.Dot("Behavior Tree", graph_type="digraph", bgcolor="white")
    recursive_init(graph, root)
    return graph


def recursive_init(graph: pydot.Dot, node: Node) -> None:

    label = f"{node.name}\n{node.label}"
    dot_node = pydot.Node(node.id, label=label, color="gray")

    graph.add_node(dot_node)

    if issubclass(type(node), ParentNode):
        if not node.blackbox:
            for child in node.children:
                recursive_init(graph, child)
                edge = pydot.Edge(node.id, child.id, color="black")
                graph.add_edge(edge)
        else:
            dot_node.set_label(f"{label}\n...")
            


def color_graph(graph: pydot.Dot, status_dict: dict) -> pydot.Dot:

    executed = [*status_dict] # Returns list of node ids which have been executed
    
    for dot_node in graph.get_node_list():

        node_color = "black"
        id = dot_node.get_name()
        if id in executed:
            node_color = color_map[status_dict[id]]
            dot_node.set_style(node_styles[0])
        else:
            dot_node.set_style(node_styles[1])

        dot_node.set_color(node_color)

    for edge in graph.get_edge_list():
        src = edge.get_source()
        dest = edge.get_destination()
        if (src in executed) and (dest in executed):
            edge.set_style(edge_styles[0])
        else:
            edge.set_style(edge_styles[1])
    
    return graph


def graph_imgmsg_from_str(graph_str: str) -> Image:

        graph = pydot.graph_from_dot_data(graph_str)[0]

        byte_img = graph.create_jpg()

        np_img = np.frombuffer(byte_img, dtype=np.int8)

        cv_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

        return CvBridge().cv2_to_imgmsg(cv_img)


def same_tree_state(status_dict: dict, prev_status_dict: dict) -> bool:

    old_keys = list(prev_status_dict.keys())
    new_keys = list(status_dict.keys())

    if len(old_keys) != len(new_keys):
        return False

    for old_key, new_key in zip(old_keys, new_keys):
        if old_key != new_key:
            return False
        if prev_status_dict[old_key] != status_dict[new_key]:
            return False
    
    return True

