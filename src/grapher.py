#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from graph_utils import graph_imgmsg_from_str


class Grapher:

    def __init__(self):

        self.pub = rospy.Publisher("btree", Image, queue_size=10)
        self.sub = rospy.Subscriber("graph_dot", String, callback=self.cb)


    def cb(self, msg):

        graph_str = msg.data
        graph_imgmsg = graph_imgmsg_from_str(graph_str)
        self.pub.publish(graph_imgmsg)


if __name__ == "__main__":

    rospy.init_node("grapher")

    g = Grapher()

    rospy.spin()
