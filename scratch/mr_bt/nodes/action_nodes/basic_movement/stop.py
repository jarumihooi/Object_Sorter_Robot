#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ...nodes.action import Action


'''
Publishes a cmd_vel topic to cease all linear and angular movement.
'''
class Stop(Action):


    def __init__(self):

        super().__init__()

        self.twist = Twist()

        self.twist.linear.x = 0
        self.twist.angular.z = 0

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, blackboard: dict) -> str:

        self.pub.publish(self.twist)

        return "success"