#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ...nodes.action import Action


class LinearAngularDynamic(Action):


    def __init__(self, linear_var_name: str, angular_var_name: str):

        super().__init__()

        self.lin_var_name, self.ang_var_name = linear_var_name, angular_var_name
        self.twist = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, blackboard: dict) -> str:

        try:
            self.twist.linear.x = blackboard[self.lin_var_name]
            self.twist.angular.z = blackboard[self.ang_var_name]

            self.pub.publish(self.twist)

            return "success"
        except:
            return "failure"
