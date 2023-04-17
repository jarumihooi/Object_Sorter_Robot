#!/usr/bin/env python3
from typing import Union
import rospy
from ...nodes.action import Action
from geometry_msgs.msg import Twist

class GamepadTeleop(Action):

    def __init__(self, lin_scale: Union[float, int] = 2, ang_scale: Union[float, int] = 3):

        super().__init__()

        self.lin_scale = lin_scale
        self.ang_scale = ang_scale

        self.twist = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    def execute(self, blackboard):
        try:
            joy = blackboard['/joy']
            self.twist.angular.z = joy.axes[3]*self.ang_scale
            self.twist.linear.x = joy.axes[1]*self.lin_scale

            self.pub.publish(self.twist)
            return "success"
        except:
            return "failure"

        



