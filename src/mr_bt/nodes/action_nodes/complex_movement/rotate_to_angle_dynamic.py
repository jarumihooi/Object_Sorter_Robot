#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ...nodes.action import Action


class RotateToAngleDynamic(Action):


    def __init__(self, angle_var_name: str, curr_angle_var_name: str):

        super().__init__()

        self.angle_var_name = angle_var_name
        self.curr_angle_var_name = curr_angle_var_name
        self.twist = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, blackboard:dict) -> str:

        try:
            curr_angle = blackboard[self.curr_angle_var_name]
            goal_angle = blackboard[self.angle_var_name]

            if abs(curr_angle-goal_angle) > 1:

                if curr_angle >= 180:
                    curr_angle = curr_angle - 360

                curr_rad = (3.1415/180)*curr_angle
                goal_rad = (3.1415/180)*goal_angle
                self.twist.angular.z = goal_rad - curr_rad
                self.pub.publish(self.twist)

                return "running"
            else:
                return "success"
        except:
            return "failure"
