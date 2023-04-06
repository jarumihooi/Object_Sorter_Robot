#!/usr/bin/env python

import rospy
from transitions import Machine
from geometry_msgs.msg import Twist

class Robot(object):

    def __init__(self):
        # Initialize the state machine
        self.machine = Machine(model=self, states=['stopped', 'moving'], initial='stopped')

        # Define the state transitions
        self.machine.add_transition('start', 'stopped', 'moving')
        self.machine.add_transition('stop', 'moving', 'stopped')

        # Initialize the ROS node
        rospy.init_node('robot_move')

        # Initialize the publisher for the robot's velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set the robot's initial velocity to zero
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0

    def start_moving(self):
        # Set the robot's velocity to move forward
        self.vel_cmd.linear.x = 0.2
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)

    def stop_moving(self):
        # Set the robot's velocity to stop
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)

if __name__ == '__main__':
    # Create a new robot instance
    robot = Robot()

    # Start the robot moving forward
    robot.start()

    # Wait for 4 seconds
    rospy.sleep(400)

    # Stop the robot
    robot.stop()
