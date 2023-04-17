#!/usr/bin/env python
"""
This code controls the claw of the robot as well as the movement of the robot.
The code is a reworked code from the 'danser' PA, which is why the movement is very simular to that PA.
However, there is an adding of another publisher called servo_pub. This will publish a boolean value to the '/servo' topic
which will tell the claw to open or close. By using the my reworked key_publisher, the 'key_sub' will subscribe to
whatever key is published by the key_publisher. Depending on the key gotten from the key_publisher, this file will 
tell the robot or claw to do something. There are fixed variables for linear speed and angular speed, which can be
edited by the coder depending on how fast one wants the robot to move.

****** PLEASE BE AWARE THAT THE 'h' BUTTON WILL NOT STOP THE ROBOT COMPLELTLY, AS MOMENTUM CARRIES THE ROBOT FURTHER ********

Code written by David Pollack, adapted from prrexamples starter code
"""

#All the imports!
import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

#Scan callback method, does nothing.
def scan_cb(msg):
    return
#This method is all about what to tell the robot after a key is gotten from the key_publisher.py
def key_cb(msg):
    global state; global last_key_press_time; global move_cmd
    state = msg.data
    last_key_press_time = rospy.Time.now()

    #You can edit this to make the robot go faster or turn faster, not needed though, moves and turns fast enough.
    linear_speed = 0.5
    angular_speed = 1.0

    #The following are all of the commands

    #Open claw
    if state == 'o':
        servo_pub.publish(False)
    #Close claw
    elif state == 'c':
        servo_pub.publish(True)
    #Go forward
    elif state == 'w':
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0
    #Go backwards
    elif state == 's':
        move_cmd.linear.x = -linear_speed
        move_cmd.angular.z = 0
    #Turn left
    elif state == 'a':
        move_cmd.angular.z = angular_speed
        move_cmd.linear.x = 0
    #Turn right
    elif state == 'd':
        move_cmd.angular.z = -angular_speed
        move_cmd.linear.x = 0
    #Go forward and left
    elif state == 'q':
        move_cmd.angular.z = angular_speed
        move_cmd.linear.x = linear_speed
    #Go forward and right
    elif state == 'e':
        move_cmd.angular.z = -angular_speed
        move_cmd.linear.x = linear_speed
    #Go backwards and left
    elif state == 'z':
        move_cmd.angular.z = -angular_speed
        move_cmd.linear.x = -linear_speed
    #Go backwards and right
    elif state == 'x':
        move_cmd.angular.z = angular_speed
        move_cmd.linear.x = -linear_speed
    #Stop (*******WARNING WILL STILL GO FOR A BIT*******)
    elif state == 'h':
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

#Odom callback, does nothing for now
def odom_cb(msg):
    return
#Prints to console when running code
def print_state():
    print("---")
    print("STATE: " + state)
    time_since = rospy.Time.now() - last_key_press_time
    print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

#node initilized 
rospy.init_node('claw_control')

#All subscribers (only use key_sub, kept others in case I do things with them): 
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)

#All publishers:
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
servo_pub = rospy.Publisher('/servo', Bool, queue_size=1)

#This code makes sure the robot starts stopped...idk if needed but don't want to mess with
state = "H"

last_key_press_time = rospy.Time.now()

#Twist value to publish
move_cmd = Twist()

rate = rospy.Rate(10)

#Code that runs code unitl shut down.
while not rospy.is_shutdown():
    print_state()
    cmd_vel_pub.publish(move_cmd)
    rate.sleep()
