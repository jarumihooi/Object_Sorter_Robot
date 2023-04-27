#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 3.1415/4



# fill in scan callback
def scan_cb(msg):
   global state
   global linear_component
   global angular_component

   rangesb = []

   for i in range(len(msg.ranges)):

      if msg.ranges[i] < 0.05:
         rangesb.append(200)
      else:
         rangesb.append(msg.ranges[i])
      
   

   closest_distance = get_closest(rangesb)

   print("closest_dist:  " + str(closest_distance))

   if(closest_distance < 0.3):
      linear_component = 0
      angular_component = 0



def get_closest(arr):

    avg_arr = []
    for i in range(0, len(arr) - 11):
        sum = 0
        for ten_deg in range(0,10):
            sum += arr[i+ten_deg]
        avg_arr.append(sum / 10)

    print(avg_arr)
    return min(avg_arr)

   
# def process_ranges(msg):
   # for i in msg:
   #    if i < 0.05:
   #       i = 200
   
#    for i in ranges(0, len(msg) - 10):
#       for j in ranges(10):
#          sum +=

      




# odom is also not necessary but very useful
def odom_cb(msg):

   #these variables track the speed and location to print out
   global linear_speed_odom
   global angular_speed_odom

   linear_speed_odom = msg.twist.twist.linear.x
   angular_speed_odom = msg.twist.twist.angular.z

   ppoint = msg.pose.pose.position
   global x_pos
   global y_pos

   x_pos = ppoint.x
   y_pos = ppoint.y



# print the state of the robot
def print_state():


   print("---")


# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time

#init global variables
linear_component = 1
angular_component = 1

linear_transform = 1
angular_transform = 1

linear_speed_odom = 0
angular_speed_odom = 0

closest_distance = 10

x_pos = 0
y_pos = 0

last_key_press_time = rospy.Time.now()


# set rate
rate = rospy.Rate(10)


# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

   # print out the current state and time since last key press
   print_state()

   # publish cmd_vel from here 
   t = Twist()
   
  

   #SPEED is a constant, component is set based on the state, transform changes over time to implement spiral and zigzag
   t.linear.x = LINEAR_SPEED * linear_component
   t.angular.z = ANGULAR_SPEED * angular_component


   cmd_vel_pub.publish(t)

   # run at 10hz
   rate.sleep()