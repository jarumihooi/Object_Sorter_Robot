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
   closest_distance = min(msg.ranges)
   if(closest_distance <= 0.2):
      state = "h"

   return

# it is not necessary to add more code here but it could be useful
# def key_cb(msg):
#    global state; global last_key_press_time
#    global sprial_zigzag_timer
#    state = msg.data
#    last_key_press_time = rospy.Time.now()
#    #timer for implementing the spiral and zigzag movement
#    sprial_zigzag_timer = rospy.Time.now()

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


   global linear_transform
   global angular_transform

   #this section implements the spiral and zigzag movements
   time_since_state = rospy.Time.now() - sprial_zigzag_timer

   # This produces a value starting at 1 and decreasing to 0. Increase 10 to slow down the rate it changes and make the spiral tighter
   if(state == "s"):
      angular_transform = 10 / (time_since_state.secs+10)

#changes the robot from driving straight to turning in 2s increments on a 8s "loop"
   if(state == "z"):
      timer = time_since_state.secs % 8
      if(0 <= timer < 2):
         linear_transform = 1
         angular_transform = 0

      if(2 <= timer < 4):
         linear_transform = 0
         angular_transform = 1
      
      if(4 <= timer < 6):
         linear_transform = 1
         angular_transform = 0
      
      if(6 <= timer < 8):
         linear_transform = 0
         angular_transform = -1





   #oreuler = msg.pose.pose.orientation
   #roll, pitch, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w]) 

   #return


# print the state of the robot
def print_state():


   print("---")
   print("STATE: " + state)

   #prints speed and location rounded to 5 decimals
   print(f'x-speed: {round(linear_speed_odom, 5)} z-speed: {round(angular_speed_odom, 5)} xy-location: ({round(x_pos,5)}, {round(y_pos,5)})')

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

def get_vector(state):
   global linear_component
   global angular_component
   global linear_transform
   global angular_transform

   #resets transform variable when not in zigzag or spiral
   #if(state == "l" or state == "r" or state == "f" or state == "b" or state == "h" ):
   if(state != "s" and state != "z"):
      linear_transform = 1
      angular_transform = 1

   if(state == 'l'):
      linear_component = 0
      angular_component = 1

   if(state == 'r'):
      linear_component = 0
      angular_component = -1

   if(state == 'f'):
      linear_component = 1
      angular_component = 0

   if(state == 'b'):
      linear_component = -1
      angular_component = 0

   if(state == 'h'):
      linear_component = 0
      angular_component = 0

   #circle
   if(state == 'c'):
      linear_component = 1
      angular_component = 1
   

   #spiral
   if(state == 's'):
      linear_component = 1
      angular_component = 1

   #zigzag
   if(state == 'z'):
      linear_component = 1
      angular_component = 1


# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
#key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time
state = "c"

#init global variables
linear_component = 0
angular_component = 0

linear_transform = 1
angular_transform = 1

linear_speed_odom = 0
angular_speed_odom = 0

closest_distance = 10

x_pos = 0
y_pos = 0

last_key_press_time = rospy.Time.now()

sprial_zigzag_timer = rospy.Time.now()


#marker stuff
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
marker_counter = 0



# set rate
rate = rospy.Rate(10)


# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

   # print out the current state and time since last key press
   print_state()

  # more marker stuff
   if(math.floor(rospy.Time.now().to_nsec()/100000000) % 10 == 0):
      #print(math.floor(rospy.Time.now().to_nsec()/100000000))
      marker = Marker()

      marker.header.frame_id = "odom"
      marker.header.stamp = rospy.Time.now()

      # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
      marker.type = 2

      marker.id = marker_counter
      marker_counter += 1

      # Set the scale of the marker
      marker.scale.x = 0.1
      marker.scale.y = 0.1
      marker.scale.z = 0.1

      # Set the color
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 1.0

      # Set the pose of the marker
      marker.pose.position.x = x_pos
      marker.pose.position.y = y_pos
      marker.pose.position.z = 0
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 1.0

      marker_pub.publish(marker)

   # publish cmd_vel from here 
   t = Twist()
   
  
   get_vector(state)

   #SPEED is a constant, component is set based on the state, transform changes over time to implement spiral and zigzag
   t.linear.x = LINEAR_SPEED * linear_component * linear_transform
   t.angular.z = ANGULAR_SPEED * angular_component * angular_transform


   # one idea: use vector-2 to represent linear and angular velocity
   # velocity_vector = [linear_component, angular_component]
   # then represent:
   # twist.linear.x = LINEAR_SPEED * linear_component
   # twist.angular.z = ANGULAR_SPEED * angular_component 
   # where for example:
   # LINEAR_SPEED = 0.2, ANGULAR_SPEED = pi/4
   # velocity_vector = [1, 0] for positive linear and no angular movement
   # velocity_vector = [-1, 1] for negative linear and positive angular movement
   # we can then create a dictionary state: movement_vector to hash the current position to get the movement_vector
   # in order to get the zig zag and spiral motion you could you something like this:
   # twist.linear.x = LINEAR_SPEED * linear_component * linear_transform
   # twist.angular.z = ANGULAR_SPEED * angular_component * angular_transform
   # where the [linear_transform, angular_transform] is derived from another source that is based on the clock
   # now you can change the velocity of the robot at every step of the program based on the state and the time
   cmd_vel_pub.publish(t)

   # run at 10hz
   rate.sleep()