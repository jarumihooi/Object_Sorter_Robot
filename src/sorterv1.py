#!/usr/bin/env python
'''Jeremy Huey, Isaac Goldings, David Pollack
Robotics Sp2023 Final Project - (Object) Sorter
v1J
The comment "bf" means bugfix: Its a note to myself as to something I fixed and why that bug occured. 
This class is the main executable for the project. It will set up the nodes required.
The expected nodes are: this, state-via_pytransitions, movement, color. 
This version adds basic structure. 

Look at Jeremy wall follower for orig code template. 
'''
import sys
import rospy
import math
from std_msgs.msg import String, Int16, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class Sorter():
    def __init__(self):
        print("Sorter init()")
        self.node = rospy.init_node('main')
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        # time info
        self.rate = rospy.Rate(30) 

        # all of the subscribed material to run. 
        self.state_sub = rospy.Subscriber('state', Int16, self.state_cb) #check state, this would get state if we had a diff file to give state. 
        # self.sub_pid_twist = rospy.Subscriber('pid_twist', Twist, self.cb_twist) #this would control pid if used.
        self.key_sub = rospy.Subscriber('keys', String, self.key_cb) #this gets single input keystrokes. #bf: needed to add self.
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb) #this gets odom info to improve perception, used for location. 
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb) 

        # State params: 
        self.state = "h"
        # motion params: 
        self.PROX_DIST = 0.3 #0.8 for sim
        self.LINEAR_SPEED = 0.4
        self.ANGULAR_SPEED = math.pi/4

        #Create two twist variable, one is modified here, one is copied from the PID messages
        self.twist = Twist()

    '''Callback func for key_publisher, lets us control state.'''
    def key_cb(self, msg):
        self.state = msg.data
        # self.last_key_press_time = rospy.Time.now()
        # self.last_pressed = rospy.Time.now().to_sec()

    #Cb f for state.
    def state_cb(self, msg):
        self.state = msg.data

    #Makes the twist object sent from PID global
    # def cb_twist(self, msg):
    #     self.t_pid = msg #bf: forgot self here instead of global

    # odom is also not necessary but very useful
    # def odom_cb(self, msg):
    #     self.location[0] = msg.pose.pose.position.x
    #     self.location[1] = msg.pose.pose.position.y

    def scan_cb(self, msg):
        clean_ranges = list(np.where(np.array(msg.ranges)<0.03, 4, np.array(msg.ranges)))
        print(clean_ranges)
        if (self.state not in ["c","h", "d"]):#and manual_col_override == False):
            # clean data, move it here. 
        
            self.cur_dist = min(msg.ranges[248:293])
            print("cur_dist", self.cur_dist)
        #end if
    #end m()

    # ==== running code ======
    '''This method prints the state of the system'''
    def print_state(self):
        print("STATE: " + self.state)
        if (self.state != ("h" or "d") ):
            speed = self.t_pub.linear.x
        else: 
            speed = 0.0

    '''This method runs the program'''
    def run(self):
        print("Running()")
        while (not rospy.is_shutdown()) and (self.state != "d"):
            self.print_state()
            self.rate.sleep()
        # end while
        self.shutdown() #bf: added self.

    '''This method ends the program'''
    def shutdown(self):
        print("shutting down")
        self.vel_vec = [0,0]
        self.compute_pub_twist() #convert vel_vec #bf: i forgot to do this step after setting vel_vec
        self.pub_vel.publish(self.t_pub)
        #self.rate.sleep()
        rospy.sleep(1)
        self.vel_vec = [0,0]
        self.compute_pub_twist() #convert vel_vec
        self.pub_vel.publish(self.t_pub)
        sys.exit(0)








# =================================
# ===== main ======================
def main():
    print("STARTING main()")
    sorter = Sorter()
    sorter.run()
main()


'''
TODO: try and clean the scan. 
'''