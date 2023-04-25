#!/usr/local/bin/python
#!/usr/bin/python


import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from std_msgs.msg import Bool # for the claw
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time
import sys

def degrees(r):
    return 180.0 * r / math.pi


class Follow:
    """Constructor for our class"""
    
    def __init__(self):
        rospy.init_node('main')
        print("Starting main_control in object_Sorter project.")

        # Set up a transform listener so we can lookup transforms in the past
        self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
        self.lr = tf2_ros.TransformListener(self.tfBuffer)

        # Setup a transform broadcaster so that we can publish transforms
        # This allows to visualize the 3D position of the fiducial easily in rviz
        self.br = tf2_ros.TransformBroadcaster()

        # A publisher for robot motion commands
        self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Flag to avoid sending repeated zero speeds
        self.suppressCmd = False

        # The name of the coordinate frame of the fiducial we are interested in
        # self.fid_list = rospy.get_param("~target_fiducial", ["fid103", "fid107", "fid112", "fid109", "fid000"])
        self.fid_list = rospy.get_param("~target_fiducial", ["fid104", "fid108", "fid100", "fid109", "fid000"])

        self.target_fiducial = rospy.get_param("~target_fiducial", "fid103")#, "fid106"])
        self.target_fiducial2 = rospy.get_param("~target_fiducial", "fid107")

        # Minimum distance we want the robot to be from the fiducial
        self.min_dist = rospy.get_param("~min_dist", 0.3) #changed this to allow closer and better motion. 

        # Maximum distance a fiducial can be away for us to attempt to follow
        self.max_dist = rospy.get_param("~max_dist", 2.5)

        # Proportion of angular error to use as angular velocity
        self.angular_rate = rospy.get_param("~angular_rate", 2.0)

        # Maximum angular speed (radians/second)
        self.max_angular_rate = rospy.get_param("~max_angular_rate", 1.2)

        # Angular velocity when a fiducial is not in view
        self.lost_angular_rate = rospy.get_param("~lost_angular_rate", 0.6)

        # Proportion of linear error to use as linear velocity
        self.linear_rate = rospy.get_param("~linear_rate", 1.2)

        # Maximum linear speed (meters/second)
        self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.5)

        # Linear speed decay (meters/second)
        self.linear_decay = rospy.get_param("~linear_decay", 0.9)

        # How many loop iterations to keep linear velocity after fiducial
        # disappears
        self.hysteresis_count = rospy.get_param("~hysteresis_count", 20)

        # How many loop iterations to keep rotating after fiducial disappears
        self.max_lost_count = rospy.get_param("~max_lost_count", 800) #400

        # Subscribe to incoming transforms
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
        self.fid_x = self.min_dist
        self.fid_y = 0
        self.got_fid = False

    '''This method ends the program'''
    def shutdown(self):
        print("shutting down")
        shut_twist = Twist()
        shut_twist.linear.x = 0.0
        shut_twist.angular.z = 0.0
        self.cmdPub.publish(shut_twist)
        #self.rate.sleep()
        #rospy.sleep(1)
        # self.vel_vec = [0,0]
        # self.compute_pub_twist() #convert vel_vec
        # self.pub_vel.publish(self.t_pub)
        sys.exit(0)