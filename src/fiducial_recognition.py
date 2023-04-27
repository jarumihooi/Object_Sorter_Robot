#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from diagnostic_msgs.msg import DiagnosticArray
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time


def degrees(r):
    return 180.0 * r / math.pi

class Follow:
    """
    Constructor for our class
    """
    def __init__(self):
       rospy.init_node('follow')

       # Set up a transform listener so we can lookup transforms in the past
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)

       # Setup a transform broadcaster so that we can publish transforms
       # This allows to visualize the 3D position of the fiducial easily in rviz
       self.br = tf2_ros.TransformBroadcaster()

       # A publisher for robot motion commands 
       self.fid_pub = rospy.Publisher("/fid_pub", TransformStamped, queue_size=1) #TODO list or tuple?

       # Subscribe to incoming transforms
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)



    """
    Called when a FiducialTransformArray is received
    """

    def newTf(self, msg):
        imageTime = msg.header.stamp

        print (imageTime, rospy.Time.now())
        print ("*****")

        finalTransforms = {}


        # For every fiducial found by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            print ("Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, trans.x, trans.y, trans.z,
                                  rot.x, rot.y, rot.z, rot.w))
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = msg.header.frame_id
            t.header.stamp = imageTime

            t.transform.translation.x = trans.z #x to z
            t.transform.translation.y = trans.x # y to x
            t.transform.translation.z = trans.y # z to y

            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            self.br.sendTransform(t)
            print(t)
            self.fid_pub.publish(t)




            







    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        rate = rospy.Rate(20)


        # While our node is running
        while not rospy.is_shutdown():
            # Calculate the error in the x and y directions




            #publish result here
            rate.sleep()


if __name__ == "__main__":
    # Create an instance of our follow class
    node = Follow()
    # run it
    node.run()