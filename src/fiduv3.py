#!/usr/local/bin/python
'''
Fidu Follower 
V3
Jeremy Huey 
'''
import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time

def degrees(r):
    return 180.0 * r / math.pi

class Fidu:
    def __init__(self):
        print("Starting fidu.py")
        rospy.init_node('fidu')

        self.rate = rospy.Rate(30)
        self.linSpeed = 0.25
        self.angSpeed = 0.25

        self.forward_error = 0.0
        self.lateral_error = 0.0

        self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
        self.lr = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.found1 = False
        self.found2 = False
        self.fid_list = rospy.get_param("~target_fiducial", ["fid103", "fid109", "fid112", "fid107", "fid000"])
        self.target_fiducial = rospy.get_param("~target_fiducial", "fid103")#, "fid107"])
        self.target_fiducial2 = rospy.get_param("~target_fiducial", "fid107")

        self.min_dist = rospy.get_param("~min_dist", 0.3) #bf: should be 0.6

        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
        self.fid_x = self.min_dist
        self.fid_y = 0
        self.got_fid = False

        print("Finished init")

    def newTf(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        print("imageTime:",imageTime, rospy.Time.now())
        print("*****")
        self.found1 = False
        self.found2 = False

        # For every fiducial found1 by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            print("Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, trans.x, trans.y, trans.z,
                                  rot.x, rot.y, rot.z, rot.w) )
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = msg.header.frame_id
            t.header.stamp = imageTime
            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            self.br.sendTransform(t)

            if t.child_frame_id == self.fid_list[0]: # changed from == to in
                # We found the fiducial we are looking for
                self.found1 = True

                # Add the transform of the fiducial to our buffer
                self.tfBuffer.set_transform(t, "follow")

            # if t.child_frame_id == self.target_fiducial: # changed from == to in
            #     # We found the fiducial we are looking for
            #     self.found1 = True

            #     # Add the transform of the fiducial to our buffer
            #     self.tfBuffer.set_transform(t, "follow")
            # if t.child_frame_id == self.target_fiducial2:
            #     # We found the fiducial we are looking for
            #     self.found2 = True

            #     # Add the transform of the fiducial to our buffer
            #     self.tfBuffer.set_transform(t, "follow")

    def move(self):
        twist = Twist()
        print("Move(), self.forward_error:", self.forward_error, "found1:", self.found1)
        print("found2:", self.found2)
        if self.found1 or self.found2:#self.forward_error > 0:
            print("moving")
            twist.linear.x = 0.4
            twist.angular.z = 0
        else:
            twist.linear.x = 0
            twist.angular.z = 0.4
            
            #del self.fid_list[0]

        self.cmdPub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            # Calculate the error in the x and y directions
            self.forward_error = self.fid_x - self.min_dist
            self.lateral_error = self.fid_y

            angular_error = math.atan2(self.fid_y, self.fid_x)

            # print("Errors: forward %f lateral %f angular %f" % \
            #   (forward_error, lateral_error, degrees(angular_error)) )

            if (self.found1 == False):
                print("No fidu found1")
            else:
                print("fidu", self.target_fiducial)
            
            self.move()
            self.rate.sleep()

if __name__ == "__main__":
    # Create an instance of our follow class
    fidu = Fidu()
    # run it
    fidu.run()
# eof