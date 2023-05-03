#!/usr/local/bin/python
#!/usr/bin/python
'''Main control for sorter
bf: means bugfix
'''

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from std_msgs.msg import Bool, String # for the Bool/claw, String/state
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time
import sys
from std_msgs.msg import Bool, String


def degrees(r):
    return 180.0 * r / math.pi


class Sorter:
    """Constructor for our class"""
    
    def __init__(self):
        rospy.init_node('main')
        print("Starting main_control in object_Sorter project.")
        # basic params ====
        self.rate = rospy.Rate(30)

        self.state = "find_item"
        self.target_color = "red"

        self.red_target_fiducial = rospy.get_param("~target_fiducial", "fid104")
        self.green_target_fiducial = rospy.get_param("~target_fiducial", "fid108")

        self.mixed_target_fiducial = rospy.get_param("~target_fiducial", "fid109")
        
        #whether the target fiducial has been found, used to solve edge cases related to state changes
        self.acquired = False


        self.target_fiducial = self.red_target_fiducial


        self.LinSpd = 0.0
        self.AngSpd = 0.0

        self.forward_error = 10
        self.fid_x = 1 #set higher so it doesn't trigger conditional immediately
        self.fid_y = 0

        # base rate of speed
        self.angular_rate = 2.0
        self.linear_rate = 0.2

        self.max_angular_rate = 1.2
        self.max_linear_rate = 1.5

        self.min_dist = 0.3

        self.colorTwist = Twist()
        self.fiducialTwist = Twist()

        # extended params ====
        self.red_left = 3 # cans
        self.green_left = 3

        # CORE pub subs ====
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # A publisher for robot motion commands
        self.servo_pub = rospy.Publisher('/servo', Bool, queue_size=1)#publishes commands to the claw
        self.color_pub = rospy.Publisher('/color', String, queue_size=1)#publishes the color the robot is looking for


        self.color_sub = rospy.Subscriber("/color_direction_twist", Twist, self.color_cb) #subscribes to the image recognition node
        self.fid_transform_sub = rospy.Subscriber("/fid_pub", TransformStamped, self.fid_cb) #subscribes to the fiducial transform node


    # callbacks =====
    def color_cb(self, msg):
        self.colorTwist = msg


    def fid_cb(self, msg):

        if msg.child_frame_id == self.target_fiducial:
            self.acquired = True
            ct = msg.transform.translation
            cr = msg.transform.rotation
            #print ("T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                                #(ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w))

            # Set the state varibles to the position of the fiducial
            self.fid_x = ct.x
            self.fid_y = ct.y

            # Calculate the error in the x and y directions
            self.forward_error = self.fid_x - self.min_dist
            lateral_error = self.fid_y

            # Calculate the amount of turning needed towards the fiducial
            # atan2 works for any point on a circle (as opposed to atan)
            angular_error = math.atan2(self.fid_y, self.fid_x)


            self.fiducialTwist.linear.x = (self.forward_error /1.2) * self.linear_rate #damping fidu #1.1 or 1.25
            self.fiducialTwist.angular.z = -angular_error * self.angular_rate - self.fiducialTwist.angular.z / 2.0 #- for some reason
            



    def print_state(self):
        ps = "State: "+self.state + "   TargetColor: " + self.target_color + "    TargetFid: " + self.target_fiducial
        fidtwist = "fidtwist:   " + str(self.fiducialTwist)

        # ps += "\n Pos X undef, Pos Y undef, LinSpd:"+str(self.LinSpd)+" AngSpd:"+str(self.AngSpd)
        # ps += "\n Other vars here. Remaining: "+str(self.remaining)
        print(ps)
        print("ac:    "+ str(self.acquired))



    def run(self):

        print("starting run")
        print(self.state)

        while not rospy.is_shutdown():
            self.print_state()

            self.color_pub.publish(self.target_color)
            finalTwist = Twist()

            if self.state == "find_item":
                #find item

                #open and closes claw based on unused twist value published by contour_image_rec
                if self.colorTwist.linear.y == 0: 
                    self.servo_pub.publish(True) #open
                elif self.colorTwist.linear.y == 1.0:
                    self.servo_pub.publish(False) #close
                    self.state = "deliver_item"
                    self.acquired = False


                finalTwist.linear.x = self.linear_rate
                finalTwist.angular.z = self.colorTwist.angular.z

            elif self.state == "deliver_item":
                # deliver item

                #set target fiducial
                if self.target_color == "red":
                    self.target_fiducial = self.red_target_fiducial
                else:
                    self.target_fiducial = self.green_target_fiducial
                

                
                # delivered
                if self.forward_error < 0.4 and self.acquired: #0.5
                    self.servo_pub.publish(True) #open
                    self.fiducialTwist.linear.x = 0
                    # this section switches color if remaining ====
                    if self.target_color == "green":
                        self.green_left -= 1
                        if self.red_left > 0:
                            self.target_color = "red"
                    else: 
                        self.red_left -= 1
                        if self.green_left > 0:
                            self.target_color = "green"
                    if self.red_left <=0 and self.green_left <= 0:
                        self.state = "shutdown"
                        self.shutdown()
                    # end switch-color section, includes shutdown() ====
                    self.state = "return_to_start"
                    self.target_fiducial = self.mixed_target_fiducial
                    self.acquired = False

                finalTwist = self.fiducialTwist

            elif self.state == "return_to_start":
                #return to starting position


                if self.forward_error < 1 and self.acquired:
                    self.servo_pub.publish(True) #open
                    self.state = "find_item"

                finalTwist = self.fiducialTwist

            
            #twist processing


            # Make sure that the angular speed is within limits
            if finalTwist.angular.z < -self.max_angular_rate:
                finalTwist.angular.z = -self.max_angular_rate
            if finalTwist.angular.z > self.max_angular_rate:
                finalTwist.angular.z = self.max_angular_rate


            # Make sure that the linear speed is within limits
            if finalTwist.linear.x < -self.max_linear_rate:
                finalTwist.linear.x = -self.max_linear_rate
            if finalTwist.linear.x > self.max_linear_rate:
                finalTwist.linear.x = self.max_linear_rate


            #rotates to find fiducial if not present
            if not self.acquired and self.state != "find_item":
                finalTwist.angular.z = 0.4 # turning to find 
                finalTwist.linear.x = -0.05 # reverse param


            self.cmd_pub.publish(finalTwist)


            self.rate.sleep()
        # self.shutdown()




    '''This method ends the program'''
    def shutdown(self):
        print("shutting down")
        shut_twist = Twist()
        shut_twist.linear.x = 0.0
        shut_twist.angular.z = 0.0
        self.cmd_pub.publish(shut_twist)
        #self.rate.sleep()
        #rospy.sleep(1)
        # self.vel_vec = [0,0]
        # self.compute_pub_twist() #convert vel_vec
        # self.pub_vel.publish(self.t_pub)
        sys.exit(0)

if __name__ == '__main__':
    s = Sorter()
    s.run()

