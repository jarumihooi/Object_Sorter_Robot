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
from fsm_state import FSM

def degrees(r):
    return 180.0 * r / math.pi


class Sorter:
    """Constructor for our class"""
    
    def __init__(self):
        rospy.init_node('main')
        print("Starting main_control in object_Sorter project.")
        # basic params ====
        # self.fsm = FSM() // we connect with this via pub sub only. 
        self.rate = rospy.Rate(2)

        self.state = "find_item"
        self.target_color = "red"

        self.red_target_fiducial = rospy.get_param("~target_fiducial", "fid104")
        self.green_target_fiducial = rospy.get_param("~target_fiducial", "fid109")

        self.target_fiducial = red_target_fiducial


        self.LinSpd = 0.0
        self.AngSpd = 0.0

        self.angular_rate = 2.0
        self.linear_rate = 1.2

        self.colorTwist = Twist()
        self.fiducialTwist = Twist()

        # extended params ====
        self.remaining = 3 # cans

        # CORE pub subs ====
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # A publisher for robot motion commands

        self.color_sub = rospy.Subscriber("/color_direction_twist", Twist, self.color_cb) #subscribes to the image recognition node
        self.fid_transform_sub = rospy.Subscriber("/fid_pub", TransformStamped, self.fid_cb) #subscribes to the fiducial transform node


    # callbacks =====
    def color_cb(self, msg):
        self.colorTwist = msg

    def fid_cb(self, msg):
        
        if(msg.child_frame_id = target_fiducial):
            ct = msg.transform.translation
            cr = msg.transform.rotation
            print ("T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                                (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w))

            # Set the state varibles to the position of the fiducial
            self.fid_x = ct.x
            self.fid_y = ct.y

            # Calculate the error in the x and y directions
            forward_error = self.fid_x - self.min_dist
            lateral_error = self.fid_y

            # Calculate the amount of turning needed towards the fiducial
            # atan2 works for any point on a circle (as opposed to atan)
            angular_error = math.atan2(self.fid_y, self.fid_x)


            fiducialTwist.linear.x = forward_error * self.linear_rate
            fiducialTwist.angular.z = angular_error * self.angular_rate - angSpeed / 2.0




    def print_state(self):
        ps = "State: "+self.state + "   TargetColor: " + self.target_color + "TargetFid: " + self.target_fiducial
        # ps += "\n Pos X undef, Pos Y undef, LinSpd:"+str(self.LinSpd)+" AngSpd:"+str(self.AngSpd)
        # ps += "\n Other vars here. Remaining: "+str(self.remaining)
        print(ps)



    def run(self):

        print("starting run")
        print(self.state)

        while not rospy.is_shutdown():
            self.print_state()
            twist = Twist()

            if self.state == "find_item":
                #find item
                twist = self.colorTwist

            else if self.state == "deliver_item":
                # deliver item

                #set target fiducial
                if self.target_color == "red":
                    self.target_fiducial = red_target_fiducial
                else:
                    self.target_fiducial = green_target_fiducial
                
                twist = fiducialTwist




            else if self.state == "return_to_start":
                #return to starting position




            self.cmdPub.publish(twist)


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

