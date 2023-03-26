#!/usr/bin/env python
'''Jeremy Huey, Isaac Goldings, David Pollack
Robotics Sp2023 Final Project - (Object) Sorter
v1J
The comment "bf" means bugfix: Its a note to myself as to something I fixed and why that bug occured. 
This class is the main executable for the project. It will set up the nodes required.
The expected nodes are: this, state-via_pytransitions, movement, color. 
This version adds basic structure. 
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
        self.state_sub = rospy.Subscriber('state', Int16, self.cb_state) #check state, this would get state if we had a diff file to give state. 
        # self.sub_pid_twist = rospy.Subscriber('pid_twist', Twist, self.cb_twist) #this would control pid if used.
        self.key_sub = rospy.Subscriber('keys', String, self.key_cb) #this gets single input keystrokes. #bf: needed to add self.
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb) #this gets odom info to improve perception, used for location. 
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb) 
        self.

        
        
        # State params: 
        self.prev_state = "h"
        self.state = "h"
        self.location = [0,0]
        self.PROX_DIST = 0.3 #0.8 for sim
        self.wall_sensed = {"F":False,"FR":False,"R":False,"BR":False, "B":False,"BL":False,"L":False,"FL":False}
        # self.vel_vec = [0,0]
        # self.spiral_turn = 0
        # self.cur_dist = 0.0

        # motion params: 
        self.LINEAR_SPEED = 0.4
        self.ANGULAR_SPEED = math.pi/4

        #Create two twist variable, one is modified here, one is copied from the PID messages
        self.twist = Twist()
        # self.t_pid = Twist()

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
        # clean data section ====


        # end clean data ====
      
         
        self.wall_sensed = {"F":False,"FR":False,"R":False,"BR":False, "B":False,"BL":False,"L":False,"FL":False}
                            #F    FR    R     BR     B     BL    L     FL
        
        if (self.state not in ["c", "h", "d"]):#and manual_col_override == False):
            #clean = lambda x: x if x < 0.03 else 4
            #clean_ranges = [i for i in msg.ranges]
            clean_ranges = list(np.where(np.array(msg.ranges)<0.03, 4, np.array(msg.ranges)))
            #prev_key_press = state
            # avg_arr = {} #ang: dist
            avg_arr = dict((i,j) for i,j in enumerate(clean_ranges))
            print("avg_arr", len(avg_arr))
            # for ang in range(len(msg.ranges)): #(-45,46)
            #     if msg.ranges[ang] < self.PROX_DIST:
            #         sum = 0
            #         for five_deg in range(-2,3):
            #             sum += msg.ranges[(ang+five_deg)%360] # next time try median.
            #         # end for
            #         avg_arr[ang] = sum / 5 #also can try using 7
            # end for 
            # check 
            for ang, dist in avg_arr.items(): #bf added .items()
                if dist < self.PROX_DIST : 
                    if ang >= 338:
                        self.wall_sensed["F"] = True
                    elif ang >= 293:
                        self.wall_sensed["FR"] = True
                    elif ang >= 248:
                        self.wall_sensed["R"] = True
                    elif ang >= 203:
                        self.wall_sensed["BR"] = True
                    elif ang >= 158:
                        self.wall_sensed["B"] = True
                    elif ang >= 113:
                        self.wall_sensed["BL"] = True
                    elif ang >= 68:
                        self.wall_sensed["L"] = True
                    elif ang >= 23:
                        self.wall_sensed["FL"] = True
                    else: #ang >= 0
                        self.wall_sensed["F"] = True
                    scan_time = rospy.Time.now().to_sec()
                    # print(self.wall_sensed)
            # end for
            self.cur_dist = min(msg.ranges[248:293])

                    # state = "c" #h
                    # #manual_col_override = True
                    # print("OBJECT PROXIMITY ERRR ERRR ERRR ERRR, dist = ", avgs, 
                    # "\n Moving to collision correction mode [c]")
                    # coll_det_T = rospy.Time.now().to_sec()
                    # coll_seq_time[0] = coll_time[0] + coll_det_T
                    # coll_seq_time[1] = coll_time[1] + coll_seq_time[0] #dont forget to make these array[ind] 
                    #state = "h"
                    #print("OBJECT PROXIMITY ERRR ERRR ERRR ERRR, dist = ", avgs)
            # end for
            #reutrn False #no avg val was under 0.3m. 
        #end if
    #end m()


    # ========================
    # ==== running code ======
    '''This method figures out what the state command is and does that. 
    Following wall on right of robot. '''
    # 'g' = go wall, 'a' = align, "p" = parallel
    def do_states(self, state):
        # stopping states ==========================================
        if (self.state ==  "h" ):
            self.vel_vec = [0,0]
        elif (self.state == "d"):
            print("Go to shutdown state")
            self.vel_vec = [0,0]
        # automation states ===================================
        elif(self.state == "g"):
            if ( self.wall_sensed["F"] and (self.wall_sensed["FR"] or self.wall_sensed["FR"]) ): #found wall
                print("found wall, aligning")
                self.state = "a"
                self.vel_vec = [0,0]
            else: #not found, continue

                self.vel_vec = [1,0] #go forward
        elif(self.state == "a"):
            if (self.wall_sensed["F"]): #still walled, rotate left
                self.vel_vec = [-0.1,1] #rot l
                print("rot l, aligning")
            elif ((not self.wall_sensed["F"]) and self.wall_sensed["R"]): # okay, now parallel.
                self.vel_vec = [0,0]
                self.state = "p"
                print("paralleled, moving along wall")
            else: 
                self.vel_vec = [0,1] #rot l
                print("rot l, still aligning")                   
                
        elif(self.state == "p"):
            if ((not self.wall_sensed["F"]) and self.wall_sensed["R"]): #still walled, rotate left
                self.vel_vec = [1,0]#self.t_pid.angular.z] #paralleling wall on right. 
                #self.vel_vec = [1,self.t_pid.angular.z]; print("self.t_pid.angular.z", self.t_pid.angular.z) # PID SECTION
            elif (self.wall_sensed["F"] ): #and self.wall_sensed["R"]): # hit R corner
                #self.vel_vec = [0,1]
                self.state = "a"
                print("cornered, move to align[a]")
            # elif (self.wall_sensed["BR"] and (not self.wall_sensed["F"]) and (not self.wall_sensed["FR"]) ):
            #     self.vel_vec = [0,-1] #rot r.
            #     print("Returning towards wall")
            # elif (all(value == False for value in self.wall_sensed.values()) ):
            #     self.last_pressed = rospy.Time.now().to_sec() #must reset last pressed time. 
            #     self.state = "s"
            #     print("Lost wall, spiralling.")
            elif ((not self.wall_sensed["F"]) and (not self.wall_sensed["R"])): #if we dont have the wall.
                self.vel_vec = [0.4,-0.75] #rot r.
                print("Returning towards wall")
            else: 
                print("I'm confused, halting. ")
                # BUG: THIS CAN OCCUR IF WE HIT THE WALL AT A WEIRD ANGLE OR ON THE LEFT
                self.state = "h"
        elif (self.state == "s"):
            if (all(value == False for value in self.wall_sensed.values()) ):
                self.spiral_turn = max(0.1, 2.0 + 0.05 * (self.last_pressed - self.curT)) #gradually reduces the spiraling. 
                # BUG: It might never find the wall and start to circle. 
                self.vel_vec = [0.75,-self.spiral_turn];# print(self.spiral_turn)
                print("Lost wall, spiralling right in search. Spiral =",self.spiral_turn)
            elif ( not all(value == False for value in self.wall_sensed.values()) ): #found wall
                print("found wall, aligning")
                #self.spiral_turn = 2.5 #reset??
                #self.vel_vec = [0,0]
                self.state = "a"
            else: 
                print("I'm confused in spiralling, halting. ")
                self.state = "h"

        #these are movement options
        elif (self.state == "l"):
            self.vel_vec = [0,1]
        elif (self.state == "r"):
            self.vel_vec = [0,-1]
        elif (self.state == "f"):
            self.vel_vec = [1,0]
        elif (self.state == "b"):
            self.vel_vec = [-1,0]
        # elif(self.vel_vec == "s"):
        #     self.spiral_turn = max(0.5, 2.5 + 0.1 * (self.last_pressed - self.curT)) #gradually reduces the spiraling. 
        #     self.vel_vec = [0.75,self.spiral_turn];# print(self.spiral_turn)
        
        #elif (self.state == "g" ):
            #Calculate and set appropriate t_pub values
        else:
            print("STATE NOT FOUND, setting to halt")
            self.state = "h"

        # if in command state:
        self.compute_pub_twist() #convert vel_vec to actual linear and angular twist commands. 
    #end m()

    '''convert vel_vec to actual linear and angular twist commands. '''
    def compute_pub_twist(self):
        self.t_pub.linear.x = self.LINEAR_SPEED * self.vel_vec[0] #bf forgot self. on everything
        self.t_pub.angular.z = self.ANGULAR_SPEED * self.vel_vec[1]

    '''This method prints the state of the system at Rate'''
    def print_state(self):
        print("---")
        print("STATE: " + self.state)
        if (self.state != ("h" or "d") ):
            speed = self.t_pub.linear.x
        else: 
            speed = 0.0
        print("location:", self.location, "speed:", speed)
        print("angle:", self.t_pub.angular.z)
        print(self.wall_sensed)
        if (self.state == "s"):
            print("Spiral angle:",self.spiral_turn)

        # calculate time since last key stroke
        #time_since = rospy.Time.now() - self.last_key_press_time
        #print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

    '''This method runs the program'''
    def run(self):
        print("Running()")
        while (not rospy.is_shutdown()) and (self.state != "d"):
            self.curT = rospy.Time.now().to_sec()
            self.print_state()
            self.do_states(self.state)
            self.pub_vel.publish(self.t_pub)
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
    print("STARTING control")
    cl = control()
    cl.run()
main()


'''
TODO: try and clean the scan. 
'''