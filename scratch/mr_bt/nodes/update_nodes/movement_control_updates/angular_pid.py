#!/usr/bin/env python3

import rospy

from ...nodes.update import Update


'''
This class adds a variable to the blackboard which specifies an angular velocity for which
to turn so that the robot can track an object.
'''
class AngularPid(Update):


    def __init__(self, pid_err_var_name, nearest_dist_var_name, nearest_angle_var_name, kp, kd, kp2, dwall, offset=0):

        super().__init__()

        self.pid_err_var_name = pid_err_var_name
        self.nearest_dist_var_name = nearest_dist_var_name
        self.nearest_angle_var_name = nearest_angle_var_name

        self.kp = kp
        self.kd = kd
        self.kp2 = kp2

        self.dwall = dwall
        self.offset = offset

        self.dmin_prev = None
        self.tn_prev = rospy.Time.now().to_sec()


    def update_blackboard(self, blackboard:dict) -> str:

        try:

            tn = rospy.Time.now().to_sec()
            amin = blackboard[self.nearest_angle_var_name]

            dmin = blackboard[self.nearest_dist_var_name]
            if self.dmin_prev is None:
                self.dmin_prev = dmin
                
            wall_err = dmin - self.dwall
            wall_err_prev = self.dmin_prev - self.dwall

            PDct = self.kp * wall_err + self.kd * ( (wall_err - wall_err_prev) / (tn - self.tn_prev + 1e-10) )

            if amin >= 3.1415:
                di = -1
            else:
                di = 1

            a_err = amin - (self.offset)*di

            Pct = self.kp2*a_err

            final_angular_velocity = (PDct + Pct)
            
            blackboard[self.pid_err_var_name] = final_angular_velocity*di

            self.tn_prev = tn
            self.dmin_prev = dmin

            return "success"

        except:

            return "failure"
