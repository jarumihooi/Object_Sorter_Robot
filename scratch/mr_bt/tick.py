#!/usr/bin/env python3

import rospy

from std_msgs.msg import Byte





if __name__ == "__main__":

    rospy.init_node("ticker")

    pub = rospy.Publisher('/tick', Byte, queue_size=1)

    tps = rospy.get_param("rate")

    r = rospy.Rate(15)

    while not rospy.is_shutdown():
        
        pub.publish(True)

        r.sleep()
        





