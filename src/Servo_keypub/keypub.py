#!/usr/bin/env python
"""
This code publishes the key that the user inputs on the keyboard to the claw_control.py file. 

Written by David Pollack, adapted from key_publisher.py from Arjun Albert
"""
import sys, select, tty, termios
import rospy
from std_msgs.msg import String
#These are all of the recongnizable keys that can be published 
bindings = {
    'o': 'o', #Open the claw
    'c': 'c', #Close the claw
    'w': 'w', #Move forward
    's': 's', #Move backwards
    'a': 'a', #Turn left
    'd': 'd', #Turn right
    'h': 'h', #STOP MOMOMENTUM WILL MAKE THE ROBOT NOT STOP COMPLEATLY
    'q': 'q', #Go diagonally NW
    'e': 'e', #Go diagonally NE
    'z': 'z', #Go diagonally SW
    'x': 'x', #Go diagonally SE
}
# This code (if ran as executable code), will publish the keys pressed if they are inside of the bindings list.
if __name__ == '__main__q':
    key_pub = rospy.Publisher('keys', String, queue_size=1) 
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)

    old_attr = termios.tcgetattr(sys.stdin) 
    tty.setcbreak(sys.stdin.fileno())
    print("Publishing keystrokes. Press Ctrl-C to exit...")

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1)
            if key in bindings:
                key_pub.publish(bindings[key])
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)