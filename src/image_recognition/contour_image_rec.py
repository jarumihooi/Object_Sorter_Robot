#!/usr/bin/env python

import rospy, cv2, cv_bridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
import math
import numpy as np
from std_msgs.msg import Bool, String

from image_tools import ImageTools


def ang_vel_control(x):
    return -1/(1+math.e**x) + 0.5



class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, buff_size=2**24)
        #self.image_sub = rospy.Subscriber('/raspicam_node/image', Image, self.image_callback)

        self.centroid_pub = rospy.Publisher('centroid', CompressedImage, queue_size=1)
        self.direction_pub = rospy.Publisher('/color_direction_twist', Twist, queue_size=1)

        #TODO for testing only, remove after
        self.servo_pub = rospy.Publisher('/servo', Bool, queue_size=1)#publishes commands to the claw

        self.color_pub = rospy.Subscriber('/color', String, self.set_color_cb)
        self.target_color = "green"


        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def set_color_cb(self, msg):
        self.target_color = msg.data
        print(self.target_color)

    def image_callback(self, msg):
        # print('img_callback')

        #to prevent errors in the print statement when there are no contours on screen
        w=0

        #self.servo_pub.publish(True)

        converter = ImageTools()

        imagemsg = converter.convert_to_ros_msg(msg)

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(imagemsg)

        # np_arr = np.fromstring(msg.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #ideal color is 0, 100, 95 #prevously 160-180
        lower_red = np.array([170,80,80])
        upper_red = np.array([190,255,255])

            #50-90 ideal
        lower_green = np.array([30,80,80])
        upper_green = np.array([85,255,255])
        
        lower_target = lower_green
        upper_target = upper_green


        if self.target_color == "red":
            lower_target = lower_red
            upper_target = upper_red


        # the mask
        mask = cv2.inRange(hsv, lower_target, upper_target)
        output = cv2.bitwise_and(image, image, mask=mask)

        cv2.imshow("mask", mask)

        cx = 0
        cy = 0

        ret,thresh = cv2.threshold(mask, 40, 255, 0)
        if (int(cv2.__version__[0]) > 3):
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(output, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            cx = x + (w/2)
            cy = y + (h/2)

            # draw the biggest contour (c) in green
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

        # show the images
        cv2.circle(image, (int(cx), int(cy)), 8, (255,255,0), -1)

        print("rect width:   " + str(w))

        #following test**********************
        h2, w2, d2 = image.shape
        
        #sets unused twist value to one to signal main file to close claw once close enough
        #ideal value so far is 220
        if(w > 200):
            self.twist.linear.y = 1.0
            print("closed")
        elif w > 120 and (cx < 70 or cx > 330):
            self.twist.linear.y = 1.0
            print("closed")
        else:
            self.twist.linear.y = 0

        
        err = cx - w2/2
        ang_vel = ang_vel_control(-float(err) / 100)
        print("err"+str(err))

        print("cx:  " + str(cx))


        print("ang_vel= "+str(ang_vel)) #I turned this off for launch testing too. 
            
        self.twist.angular.z = ang_vel
        self.direction_pub.publish(self.twist) 
        #*************************

        cv2.imshow("Result", np.hstack([image, output]))
        cv2.waitKey(3)


print('running')
rospy.init_node('follower')
follower = Follower()
rospy.spin()
