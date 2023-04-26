#!/usr/bin/env python

import rospy, cv2, cv_bridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
import math
import numpy as np
from image_tools import ImageTools


def ang_vel_control(x):
    return -1/(1+math.e**x) + 0.5



class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        #self.image_sub = rospy.Subscriber('/raspicam_node/image', Image, self.image_callback)

        self.centroid_pub = rospy.Publisher('centroid', CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_callback(self, msg):
        # print('img_callback')

        converter = ImageTools()

        imagemsg = converter.convert_to_ros_msg(msg)

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(imagemsg)

        # np_arr = np.fromstring(msg.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 40, 0, 0]) 
        upper_yellow = np.array([ 120, 255, 255])

        lower_red = np.array([160,50,50])
        upper_red = np.array([180,255,255])
        
        # mask = cv2.inRange(hsv,  lower_red, upper_red)
        # masked = cv2.bitwise_and(image, image, mask=mask)
        # cv2.imshow("mask", mask)
        # cv2.imshow("masked", masked)

        # find the colors within the specified boundaries and apply

        # the mask
        mask = cv2.inRange(hsv, lower_red, upper_red)
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

        #following test**********************
        h, w, d = image.shape

        
        err = cx - w/2
        self.twist.linear.x = 0.2
        ang_vel = ang_vel_control(-float(err) / 100)

        print("ang_vel= "+str(ang_vel))
            
        self.twist.angular.z = ang_vel
        #self.cmd_vel_pub.publish(self.twist) I TURNED THIS OFF ISAAC -DAVID (<3)
        #*************************

        cv2.imshow("Result", np.hstack([image, output]))
        cv2.waitKey(3)

    # clear all but a 20 pixel band near the top of the image
        # h, w, d = image.shape
        # search_top = int(3 * h /4)
        # search_bot = search_top + 20
        # mask[0:search_top, 0:w] = 0
        # mask[search_bot:h, 0:w] = 0
        


    # # Compute the "centroid" and display a red circle to denote it
    #     M = cv2.moments(mask)
    #     self.logcount += 1
    #     print("M00 %d %d" % (M['m00'], self.logcount))
        
        
    #     if M['m00'] > 0:
    #         cx = int(M['m10']/M['m00']) + 100
    #         cy = int(M['m01']/M['m00'])
    #         cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

    #         # Move at 0.2 M/sec
    #         # add a turn if the centroid is not in the center
    #         # Hope for the best. Lots of failure modes.
    #         err = cx - w/2
    #         self.twist.linear.x = 0.2
    #         ang_vel = ang_vel_control(-float(err) / 100)

    #         print("ang_vel= "+str(ang_vel))
            
    #         self.twist.angular.z = ang_vel
    #         #self.cmd_vel_pub.publish(self.twist)
    #         #self.centroid_pub.publish(self.bridge.cv2_to_imgmsg(image))
        
    #     else:
    #         self.twist.linear.x = 0
    #         self.twist.angular.z = 0.3
    #         print("twist= " +str(self.twist.angular.z))
    #       #  self.cmd_vel_pub.publish(self.twist)

        # cv2.imshow("image", image)
        # cv2.waitKey(3)

print('running')
rospy.init_node('follower')
follower = Follower()
rospy.spin()
