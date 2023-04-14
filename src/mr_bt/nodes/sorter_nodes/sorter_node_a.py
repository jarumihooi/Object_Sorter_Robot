import rospy
from src.nodes.nodes.update import Update
import rospy, cv2, cv_bridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
import math
import numpy as np
from image_tools import ImageTools




class SorterNodeA(Update):
    def ang_vel_control(self, x):
        return -1/(1+math.e**x) + 0.5
    
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.logcount = 0
        self.lostcount = 0
        # no pubs needed

    # is this override? 
    def update_blackboard(self, blackboard) -> str:
        # print('img_callback')

        converter = ImageTools()
        msg = blackboard["/raspicam_node/image/compressed"]
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

            # draw the biggest contour (c) in green
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

        # show the images
        cv2.imshow("Result", np.hstack([image, output]))
        cv2.waitKey(3)
        found_centroid = False
        if not found_centroid:
            blackboard["centroid"] = null
            return "failure"
        else :
            blackboard["centroid"] = [3,3]
            return "success"
