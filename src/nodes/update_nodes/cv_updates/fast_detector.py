# !/usr/bin/env python3


import rospy
import numpy as np
import cv2
import math
import torch
import time

from ...nodes.update import Update

from .models.detector_models import COCO_Detector_Accurate, COCO_Detector_Fast


def imgmsg_to_np(img_msg):
        
    img = np.fromstring(img_msg.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)

    return img


class FastDetector(Update):

    def __init__(self, label_dict_var_name, camera_var_name, detection_var_name):

        self.model = COCO_Detector_Fast()

        self.camera_var_name = camera_var_name
        self.detection_var_name = detection_var_name

        self.label_dict_var_name = label_dict_var_name


    def update_blackboard(self, blackboard:dict) -> str:

        try:

            blackboard[self.label_dict_var_name] = self.model.label_list

            imgmsg = blackboard[self.camera_var_name]

            img = imgmsg_to_np(imgmsg)

            output = self.model.forward(img)

            blackboard[self.detection_var_name] = output
            

            return "success"

        except:

            return "failure"

