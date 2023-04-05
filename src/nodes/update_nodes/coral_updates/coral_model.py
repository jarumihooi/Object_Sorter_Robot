#!/usr/bin/env python3
from ...nodes.update import Update

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import pycoral.adapters.common as common
import pycoral.adapters.detect as detect
import tflite_runtime.interpreter as tflite

from PIL import Image as im
import numpy as np
import cv2
from cv_bridge import CvBridge

import os


class CoralModel(Update):

    def __init__(self, detection_key: str, model_path: str):

        # All tensors and tensor operations will be performed on the GPU if one is available.
        
        self.key = detection_key

        self.interpreter = tflite.Interpreter(model_path,
        experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

        self.interpreter.allocate_tensors()
        self.size = common.input_size(self.interpreter)


        self.label_dict = self.make_label_dict()

        # self.pub = rospy.Publisher("/boxes", Image, queue_size = 10)

        self.bridge = CvBridge()


    def make_label_dict(self):
        with open(dir + "labels/coco_labels.txt") as f:
            label_dict = {}
            lines = f.readlines()

            for line in lines:
                split = line.split()
                id, label = split[0], split[1]
                label_dict[int(id)] = label

            return label_dict

    def draw_boxes(self, img, output):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (255, 0, 0)
        thickness = 1
        for out in output:
            bbox = out.bbox
            label = self.label_dict[out.id]
            img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
            org = (bbox.xmin, bbox.ymin)
            img = cv2.putText(img, label, org, font, 
                fontScale, color, thickness, cv2.LINE_AA)

        return img


    def run_model(self, image):

        img = im.fromarray(image).convert('RGB').resize(self.size, im.ANTIALIAS)
        
        common.set_input(self.interpreter, img)
        self.interpreter.invoke()
        