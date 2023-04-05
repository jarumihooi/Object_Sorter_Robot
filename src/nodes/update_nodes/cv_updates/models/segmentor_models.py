#!/usr/bin/env python3
import rospy
from .model_parent import Model

from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.rpn import AnchorGenerator
from torchvision.models.detection import FasterRCNN
from PIL import Image
import numpy as np
import torchvision.transforms as T
import torch
import torchvision
from time import time

import rospkg



class Segmentor(Model):


    def __init__(self, trf=None):

        super().__init__()

        if trf != None:
            self.trf = trf 
        else:
            self.trf = T.Compose([
                T.ToTensor(),
                T.Normalize(mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225])
            ])

        self.sigmoid = torch.nn.Sigmoid()


    def forward(self, img):

        inpt = self.trf(img).to(self.device).unsqueeze(0)
        out = self.model(inpt)['out']
        masks = out.squeeze().detach()
        return self.sigmoid(masks)


class COCO_Segmentor(Segmentor):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.label_dict = {
            0: 'background',
            1: 'aeroplane', 
            2: 'bicycle', 
            3: 'bird', 
            4: 'boat', 
            5: 'bottle', 
            6: 'bus',
            7: 'car', 
            8: 'cat', 
            9: 'chair', 
            10: 'cow', 
            11: 'diningtable', 
            12: 'dog', 
            13: 'horse', 
            14: 'motorbike',
            15: 'person', 
            16: 'pottedplant', 
            17: 'sheep', 
            18: 'sofa', 
            19: 'train', 
            20: 'tvmonitor'
        }

        self.label_list = list(self.label_dict.values())


class COCO_Segmentor_Accurate(COCO_Segmentor):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.model = torchvision.models.segmentation.fcn_resnet101(pretrained=True)

        self.model.to(self.device)

        print("Model is running on {}".format(self.device))

        self.model.eval()


class COCO_Segmentor_Fast(COCO_Segmentor):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.model = torchvision.models.segmentation.lraspp_mobilenet_v3_large(pretrained=True)

        self.model.to(self.device)

        print("Model is running on {}".format(self.device))

        self.model.eval()