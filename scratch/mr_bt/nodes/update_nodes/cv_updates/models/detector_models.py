#!/usr/bin/env python3
import rospy
from .model_parent import Model

import torch
import numpy as np
import torchvision

import torchvision.transforms as T
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.rpn import AnchorGenerator
from torchvision.models.detection import FasterRCNN


class Detector(Model):
    # Parent class for all detector models

    def __init__(self, trf=None):

        super().__init__()

        # If a custom transform is passed, then that will be applied to the numpy array as it is 
        # passed through .forward(). Otherwise, the default transform will be used, which converts
        # numpy.ndarray of shape (H, W, C) to a torch tensor of shape (C, H, W).
        if trf != None:
            self.trf = trf 
        else:
            self.trf = T.Compose([
                T.ToTensor()
            ])


    def forward(self, img):
        # Preprocessing the input image. Designed to work with a typical numpy.ndarray
        inpt = self.trf(img).to(self.device).unsqueeze(0)

        # Passing preprocessed image into model
        output = self.model(inpt)[0]
        dict_output = {}
        dict_output['boxes'] = output['boxes'].detach().cpu()
        dict_output['scores'] = output['scores'].detach().cpu()
        dict_output['labels'] = output['labels'].detach().cpu()

        return dict_output



class COCO_Detector(Detector):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.label_dict = {
            0: 'background',
            1: 'person',
            2: 'bicycle',
            3: 'car',
            4: 'motorcycle',
            5: 'airplane',
            6: 'bus',
            7: 'train',
            8: 'truck',
            9: 'boat',
            10: 'traffic light',
            11: 'fire hydrant',
            12: 'N/A',
            13: 'stop sign',
            14: 'parking meter',
            15: 'bench',
            16: 'bird',
            17: 'cat',
            18: 'dog',
            19: 'horse',
            20: 'sheep',
            21: 'cow',
            22: 'elephant',
            23: 'bear',
            24: 'zebra',
            25: 'giraffe',
            26: 'N/A',
            27: 'backpack',
            28: 'umbrella',
            29: 'N/A',
            30: 'N/A',
            31: 'handbag',
            32: 'tie',
            33: 'suitcase',
            34: 'frisbee',
            35: 'skis',
            36: 'snowboard',
            37: 'sports ball',
            38: 'kite',
            39: 'baseball bat',
            40: 'baseball glove',
            41: 'skateboard',
            42: 'surfboard',
            43: 'tennis racket',
            44: 'bottle',
            45: 'N/A',
            46: 'wine glass',
            47: 'cup',
            48: 'fork',
            49: 'knife',
            50: 'spoon',
            51: 'bowl',
            52: 'banana',
            53: 'apple',
            54: 'sandwich',
            55: 'orange',
            56: 'broccoli',
            57: 'carrot',
            58: 'hot dog',
            59: 'pizza',
            60: 'donut',
            61: 'cake',
            62: 'chair',
            63: 'couch',
            64: 'potted plant',
            65: 'bed',
            66: 'N/A',
            67: 'dining table',
            68: 'N/A',
            69: 'N/A',
            70: 'toilet',
            71: 'N/A',
            72: 'tv',
            73: 'laptop',
            74: 'mouse',
            75: 'remote',
            76: 'keyboard',
            77: 'cell phone',
            78: 'microwave',
            79: 'oven',
            80: 'toaster',
            81: 'sink',
            82: 'refrigerator',
            83: 'N/A',
            84: 'book',
            85: 'clock',
            86: 'vase',
            87: 'scissors',
            88: 'teddy bear',
            89: 'hair drier',
            90: 'toothbrush'
        }

        self.label_list = list(self.label_dict.values())


class COCO_Detector_Fast(COCO_Detector):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=True)

        self.model.to(self.device)

        print("Model is running on {}".format(self.device))

        self.model.eval()


class COCO_Detector_Accurate(COCO_Detector):

    def __init__(self, trf=None):

        super().__init__(trf=trf)

        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)

        self.model.to(self.device)

        print("Model is running on {}".format(self.device))

        self.model.eval()


class PersonFace_Detector(Detector):

    def __init__(self, trf=None):

        super().__init__(trf)

        print('Loading Mobilenetv3 backbone...')

        mobilenet = torchvision.models.mobilenet_v3_small(pretrained=True)

        backbone = mobilenet.features

        backbone.out_channels = 576

        print('Loading Anchor Generator...')

        anchor_generator = AnchorGenerator(sizes=((31,64,128,256,512),), aspect_ratios=((0.5, 1.0, 2.0),))

        print('Loading ROI pooler...')

        roi_pooler = torchvision.ops.MultiScaleRoIAlign(featmap_names=['0'], output_size=7, sampling_ratio=2)

        print('Loading FasterRCNN')

        self.model = FasterRCNN(backbone, 3, rpn_anchor_generator=anchor_generator, box_roi_pool=roi_pooler)

        print('Loading model state...')

        # state_dict_path = self.rospack.get_path('mr_cv/model_state_dicts/mobilenet_v3_state_dict_pytorch.pth')

        real_path = 'src/mr_cv/model_state_dicts/mobilenet_v3_state_dict_pytorch.pth'

        self.model.load_state_dict(torch.load(real_path))

        print('Finished loading model.')

        self.model.to(self.device)

        print("Model is running on {}".format(self.device))

        self.model.eval()
        
        self.label_dict = {
            0: 'background', 
            1: 'person', 
            2: 'face'
        }

        self.label_list = list(self.label_dict.values())

