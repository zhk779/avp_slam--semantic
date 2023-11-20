#!/usr/bin/env python3
#coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
import numpy as np

#seg model 
import torch
import yaml
import math
from torch import Tensor
from torch.nn import functional as F
from pathlib import Path
from torchvision import io
from torchvision import transforms as T
import sys
import os
sys.path.insert(0, './')
from semseg.models import *
from semseg.datasets import *
from semseg.utils.utils import timer

import time

class SemSeg:
    def __init__(self, cfg) -> None:
        # inference device cuda or cpu
        self.device = torch.device(cfg['DEVICE'])
        # get dataset classes' colors
        self.palette = eval(cfg['DATASET']['NAME']).PALETTE
        # initialize the model and load weights and send to device
        self.model = eval(cfg['MODEL']['NAME'])(cfg['MODEL']['BACKBONE'], len(self.palette))
        self.model.load_state_dict(torch.load(cfg['TEST']['MODEL_PATH'], map_location='cpu'))
        self.model = self.model.to(self.device)
        self.model.eval()
        # preprocess parameters
        self.size = cfg['TEST']['IMAGE_SIZE']
        self.norm = T.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
        self.bridge = CvBridge()

    def resize(self, image: Tensor) -> Tensor:
        H, W = image.shape[1:]
        # scale the short side of image to target size
        scale_factor = self.size[0] / min(H, W)
        nH, nW = round(H*scale_factor), round(W*scale_factor)
        # make it divisible by model stride
        nH, nW = int(math.ceil(nH / 32)) * 32, int(math.ceil(nW / 32)) * 32
        # resize the image
        image = T.Resize((nH, nW))(image)
        return image

    def preprocess(self, image: Tensor) -> Tensor:
        # resize the image to inference image size
        img = self.resize(image)
        # img = img[[2, 1, 0], ...]     # RGB to BGR (for hardnet)
        # scale to [0.0, 1.0]
        img = img.float() / 255
        # normalize
        img = self.norm(img)
        # add batch size and send to device
        img = img.unsqueeze(0).to(self.device)
        return img

    def postprocess(self, seg_map: Tensor, orig_size: list) -> Tensor:
        # resize to original image size
        seg_map = F.interpolate(seg_map, size=orig_size, mode='bilinear', align_corners=True)
        # get segmentation map (value being 0 to num_classes)
        seg_map = seg_map.softmax(dim=1).argmax(dim=1).to(int)
        # convert segmentation map to color map
        seg_map = self.palette[seg_map]
        return seg_map.squeeze().cpu().permute(2, 0, 1)

    @torch.no_grad()
    @timer
    def model_forward(self, img: Tensor) -> Tensor:
        return self.model(img)
        
    def predict(self, image_message: Image, overlay: bool) -> Tensor:
        ########## utililze ros_numpy #############
        # image = ros_numpy.numpify(image_message)
        # image = np.ascontiguousarray(image[:,:,:3])
        # image = torch.from_numpy(image).permute(2,0,1)

        ########## utililze cv_bridge #############
        image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        image = torch.from_numpy(image).permute(2,0,1)
        rospy.loginfo(image.shape)
        
        img = self.preprocess(image)
        seg_map = self.model_forward(img)
        seg_map = self.postprocess(seg_map, image.shape[-2:])
        if overlay: seg_map = (image * 0.4) + (seg_map * 0.6)
        return seg_map.to(torch.uint8)


def seg_callback(img, args):
    semseg, save_dir, test_file = args[0], args[1], args[2]
    rospy.loginfo("receive an image!")
    segmap = semseg.predict(img, cfg['TEST']['OVERLAY'])
    save_dir = str(save_dir) + "/{:06}.png".format(time.time())
    io.write_png(segmap, str(save_dir))


# def seg_callback(img):
#     rospy.logwarn("receive an image!")
#     print("receive an image!")

if __name__ == "__main__":
    '''init model'''
    with open("/home/snow/Documents/workspace/slam_ws/src/segmodel_pkg/scripts/configs/parkingslotsSFNet.yaml") as f:
        cfg = yaml.load(f, Loader=yaml.SafeLoader)

    test_file = Path("/home/snow/Documents/workspace/slam_ws/src/segmodel_pkg/scripts/" + cfg['TEST']['FILE'])
    save_dir = Path("/home/snow/Documents/workspace/slam_ws/src/segmodel_pkg/scripts/" + cfg['SAVE_DIR']) / 'test_results'
    save_dir.mkdir(exist_ok=True)

    semseg = SemSeg(cfg)

    rospy.init_node("seginfer_node")
    rospy.logwarn("seg model is running wait for image")

    # sub = rospy.Subscriber("camera/image", Image, seg_callback, queue_size=10)
    sub = rospy.Subscriber("camera/image", Image, seg_callback, (semseg, save_dir, test_file), queue_size=10)

    rospy.spin()