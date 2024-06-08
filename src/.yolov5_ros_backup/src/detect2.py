#!/usr/bin/env python3
# 참조: https://github.com/mats-robotics/yolov5_ros/blob/main/src/detect.py
# 아래 내용은 위 파일의 내용을 기반으로 필요한 부분만을 최소한으로 작성한 파일 입니다.

import sys
import os
import time
import rospy
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from pathlib import Path

from sensor_msgs.msg import Image
from yolov5_ros.msg import BoundingBox, BoundingBoxes

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    non_max_suppression,
    scale_boxes
)
from utils.torch_utils import select_device
from utils.augmentations import letterbox


class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold", 0.75)
        self.iou_thres = rospy.get_param("~iou_threshold", 0.45)
        self.agnostic_nms = rospy.get_param("~agnostic_nms", True)
        self.max_det = rospy.get_param("~maximum_detections", 1000)
        self.classes = rospy.get_param("~classes", None)
        # Initialize weights 
        weights = rospy.get_param("~weights", "yolov5s.pt")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","cpu")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn", True), data=rospy.get_param("~data", ""))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *self.img_size))  # warmup        
        
        # Initialize subscriber to Image topic
        rospy.Subscriber('/image_raw', Image, self.callback)
        self.pred_pub = rospy.Publisher(rospy.get_param("~output_topic", "/yolov5_ros"), BoundingBoxes, queue_size=10)

    def callback(self, img_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        img_cv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            img_cv = img_cv.byteswap().newbyteorder()
        im, im0 = self.preprocess(img_cv)

        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]
    
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = img_msg.header
        bounding_boxes.image_header = img_msg.header

        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)

                bbox_msg = BoundingBox()
                bbox_msg.Class = self.names[c]
                bbox_msg.probability = conf 
                bbox_msg.xmin = int(xyxy[0])
                bbox_msg.ymin = int(xyxy[1])
                bbox_msg.xmax = int(xyxy[2])
                bbox_msg.ymax = int(xyxy[3])

                bounding_boxes.bounding_boxes.append(bbox_msg)
        
        self.pred_pub.publish(bounding_boxes)
    
    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":
    rospy.init_node("yolov5", anonymous=True)

    detector = Yolov5Detector()
    
    rospy.spin()