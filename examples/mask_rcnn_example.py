import time

import math

import cv2
import pyrealsense2 as rs
import numpy as np

from utils.mask_rcnn import MaskRCNN
from utils.realsense_camera import RealsenseCamera
from utils.realsense_depth import DepthCamera

point = (400, 200)

def get_points(event, x, y, args, params):
    global point
    point = (x, y)


def create_stream():
    rs = RealsenseCamera()
    mrcnn = MaskRCNN()

    while True:
        ret, color_frame, depth_frame = rs.get_frame_stream()
        boxes, classes, contours, centers = mrcnn.detect_objects_mask(color_frame)
        color_frame = mrcnn.draw_object_mask(color_frame)
        mrcnn.draw_object_info(color_frame, depth_frame)
        key = cv2.waitKey(1)
        cv2.imshow("segmn_frame", color_frame)
        if key == 27:
            break


if __name__ == '__main__':
    create_stream()