import os

import time

import math

import cv2
import pyrealsense2 as rs
import numpy as np
from PIL import Image

from environment_setup import PROJECT_ROOT_DIR
from utils.realsense_depth import DepthCamera

point = (400, 200)

def get_points(event, x, y, args, params):
    global point
    point = (x, y)


cv2.namedWindow("color")
cv2.setMouseCallback("color", get_points)

def create_stream():
    dc = DepthCamera()
    frame_id, saved_images = 0, 0
    while True:
        frame_id += 1
        ret, depth_frame, color_frame = dc.get_frame()
        cv2.imshow("color", color_frame)
        key = cv2.waitKey(1)
        while key != 1:
            key = cv2.waitKey(1)
        if frame_id % 50 == 49:
            im = Image.fromarray(color_frame)
            im.save(os.path.join(PROJECT_ROOT_DIR, 'images', f'color_frame_{saved_images}.jpg'))
            saved_images += 1
        # exit condition
        if saved_images == 50:
            print("Exiting the program now")
            break


if __name__ == '__main__':
    create_stream()