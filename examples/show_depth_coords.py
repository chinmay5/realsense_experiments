import time

import math

import cv2
import pyrealsense2 as rs
import numpy as np

from utils.realsense_depth import DepthCamera

point = (400, 200)

def get_points(event, x, y, args, params):
    global point
    point = (x, y)


cv2.namedWindow("color")
cv2.setMouseCallback("color", get_points)

def create_stream():
    dc = DepthCamera()
    while True:
        ret, depth_frame, color_frame = dc.get_frame()
        cv2.circle(color_frame, point, 8, (0, 255, 0))
        distance = depth_frame[point[1], point[0]]
        cv2.putText(color_frame, f"{distance} mm", (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
        cv2.imshow("color", color_frame)
        cv2.imshow("depth", depth_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break


if __name__ == '__main__':
    create_stream()