import os
import time

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum

from environment_setup import PROJECT_ROOT_DIR
from examples.read_multiple_cameras import AllCamerasLoop


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        # TODO: Remove the debugging part for later
        # user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        # if user_input.lower() == 'y':
        shutil.rmtree(path_folder)
        makedirs(path_folder)
        # else:
        #     exit()


def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)


def stream_for_camera(camera):
    # Streaming loop
    frame_count = 0
    try:
        while True:
            # Get frameset of color and depth
            ret, depth_image, color_image, color_frame = camera.get_frames()

            if args.record_imgs:
                if frame_count == 0:
                    save_intrinsic_as_json(
                        join(args.output_folder, "camera_intrinsic.json"),
                        color_frame)
                cv2.imwrite("%s/%06d.png" % \
                            (path_depth, frame_count), depth_image)
                cv2.imwrite("%s/%06d.jpg" % \
                            (path_color, frame_count), color_image)
                print("Saved color + depth image %06d" % frame_count)
                frame_count += 1

            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Recorder Realsense', images)
            key = cv2.waitKey(1)

            # if 'esc' button pressed, escape loop and exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=
        "Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--output_folder",
                        default='dataset/realsense/',
                        help="set output folder")

    parser.add_argument("--record_rosbag",
                        action='store_true',
                        help="Recording rgbd stream into realsense.bag")
    parser.add_argument(
        "--record_imgs",
        default=True,
        # action='store_true',
        help="Recording save color and depth images into realsense folder")
    parser.add_argument("--playback_rosbag",
                        action='store_true',
                        help="Play recorded realsense.bag file")
    args = parser.parse_args()

    print("rosbag mode not implemented yet")

    if sum(o is not False for o in vars(args).values()) != 2:
        parser.print_help()
        exit()

    path_output = os.path.join(PROJECT_ROOT_DIR, args.output_folder)
    path_depth = join(args.output_folder, "depth")
    path_color = join(args.output_folder, "color")
    if args.record_imgs:
        make_clean_folder(path_output)
        make_clean_folder(path_depth)
        make_clean_folder(path_color)

    path_bag = join(args.output_folder, "realsense.bag")
    if args.record_rosbag:
        if exists(path_bag):
            user_input = input("%s exists. Overwrite? (y/n) : " % path_bag)
            if user_input.lower() == 'n':
                exit()

    # At this point, we should start looking at the different devices we have and start configuring them
    all_cameras_present = AllCamerasLoop().get_all_conected_cameras()


    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = all_cameras_present[0]._RealsenseCamera__config
    pipeline = all_cameras_present[0]._RealsenseCamera__pipeline
    profile = all_cameras_present[0].profile

    # Hopefully already done
    # if args.record_imgs or args.record_rosbag:
    #     # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #     #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    #     config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    #     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #     if args.record_rosbag:
    #         config.enable_record_to_file(path_bag)
    # if args.playback_rosbag:
    #     config.enable_device_from_file(path_bag, repeat_playback=True)

    # Start streaming
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.depth_units, 0.0001)

    # Using preset HighAccuracy for recording
    if args.record_rosbag or args.record_imgs:
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    while True:
        # time.sleep(10) # sleep for 10 seconds
        # TODO: improve this part
        user_input = input("Press y when ready")
        if user_input != 'y':
            break
        for camera in all_cameras_present:
            stream_for_camera(camera)