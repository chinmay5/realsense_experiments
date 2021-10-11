import pyrealsense2 as rs2
import numpy as np
import cv2
import logging

# The context object to be shared by all the
context_obj = rs2.context()

# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs2.pipeline(context_obj)
config_1 = rs2.config()
config_1.enable_device('047322071398')
config_1.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
config_1.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)

# ...from Camera 2
pipeline_2 = rs2.pipeline(context_obj)
config_2 = rs2.config()
config_2.enable_device('048522075021')
config_2.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
config_2.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)


# config_1.resolve(pipeline_1).get_device().query_sensors().set_option(rs2.stream   RS2_OPTION_INTER_CAM_SYNC_MODE, 1.f)

# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)

try:
    while True:

        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue
        cv2.imshow('PC', np.asarray(pc.map_to(color_frame_1)))

        # Camera 1
        # Wait for a coherent pair of frames: depth and color

        # frames_1 = pipeline_1.wait_for_frames()
        # depth_frame_1 = frames_1.get_depth_frame()
        # color_frame_1 = frames_1.get_color_frame()
        # if not depth_frame_1 or not color_frame_1:
        #     continue
        # # Convert images to numpy arrays
        # depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        # color_image_1 = np.asanyarray(color_frame_1.get_data())
        # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)
        #
        # # Let us get the camera extrinsics here
        # depth_to_color_extrin_1 = depth_frame_1.profile.get_extrinsics_to(color_frame_1.profile)
        #
        # # Camera 2
        # # Wait for a coherent pair of frames: depth and color
        # frames_2 = pipeline_2.wait_for_frames()
        # depth_frame_2 = frames_2.get_depth_frame()
        # color_frame_2 = frames_2.get_color_frame()
        # if not depth_frame_2 or not color_frame_2:
        #     continue
        # # Convert images to numpy arrays
        # depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        # color_image_2 = np.asanyarray(color_frame_2.get_data())
        # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.5), cv2.COLORMAP_JET)
        #
        #
        # # color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin_1, depth_frame_2)
        #
        #
        # # Stack all images horizontally
        # # images = np.hstack((color_image_1, depth_colormap_1,color_image_2, depth_colormap_2))
        #
        # # Show images from both cameras
        # cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        # cv2.imshow('RealSense', points)
        # cv2.waitKey(1)

        # Save images and depth maps from both cameras by pressing 'ds'
        ch = cv2.waitKey(25)
        # if ch == 115:
        #     cv2.imwrite("my_image_1.jpg",color_image_1)
        #     cv2.imwrite("my_depth_1.jpg",depth_colormap_1)
        #     cv2.imwrite("my_image_2.jpg",color_image_2)
        #     cv2.imwrite("my_depth_2.jpg",depth_colormap_2)
        #     print("Save")
        if ch == 27:
            break


finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()