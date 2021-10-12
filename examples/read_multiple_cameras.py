# Inspired from https://github.com/ivomarvan/samples_and_experiments/blob/master/Multiple_realsense_cameras/multiple_realsense_cameras.py


import pyrealsense2 as rs
import cv2
import numpy as np

from PIL import ImageFont, ImageDraw, Image


# --- Realsence problem core -------------------------------------------------------------------------------------------
class RealsenseCamera:
    '''
    Abstraction of any RealsenseCamera
    '''
    __colorizer = rs.colorizer()

    def __init__(
            self,
            serial_number: str,
            name: str
    ):
        self.__serial_number = serial_number
        self.__name = name
        self.__pipeline = None
        self.__started = False
        self.__start_pipeline()

    def __del__(self):
        if self.__started and not self.__pipeline is None:
            self.__pipeline.stop()

    def get_full_name(self):
        return f'{self.__name} ({self.__serial_number})'

    def __start_pipeline(self):
        # Configure depth and color streams
        self.__pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.__serial_number)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)  # stream type, resolution, format and frame rate
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # We can check the possible values from the realsense-viewer
        # Let us also limit the depth camera can see
        self.__config = config  # Storing the config object for possible later use
        self.profile = self.__pipeline.start(config)
        self.__started = True
        print(f'{self.get_full_name()} camera is ready.')

    def get_frames(self) -> [rs.frame]:
        align_to = rs.stream.color
        align = rs.align(align_to)

        frames = self.__pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not aligned_depth_frame or not color_frame:
            return False, None, None, None
        return True, depth_image, color_image, color_frame


class AllCamerasLoop:
    '''
    Take info from all conected cameras in the loop.
    '''

    @classmethod
    def get_conected_cameras_info(cls, camera_name_suffix: str = 'T265') -> [(str, str)]:
        '''
        Return list of (serial number,names) conected devices.
        Eventualy only fit given suffix (like T265, D415, ...)
        (based on https://github.com/IntelRealSense/librealsense/issues/2332)
        '''
        ret_list = []
        ctx = rs.context()
        for d in ctx.devices:
            serial_number = d.get_info(rs.camera_info.serial_number)
            name = d.get_info(rs.camera_info.name)
            # Attempts
            original_depth_table = rs.rs400_advanced_mode(d).get_depth_table()
            # Change the depth clamping to a range ensuring we do not go much beyond the immediate viscinity
            original_depth_table.depthClampMax = 500
            rs.rs400_advanced_mode(d).set_depth_table(original_depth_table)
            print(f"depth value changed to {original_depth_table.depthClampMax}")
            if camera_name_suffix and not name.endswith(camera_name_suffix):
                continue
            ret_list.append((serial_number, name))
        return ret_list

    @classmethod
    def get_all_conected_cameras(cls) -> [RealsenseCamera]:
        cameras = cls.get_conected_cameras_info(camera_name_suffix=None)
        return [RealsenseCamera(serial_number, name) for serial_number, name in cameras]

    def __init__(self):
        self.__cameras = self.get_all_conected_cameras()

    def get_frames(self, camera) -> [rs.frame]:
        '''
        Return frames in given order.
        '''
        while True:
            ret, depth_frame, color_frame, _ = camera.get_frames()
            cv2.imshow("color", color_frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

    def run_loop(self):
        for camera in self.__cameras:
            self.get_frames(camera)


if __name__ == "__main__":
    viewer = AllCamerasLoop()
    print(viewer.get_conected_cameras_info(camera_name_suffix='D435I'))
    viewer.run_loop()
