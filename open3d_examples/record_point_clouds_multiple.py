# Inspired from https://github.com/ivomarvan/samples_and_experiments/blob/master/Multiple_realsense_cameras/multiple_realsense_cameras.py
import math
import os.path
import time

import pyrealsense2 as rs
import cv2
import numpy as np

from PIL import ImageFont, ImageDraw, Image

# --- Realsence problem core -------------------------------------------------------------------------------------------
from environment_setup import PROJECT_ROOT_DIR


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
        self.create_storage_folders()
        # Attributes specific to creation of point clouds
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

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
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)  # stream type, resolution, format and frame rate
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)  # We can check the possible values from the realsense-viewer
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

    def create_pointcloud(self, seq_index):
        # Configure depth and color streams
        # pipeline = rs.pipeline()
        # config = rs.config()
        #
        # pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        # pipeline_profile = config.resolve(pipeline_wrapper)
        # device = pipeline_profile.get_device()
        #
        # found_rgb = False
        # for s in device.sensors:
        #     if s.get_info(rs.camera_info.name) == 'RGB Camera':
        #         found_rgb = True
        #         break
        # if not found_rgb:
        #     print("The demo requires Depth camera with Color sensor")
        #     exit(0)
        #
        # config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        #
        # # Start streaming
        # pipeline.start(config)

        # Get stream profile and camera intrinsics
        # profile = self.__pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** self.decimate)
        colorizer = rs.colorizer()

        def mouse_cb(event, x, y, flags, param):

            if event == cv2.EVENT_LBUTTONDOWN:
                self.mouse_btns[0] = True

            if event == cv2.EVENT_LBUTTONUP:
                self.mouse_btns[0] = False

            if event == cv2.EVENT_RBUTTONDOWN:
                self.mouse_btns[1] = True

            if event == cv2.EVENT_RBUTTONUP:
                self.mouse_btns[1] = False

            if event == cv2.EVENT_MBUTTONDOWN:
                self.mouse_btns[2] = True

            if event == cv2.EVENT_MBUTTONUP:
                self.mouse_btns[2] = False

            if event == cv2.EVENT_MOUSEMOVE:

                h, w = out.shape[:2]
                dx, dy = x - self.prev_mouse[0], y - self.prev_mouse[1]

                if self.mouse_btns[0]:
                    self.yaw += float(dx) / w * 2
                    self.pitch -= float(dy) / h * 2

                elif self.mouse_btns[1]:
                    dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
                    self.translation -= np.dot(self.rotation, dp)

                elif self.mouse_btns[2]:
                    dz = math.sqrt(dx ** 2 + dy ** 2) * math.copysign(0.01, -dy)
                    self.translation[2] += dz
                    self.distance -= dz

            if event == cv2.EVENT_MOUSEWHEEL:
                dz = math.copysign(0.1, flags)
                self.translation[2] += dz
                self.distance -= dz

            self.prev_mouse = (x, y)

        cv2.namedWindow(self.__name, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(self.__name, w, h)
        cv2.setMouseCallback(self.__name, mouse_cb)

        def project(v):
            """project 3d vector array to 2d"""
            h, w = out.shape[:2]
            view_aspect = float(h) / w

            # ignore divide by zero for invalid depth
            with np.errstate(divide='ignore', invalid='ignore'):
                proj = v[:, :-1] / v[:, -1, np.newaxis] * \
                       (w * view_aspect, h) + (w / 2.0, h / 2.0)

            # near clipping
            znear = 0.03
            proj[v[:, 2] < znear] = np.nan
            return proj

        def view(v):
            """apply view transformation on vector array"""
            return np.dot(v - self.pivot, self.rotation) + self.pivot - self.translation

        def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
            """draw a 3d line from pt1 to pt2"""
            p0 = project(pt1.reshape(-1, 3))[0]
            p1 = project(pt2.reshape(-1, 3))[0]
            if np.isnan(p0).any() or np.isnan(p1).any():
                return
            p0 = tuple(p0.astype(int))
            p1 = tuple(p1.astype(int))
            rect = (0, 0, out.shape[1], out.shape[0])
            inside, p0, p1 = cv2.clipLine(rect, p0, p1)
            if inside:
                cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)

        def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
            """draw a grid on xz plane"""
            pos = np.array(pos)
            s = size / float(n)
            s2 = 0.5 * size
            for i in range(0, n + 1):
                x = -s2 + i * s
                line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
                       view(pos + np.dot((x, 0, s2), rotation)), color)
            for i in range(0, n + 1):
                z = -s2 + i * s
                line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
                       view(pos + np.dot((s2, 0, z), rotation)), color)

        def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
            """draw 3d axes"""
            line3d(out, pos, pos +
                   np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
            line3d(out, pos, pos +
                   np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
            line3d(out, pos, pos +
                   np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)

        def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
            """draw camera's frustum"""
            orig = view([0, 0, 0])
            w, h = intrinsics.width, intrinsics.height

            for d in range(1, 6, 2):
                def get_point(x, y):
                    p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
                    line3d(out, orig, view(p), color)
                    return p

                top_left = get_point(0, 0)
                top_right = get_point(w, 0)
                bottom_right = get_point(w, h)
                bottom_left = get_point(0, h)

                line3d(out, view(top_left), view(top_right), color)
                line3d(out, view(top_right), view(bottom_right), color)
                line3d(out, view(bottom_right), view(bottom_left), color)
                line3d(out, view(bottom_left), view(top_left), color)

        def pointcloud(out, verts, texcoords, color, painter=True):
            """draw point cloud with optional painter's algorithm"""
            if painter:
                # Painter's algo, sort points from back to front

                # get reverse sorted indices by z (in view-space)
                # https://gist.github.com/stevenvo/e3dad127598842459b68
                v = view(verts)
                s = v[:, 2].argsort()[::-1]
                proj = project(v[s])
            else:
                proj = project(view(verts))

            if self.scale:
                proj *= 0.5 ** self.decimate

            h, w = out.shape[:2]

            # proj now contains 2d image coordinates
            j, i = proj.astype(np.uint32).T

            # create a mask to ignore out-of-bound indices
            im = (i >= 0) & (i < h)
            jm = (j >= 0) & (j < w)
            m = im & jm

            cw, ch = color.shape[:2][::-1]
            if painter:
                # sort texcoord with same indices as above
                # texcoords are [0..1] and relative to top-left pixel corner,
                # multiply by size and add 0.5 to center
                v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
            else:
                v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
            # clip texcoords to image
            np.clip(u, 0, ch - 1, out=u)
            np.clip(v, 0, cw - 1, out=v)

            # perform uv-mapping
            out[i[m], j[m]] = color[u[m], v[m]]

        out = np.empty((h, w, 3), dtype=np.uint8)

        while True:
            # Grab camera data
            if not self.paused:
                # Wait for a coherent pair of frames: depth and color
                frames = self.__pipeline.wait_for_frames()

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                depth_frame = decimate.process(depth_frame)

                # Grab new intrinsics (may be changed by decimation)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w, h = depth_intrinsics.width, depth_intrinsics.height

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_colormap = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data())

                if self.color:
                    mapped_frame, color_source = color_frame, color_image
                else:
                    mapped_frame, color_source = depth_frame, depth_colormap

                points = pc.calculate(depth_frame)
                pc.map_to(mapped_frame)

                # Pointcloud data to arrays
                v, t = points.get_vertices(), points.get_texture_coordinates()
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
                texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

            # Render
            now = time.time()

            out.fill(0)

            grid(out, (0, 0.5, 1), size=1, n=10)
            frustum(out, depth_intrinsics)
            axes(out, view([0, 0, 0]), self.rotation, size=0.1, thickness=1)

            if not self.scale or out.shape[:2] == (h, w):
                pointcloud(out, verts, texcoords, color_source)
            else:
                tmp = np.zeros((h, w, 3), dtype=np.uint8)
                pointcloud(tmp, verts, texcoords, color_source)
                tmp = cv2.resize(
                    tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                np.putmask(out, tmp > 0, tmp)

            if any(self.mouse_btns):
                axes(out, view(self.pivot), self.rotation, thickness=4)

            dt = time.time() - now

            cv2.setWindowTitle(
                self.__name, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
                                (w, h, 1.0 / dt, dt * 1000, "PAUSED" if self.paused else ""))

            cv2.imshow(self.__name, out)
            key = cv2.waitKey(1)

            if key == ord("e"):
                points.export_to_ply(f'out{seq_index}.ply', mapped_frame)

            if key in (27, ord("q")) or cv2.getWindowProperty(self.__name, cv2.WND_PROP_AUTOSIZE) < 0:
                break

    def create_storage_folders(self):
        base_dir = os.path.join(PROJECT_ROOT_DIR, 'dataset', self.__serial_number)
        if not os.path.exists(base_dir):
            os.makedirs(base_dir, exist_ok=True)
            os.makedirs(os.path.join(base_dir, 'point_clouds'), exist_ok=True)
        self.point_cloud_root_folder = os.path.join(PROJECT_ROOT_DIR, self.__serial_number, 'point_clouds')


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
            # original_depth_table = rs.rs400_advanced_mode(d).get_depth_table()
            # # Change the depth clamping to a range ensuring we do not go much beyond the immediate viscinity
            # original_depth_table.depthClampMax = 800
            # rs.rs400_advanced_mode(d).set_depth_table(original_depth_table)
            # print(f"depth value changed to {original_depth_table.depthClampMax}")
            # if camera_name_suffix and not name.endswith(camera_name_suffix):
            #     continue
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

    def get_point_clouds(self):
        for idx, camera in enumerate(self.__cameras):
            camera.create_pointcloud(seq_index=idx)


if __name__ == "__main__":
    viewer = AllCamerasLoop()
    print(viewer.get_conected_cameras_info(camera_name_suffix='D435I'))
    # viewer.run_loop()
    viewer.get_point_clouds()
