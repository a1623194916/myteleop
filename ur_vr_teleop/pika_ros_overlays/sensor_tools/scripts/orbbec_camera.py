#!/home/dell/anaconda3/envs/pika/bin/python3
"""Orbbec Gemini 336L ROS2 node - publishes color and depth images."""
import os
import ctypes

# Preload the correct libOrbbecSDK from pyorbbecsdk package to avoid
# loading the older version from /opt/ros/humble/lib.
_sdk_dir = os.path.join(
    os.path.dirname(__import__('pyorbbecsdk').__file__) if False else '',
)
# Compute the path before importing pyorbbecsdk
import importlib.util
_spec = importlib.util.find_spec('pyorbbecsdk')
if _spec and _spec.submodule_search_locations:
    _sdk_path = os.path.join(list(_spec.submodule_search_locations)[0], 'libOrbbecSDK.so.2')
    if os.path.exists(_sdk_path):
        ctypes.CDLL(_sdk_path, mode=ctypes.RTLD_GLOBAL)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import threading
import signal
import sys

from pyorbbecsdk import (
    Pipeline, Config, OBSensorType, OBFormat, OBAlignMode
)


class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__('orbbec_camera')

        # Declare parameters
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('align_depth_to_color', True)
        self.declare_parameter('camera_name', 'orbbec')
        self.declare_parameter('camera_frame_id', 'orbbec/camera_link')

        # Get parameters
        self.color_width = self.get_parameter('color_width').value
        self.color_height = self.get_parameter('color_height').value
        self.color_fps = self.get_parameter('color_fps').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.depth_fps = self.get_parameter('depth_fps').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.align_depth_to_color = self.get_parameter('align_depth_to_color').value
        self.camera_name = self.get_parameter('camera_name').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value

        # Publishers
        self.bridge = CvBridge()

        if self.enable_color:
            self.color_pub = self.create_publisher(
                Image, f'/{self.camera_name}/color/image_raw', 10)
            self.color_info_pub = self.create_publisher(
                CameraInfo, f'/{self.camera_name}/color/camera_info', 10)

        if self.enable_depth:
            if self.align_depth_to_color:
                self.depth_pub = self.create_publisher(
                    Image, f'/{self.camera_name}/aligned_depth_to_color/image_raw', 10)
                self.depth_info_pub = self.create_publisher(
                    CameraInfo, f'/{self.camera_name}/aligned_depth_to_color/camera_info', 10)
            else:
                self.depth_pub = self.create_publisher(
                    Image, f'/{self.camera_name}/depth/image_raw', 10)
                self.depth_info_pub = self.create_publisher(
                    CameraInfo, f'/{self.camera_name}/depth/camera_info', 10)

        # Pipeline
        self.pipeline = None
        self.running = False
        self.capture_timer = None

    def start_camera(self):
        """Initialize and start the Orbbec pipeline."""
        try:
            self.pipeline = Pipeline()
            config = Config()

            if self.enable_color:
                color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                # Try YUYV first since MJPG returns wrong format
                color_profile = None
                for fmt in [OBFormat.YUYV, OBFormat.MJPG, OBFormat.RGB, OBFormat.BGR]:
                    try:
                        color_profile = color_profiles.get_video_stream_profile(
                            self.color_width, self.color_height, fmt, self.color_fps)
                        break
                    except Exception:
                        continue

                if color_profile is None:
                    color_profile = color_profiles.get_default_video_stream_profile()
                    self.get_logger().warn(
                        f'Requested color profile not found, using default: '
                        f'{color_profile.get_width()}x{color_profile.get_height()}@{color_profile.get_fps()}fps')

                self.get_logger().info(
                    f'Color: {color_profile.get_width()}x{color_profile.get_height()}'
                    f'@{color_profile.get_fps()}fps format={color_profile.get_format()}')
                config.enable_stream(color_profile)

            if self.enable_depth:
                depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = None
                try:
                    depth_profile = depth_profiles.get_video_stream_profile(
                        self.depth_width, self.depth_height, OBFormat.Y16, self.depth_fps)
                except Exception:
                    depth_profile = depth_profiles.get_default_video_stream_profile()
                    self.get_logger().warn(
                        f'Requested depth profile not found, using default: '
                        f'{depth_profile.get_width()}x{depth_profile.get_height()}@{depth_profile.get_fps()}fps')

                self.get_logger().info(
                    f'Depth: {depth_profile.get_width()}x{depth_profile.get_height()}'
                    f'@{depth_profile.get_fps()}fps format={depth_profile.get_format()}')
                config.enable_stream(depth_profile)

            if self.align_depth_to_color and self.enable_depth and self.enable_color:
                config.set_align_mode(OBAlignMode.SW_MODE)

            self.pipeline.start(config)
            self.get_logger().info('Orbbec pipeline started successfully')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to start Orbbec camera: {e}')
            return False

    def capture_once(self):
        """Timer callback: grab and publish one frameset. Runs in main spin
        thread (same thread that created the pipeline), which is required by
        pyorbbecsdk for frames to be delivered."""
        try:
            frameset = self.pipeline.wait_for_frames(100)
            if frameset is None:
                return

            stamp = self.get_clock().now().to_msg()

            if self.enable_color:
                color_frame = frameset.get_color_frame()
                if color_frame is not None:
                    self._publish_color(color_frame, stamp)

            if self.enable_depth:
                depth_frame = frameset.get_depth_frame()
                if depth_frame is not None:
                    self._publish_depth(depth_frame, stamp)

        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    def _publish_color(self, frame, stamp):
        """Convert and publish color frame."""
        width = frame.get_width()
        height = frame.get_height()
        fmt = frame.get_format()
        data = np.asarray(frame.get_data())

        self.get_logger().debug(f'Color frame: {width}x{height}, data size={data.size}, fmt={fmt}')

        if fmt == OBFormat.MJPG:
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        elif fmt == OBFormat.RGB:
            img = data.reshape((height, width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif fmt == OBFormat.BGR:
            img = data.reshape((height, width, 3))
        elif fmt == OBFormat.YUYV or (data.size == height * width * 2):
            data = data.reshape((height, width, 2))
            img = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_YUYV)
        else:
            img = data.reshape((height, width, 3))

        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id + '_color'
        self.color_pub.publish(msg)

        # Publish camera info
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame_id + '_color'
        info.width = width
        info.height = height
        self.color_info_pub.publish(info)

    def _publish_depth(self, frame, stamp):
        """Convert and publish depth frame."""
        width = frame.get_width()
        height = frame.get_height()
        data = np.asarray(frame.get_data())

        # Depth is Y16 (uint16, in mm)
        depth_img = data.reshape((height, width))
        depth_img = depth_img.astype(np.uint16)

        msg = self.bridge.cv2_to_imgmsg(depth_img, '16UC1')
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id + '_depth'
        self.depth_pub.publish(msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame_id + '_depth'
        info.width = width
        info.height = height
        self.depth_info_pub.publish(info)

    def stop(self):
        """Stop capture and release resources."""
        self.running = False
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.get_logger().info('Orbbec camera stopped')


# Global for signal handler
_node_instance = None


def signal_handler(signum, frame):
    print(f'\nReceived signal {signum}, shutting down...')
    if _node_instance:
        _node_instance.stop()
    rclpy.shutdown()
    sys.exit(0)


def main():
    global _node_instance

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init()
    _node_instance = OrbbecCameraNode()

    if _node_instance.start_camera():
        _node_instance.get_logger().info('Orbbec camera node running')
        # Use a ROS timer (runs in the spin thread = pipeline's creation thread)
        # to poll frames. pyorbbecsdk only delivers frames on that thread.
        period = 1.0 / float(max(_node_instance.color_fps, 1)) / 2.0
        _node_instance.capture_timer = _node_instance.create_timer(
            period, _node_instance.capture_once)
        rclpy.spin(_node_instance)
    else:
        _node_instance.get_logger().error('Failed to initialize Orbbec camera')

    _node_instance.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
