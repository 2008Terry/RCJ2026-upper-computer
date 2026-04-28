#!/usr/bin/env python3

import math
import os
import time

import cv2
import numpy as np
import rclpy
import serial
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class OpenMVImagePublisher(Node):
    FRAME_MAGIC = b"\xAA\xBB"

    def __init__(self):
        super().__init__("openmv_camera")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("max_frame_size", 1048576)
        self.declare_parameter("wait_log_period_sec", 2.0)
        self.declare_parameter("frame_log_interval", 30)
        self.declare_parameter("enable_image_view", False)
        self.declare_parameter("show_published_image", True)
        self.declare_parameter("display_max_width", 960)
        self.declare_parameter("display_max_height", 720)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.max_frame_size = self.get_parameter("max_frame_size").get_parameter_value().integer_value
        self.wait_log_period_sec = self.get_parameter("wait_log_period_sec").get_parameter_value().double_value
        self.frame_log_interval = self.get_parameter("frame_log_interval").get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self.serial = serial.Serial(self.port, baudrate=int(self.baudrate), timeout=float(self.timeout_sec))
        self.timer = self.create_timer(0.01, self.publish_frame)
        self.waiting_since = time.monotonic()
        self.last_wait_log_time = 0.0
        self.frame_count = 0
        self.enable_image_view = False
        self.show_published_image = True
        self.display_max_width = 960
        self.display_max_height = 720
        self.display_available = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        self.headless_warned = False
        self.published_window_created = False
        self.published_window_name = f"{self.get_name()} published_image"

        self.sync_image_view_state()

        self.get_logger().info(
            "Listening for OpenMV JPEG frames on "
            f"{self.port} (baud={self.baudrate}, timeout={self.timeout_sec}s, frame_id={self.frame_id})"
        )

    def fit_within_bounds(self, frame):
        safe_max_width = max(1, self.display_max_width)
        safe_max_height = max(1, self.display_max_height)
        height, width = frame.shape[:2]

        if width <= 0 or height <= 0:
            return safe_max_width, safe_max_height

        width_scale = safe_max_width / width
        height_scale = safe_max_height / height
        scale = min(1.0, width_scale, height_scale)
        return max(1, int(math.floor(width * scale + 0.5))), max(1, int(math.floor(height * scale + 0.5)))

    def resize_window_to_fit_image(self, window_name, frame):
        fitted_width, fitted_height = self.fit_within_bounds(frame)
        cv2.resizeWindow(window_name, fitted_width, fitted_height)

    def sync_window(self, window_name, should_show, autosize=False):
        if should_show and not self.published_window_created:
            flag = cv2.WINDOW_AUTOSIZE if autosize else cv2.WINDOW_NORMAL
            cv2.namedWindow(window_name, flag)
            self.published_window_created = True
        elif not should_show and self.published_window_created:
            cv2.destroyWindow(window_name)
            self.published_window_created = False

    def sync_image_view_state(self):
        self.enable_image_view = self.get_bool_parameter("enable_image_view")
        self.show_published_image = self.get_bool_parameter("show_published_image")
        self.display_max_width = int(self.get_parameter("display_max_width").value)
        self.display_max_height = int(self.get_parameter("display_max_height").value)

        if self.enable_image_view and not self.display_available:
            if not self.headless_warned:
                self.get_logger().warn(
                    "enable_image_view=true but no DISPLAY/WAYLAND_DISPLAY is available; "
                    "disabling OpenCV image view for this process."
                )
                self.headless_warned = True
            self.enable_image_view = False

        self.sync_window(
            self.published_window_name,
            self.enable_image_view and self.show_published_image,
        )

    def get_bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def log_waiting(self, reason):
        now = time.monotonic()
        if now - self.last_wait_log_time < self.wait_log_period_sec:
            return

        waited = now - self.waiting_since
        self.last_wait_log_time = now
        self.get_logger().info(
            f"Still waiting for an OpenMV frame on {self.port} after {waited:.1f}s: {reason}"
        )

    def log_frame_received(self, width, height, length):
        self.frame_count += 1
        self.waiting_since = time.monotonic()
        self.last_wait_log_time = 0.0

        if self.frame_count == 1:
            self.get_logger().info(
                f"Received first OpenMV frame: {width}x{height}, jpeg_bytes={length}"
            )
            return

        if self.frame_log_interval > 0 and self.frame_count % self.frame_log_interval == 0:
            self.get_logger().info(
                f"Received frame #{self.frame_count}: {width}x{height}, jpeg_bytes={length}"
            )

    def read_exact(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.serial.read(size - len(data))
            if not chunk:
                self.log_waiting(f"timed out while reading {size} bytes")
                return None
            data.extend(chunk)
        return bytes(data)

    def read_frame(self):
        while rclpy.ok():
            first = self.serial.read(1)
            if not first:
                self.log_waiting("no serial data received yet")
                return None
            if first != self.FRAME_MAGIC[:1]:
                continue

            second = self.serial.read(1)
            if second != self.FRAME_MAGIC[1:]:
                continue

            header = self.read_exact(8)
            if header is None:
                return None

            width = int.from_bytes(header[0:2], "little")
            height = int.from_bytes(header[2:4], "little")
            length = int.from_bytes(header[4:8], "little")

            if width <= 0 or height <= 0 or length <= 0 or length > self.max_frame_size:
                self.get_logger().warn("Discarding invalid OpenMV frame header")
                continue

            jpeg = self.read_exact(length)
            if jpeg is None:
                return None

            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn("Failed to decode JPEG frame from OpenMV")
                continue

            if frame.shape[1] != width or frame.shape[0] != height:
                self.get_logger().warn(
                    f"Decoded frame size {frame.shape[1]}x{frame.shape[0]} does not match header {width}x{height}"
                )

            self.log_frame_received(width, height, length)
            return frame

        return None

    def publish_frame(self):
        frame = self.read_frame()
        if frame is None:
            return

        self.sync_image_view_state()

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self.frame_id
        camera_info.width = frame.shape[1]
        camera_info.height = frame.shape[0]
        camera_info.distortion_model = "plumb_bob"
        camera_info.k = [0.0] * 9
        camera_info.d = [0.0] * 5
        camera_info.r = [0.0] * 9
        camera_info.p = [0.0] * 12

        self.camera_info_pub.publish(camera_info)
        self.image_pub.publish(image_msg)

        if self.published_window_created:
            cv2.imshow(self.published_window_name, frame)
            self.resize_window_to_fit_image(self.published_window_name, frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.sync_window(self.published_window_name, False)
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenMVImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
