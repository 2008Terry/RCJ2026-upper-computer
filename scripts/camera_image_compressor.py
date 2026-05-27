#!/usr/bin/env python3

import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraImageCompressor(Node):
    def __init__(self):
        super().__init__("camera_image_compressor")
        self.bridge = CvBridge()
        self.image_sub = None
        self.last_publish_time = 0.0
        self.published_frames = 0

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_raw/compressed")
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("debug_jpeg_quality", 80)
        self.declare_parameter("max_fps", 0.0)
        self.declare_parameter("debug_image_max_fps", 0.0)
        self.declare_parameter("lazy", False)
        self.declare_parameter("log_size_stats", True)
        self.declare_parameter("size_log_interval", 30)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(
            CompressedImage,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription_timer = self.create_timer(0.25, self.sync_subscription)
        self.sync_subscription()

        self.get_logger().info(
            f"camera_image_compressor started. input_topic={self.input_topic}, "
            f"output_topic={self.output_topic}"
        )

    def sync_subscription(self):
        lazy = bool(self.get_parameter("lazy").value)
        should_subscribe = (not lazy) or self.publisher.get_subscription_count() > 0

        if should_subscribe and self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image,
                self.input_topic,
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Subscribed to {self.input_topic}.")
        elif not should_subscribe and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.last_publish_time = 0.0
            self.get_logger().info(
                f"No compressed image subscribers; unsubscribed from {self.input_topic}."
            )

    def image_callback(self, msg):
        if self.publisher.get_subscription_count() == 0 and bool(
            self.get_parameter("lazy").value
        ):
            return

        max_fps = self.get_float_parameter("max_fps", "debug_image_max_fps")
        now = time.monotonic()
        if max_fps > 0.0 and self.last_publish_time > 0.0:
            if now - self.last_publish_time < 1.0 / max_fps:
                return
        self.last_publish_time = now

        try:
            image, compressed_encoding = self.image_msg_to_jpeg_input(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return
        except ValueError as exc:
            self.get_logger().warn(f"Invalid image buffer: {exc}")
            return

        quality = self.get_int_parameter("jpeg_quality", "debug_jpeg_quality")
        quality = max(1, min(100, quality))
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            self.get_logger().warn("Failed to JPEG-compress camera image.")
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = f"{msg.encoding}; jpeg compressed {compressed_encoding}"
        compressed.data = encoded.tobytes()
        self.publisher.publish(compressed)
        self.published_frames += 1
        self.log_size_stats(msg, len(compressed.data), quality)

    def image_msg_to_jpeg_input(self, msg):
        encoding = msg.encoding.lower()

        if encoding in ("bgr8", "rgb8"):
            image = self.image_from_padded_rows(msg, channels=3)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(image), "bgr8"

        if encoding == "mono8":
            image = self.image_from_padded_rows(msg, channels=1)
            return np.ascontiguousarray(image), "mono8"

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return np.ascontiguousarray(image), "bgr8"

    @staticmethod
    def image_from_padded_rows(msg, channels):
        bytes_per_pixel = channels
        active_row_bytes = msg.width * bytes_per_pixel
        if msg.step < active_row_bytes:
            raise ValueError(
                f"step {msg.step} is smaller than active row bytes {active_row_bytes}"
            )

        expected_size = msg.height * msg.step
        if len(msg.data) < expected_size:
            raise ValueError(
                f"data has {len(msg.data)} bytes, expected at least {expected_size}"
            )

        rows = np.frombuffer(msg.data, dtype=np.uint8, count=expected_size).reshape(
            msg.height, msg.step
        )
        active = rows[:, :active_row_bytes]
        if channels == 1:
            return active.reshape(msg.height, msg.width)
        return active.reshape(msg.height, msg.width, channels)

    def log_size_stats(self, msg, compressed_bytes, quality):
        if not bool(self.get_parameter("log_size_stats").value):
            return

        interval = max(1, int(self.get_parameter("size_log_interval").value))
        if self.published_frames % interval != 0:
            return

        raw_bytes = len(msg.data)
        if compressed_bytes <= 0:
            ratio_text = "n/a"
            saving_text = "n/a"
        else:
            ratio = raw_bytes / compressed_bytes
            saving = 100.0 * (1.0 - compressed_bytes / max(1, raw_bytes))
            ratio_text = f"{ratio:.2f}x"
            saving_text = f"{saving:.1f}%"

        self.get_logger().info(
            "image size: "
            f"raw={self.format_bytes(raw_bytes)} ({raw_bytes} B), "
            f"jpeg={self.format_bytes(compressed_bytes)} ({compressed_bytes} B), "
            f"ratio={ratio_text}, saving={saving_text}, "
            f"quality={quality}, encoding={msg.encoding}, "
            f"size={msg.width}x{msg.height}, step={msg.step}"
        )

    @staticmethod
    def format_bytes(byte_count):
        value = float(byte_count)
        for unit in ("B", "KiB", "MiB"):
            if value < 1024.0 or unit == "MiB":
                return f"{value:.1f} {unit}"
            value /= 1024.0

    def get_int_parameter(self, primary_name, fallback_name):
        primary = self.get_parameter(primary_name).value
        fallback = self.get_parameter(fallback_name).value
        return int(primary if primary is not None else fallback)

    def get_float_parameter(self, primary_name, fallback_name):
        primary = self.get_parameter(primary_name).value
        fallback = self.get_parameter(fallback_name).value
        return float(primary if primary is not None else fallback)


def main(args=None):
    rclpy.init(args=args)
    node = CameraImageCompressor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
