#!/usr/bin/env python3

import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def format_bytes(byte_count):
    value = float(byte_count)
    for unit in ("B", "KiB", "MiB"):
        if value < 1024.0 or unit == "MiB":
            return f"{value:.1f} {unit}"
        value /= 1024.0


class ImageCompressor:
    def __init__(self):
        self.bridge = CvBridge()

    def compress_jpeg(self, msg, quality):
        image, compressed_encoding = self.image_msg_to_jpeg_input(msg)
        quality = max(1, min(100, int(quality)))

        start_time = time.perf_counter()
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        encode_ms = (time.perf_counter() - start_time) * 1000.0
        if not ok:
            raise RuntimeError("Failed to JPEG-compress image")

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = f"{msg.encoding}; jpeg compressed {compressed_encoding}"
        compressed.data = encoded.tobytes()
        return compressed, encode_ms

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


class ImageCompressionRelay:
    def __init__(
        self,
        node,
        compressor,
        name,
        input_topic,
        output_topic,
        jpeg_quality=80,
        max_fps=0.0,
        lazy=True,
        log_size_stats=True,
        size_log_interval=30,
    ):
        self.node = node
        self.compressor = compressor
        self.name = name
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.jpeg_quality = int(jpeg_quality)
        self.max_fps = float(max_fps)
        self.lazy = bool(lazy)
        self.log_size_stats_enabled = bool(log_size_stats)
        self.size_log_interval = max(1, int(size_log_interval))

        self.image_sub = None
        self.last_publish_time = 0.0
        self.published_frames = 0
        self.total_encode_ms = 0.0
        self.total_raw_bytes = 0
        self.total_compressed_bytes = 0

        self.publisher = node.create_publisher(
            CompressedImage,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription_timer = node.create_timer(0.25, self.sync_subscription)
        self.sync_subscription()

        node.get_logger().info(
            f"image relay '{self.name}' started. "
            f"input_topic={self.input_topic}, output_topic={self.output_topic}, "
            f"quality={self.jpeg_quality}, max_fps={self.max_fps}, lazy={self.lazy}"
        )

    def sync_subscription(self):
        should_subscribe = (not self.lazy) or self.publisher.get_subscription_count() > 0

        if should_subscribe and self.image_sub is None:
            self.image_sub = self.node.create_subscription(
                Image,
                self.input_topic,
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.node.get_logger().info(
                f"image relay '{self.name}' subscribed to {self.input_topic}."
            )
        elif not should_subscribe and self.image_sub is not None:
            self.node.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.last_publish_time = 0.0
            self.node.get_logger().info(
                f"image relay '{self.name}' has no subscribers; "
                f"unsubscribed from {self.input_topic}."
            )

    def image_callback(self, msg):
        if self.publisher.get_subscription_count() == 0 and self.lazy:
            return

        now = time.monotonic()
        if self.max_fps > 0.0 and self.last_publish_time > 0.0:
            if now - self.last_publish_time < 1.0 / self.max_fps:
                return
        self.last_publish_time = now

        try:
            compressed, encode_ms = self.compressor.compress_jpeg(msg, self.jpeg_quality)
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            self.node.get_logger().warn(
                f"image relay '{self.name}' failed to compress image: {exc}"
            )
            return

        self.publisher.publish(compressed)
        self.published_frames += 1
        self.total_encode_ms += encode_ms
        self.total_raw_bytes += len(msg.data)
        self.total_compressed_bytes += len(compressed.data)
        self.log_size_stats(msg, len(compressed.data), encode_ms)

    def log_size_stats(self, msg, compressed_bytes, encode_ms):
        if not self.log_size_stats_enabled:
            return
        if self.published_frames % self.size_log_interval != 0:
            return

        raw_bytes = len(msg.data)
        ratio = raw_bytes / max(1, compressed_bytes)
        saving = 100.0 * (1.0 - compressed_bytes / max(1, raw_bytes))
        avg_encode_ms = self.total_encode_ms / max(1, self.published_frames)
        avg_ratio = self.total_raw_bytes / max(1, self.total_compressed_bytes)

        self.node.get_logger().info(
            f"image relay '{self.name}' stats: "
            f"raw={format_bytes(raw_bytes)} ({raw_bytes} B), "
            f"jpeg={format_bytes(compressed_bytes)} ({compressed_bytes} B), "
            f"ratio={ratio:.2f}x, avg_ratio={avg_ratio:.2f}x, "
            f"saving={saving:.1f}%, encode={encode_ms:.2f} ms, "
            f"avg_encode={avg_encode_ms:.2f} ms, quality={self.jpeg_quality}, "
            f"encoding={msg.encoding}, size={msg.width}x{msg.height}, step={msg.step}"
        )


class ImageCompressionRelayNode(Node):
    def __init__(self, node_name="image_compression_relay"):
        super().__init__(
            node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.compressor = ImageCompressor()
        self.relays = []
        self.create_relays()

    def create_relays(self):
        relay_names = self.get_param("relay_names", "")
        if isinstance(relay_names, str):
            relay_names = [name.strip() for name in relay_names.split(",") if name.strip()]
        else:
            relay_names = [str(name).strip() for name in relay_names if str(name).strip()]

        if relay_names:
            for name in relay_names:
                self.relays.append(self.create_named_relay(name))
        else:
            self.relays.append(self.create_legacy_single_relay())

    def create_named_relay(self, name):
        prefix = f"relays.{name}."
        return ImageCompressionRelay(
            node=self,
            compressor=self.compressor,
            name=name,
            input_topic=self.get_param(prefix + "input_topic", "/camera/image_raw"),
            output_topic=self.get_param(prefix + "output_topic", f"/foxglove/{name}/image_compressed"),
            jpeg_quality=self.get_param(prefix + "jpeg_quality", self.get_param("jpeg_quality", 80)),
            max_fps=self.get_param(prefix + "max_fps", self.get_param("max_fps", 0.0)),
            lazy=self.get_param(prefix + "lazy", self.get_param("lazy", True)),
            log_size_stats=self.get_param(
                prefix + "log_size_stats", self.get_param("log_size_stats", True)
            ),
            size_log_interval=self.get_param(
                prefix + "size_log_interval", self.get_param("size_log_interval", 30)
            ),
        )

    def create_legacy_single_relay(self):
        return ImageCompressionRelay(
            node=self,
            compressor=self.compressor,
            name=self.get_param("relay_name", "camera"),
            input_topic=self.get_param("input_topic", "/camera/image_raw"),
            output_topic=self.get_param("output_topic", "/camera/image_compressed"),
            jpeg_quality=self.get_param(
                "jpeg_quality", self.get_param("debug_jpeg_quality", 80)
            ),
            max_fps=self.get_param("max_fps", self.get_param("debug_image_max_fps", 0.0)),
            lazy=self.get_param("lazy", True),
            log_size_stats=self.get_param("log_size_stats", True),
            size_log_interval=self.get_param("size_log_interval", 30),
        )

    def get_param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return default if value is None else value


def spin_image_compression_relay(node_name="image_compression_relay", args=None):
    rclpy.init(args=args)
    node = ImageCompressionRelayNode(node_name=node_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
