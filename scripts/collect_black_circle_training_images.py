#!/usr/bin/env python3

from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class BlackCircleTrainingImageCollector(Node):
    def __init__(self) -> None:
        super().__init__("black_circle_training_image_collector")

        self.declare_parameter(
            "image_topic",
            "/white_line_hsv_input_remap_node/image_remapped",
        )
        self.declare_parameter("output_dir", "dataset_raw/images")
        self.declare_parameter("metadata_path", "dataset_raw/metadata.csv")
        self.declare_parameter("save_every_n_frames", 10)
        self.declare_parameter("save_interval_sec", 0.0)
        self.declare_parameter("max_images", 0)
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("png_compression", 3)
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("save_pose_metadata", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self.metadata_path = Path(self.get_parameter("metadata_path").value).expanduser()
        self.save_every_n_frames = max(
            0, int(self.get_parameter("save_every_n_frames").value)
        )
        self.save_interval_sec = max(
            0.0, float(self.get_parameter("save_interval_sec").value)
        )
        self.max_images = max(0, int(self.get_parameter("max_images").value))
        self.image_format = self._normalize_image_format(
            str(self.get_parameter("image_format").value)
        )
        self.jpeg_quality = max(0, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.png_compression = max(
            0, min(9, int(self.get_parameter("png_compression").value))
        )
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.save_pose_metadata = bool(self.get_parameter("save_pose_metadata").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.last_save_time: Optional[rclpy.time.Time] = None
        self.latest_pose: Optional[PoseWithCovarianceStamped] = None

        self.metadata_file = self.metadata_path.open("a", newline="", encoding="utf-8")
        self.metadata_writer = csv.DictWriter(
            self.metadata_file,
            fieldnames=[
                "filename",
                "frame_index",
                "saved_index",
                "image_stamp_sec",
                "image_stamp_nanosec",
                "wall_time_utc",
                "image_topic",
                "encoding",
                "width",
                "height",
                "pose_available",
                "pose_topic",
                "pose_stamp_sec",
                "pose_stamp_nanosec",
                "pose_age_sec",
                "pose_x",
                "pose_y",
                "pose_z",
                "pose_qx",
                "pose_qy",
                "pose_qz",
                "pose_qw",
            ],
        )
        if self.metadata_path.stat().st_size == 0:
            self.metadata_writer.writeheader()
            self.metadata_file.flush()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, sensor_qos
        )

        self.pose_sub = None
        if self.save_pose_metadata and self.pose_topic:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                10,
            )

        self.get_logger().info(
            "Collecting remapped training images from '%s' into '%s'. "
            "save_every_n_frames=%d, save_interval_sec=%.3f, max_images=%d",
            self.image_topic,
            str(self.output_dir),
            self.save_every_n_frames,
            self.save_interval_sec,
            self.max_images,
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "metadata_file") and not self.metadata_file.closed:
            self.metadata_file.flush()
            self.metadata_file.close()
        return super().destroy_node()

    @staticmethod
    def _normalize_image_format(value: str) -> str:
        normalized = value.lower().lstrip(".")
        if normalized == "jpeg":
            normalized = "jpg"
        if normalized not in {"jpg", "png"}:
            raise ValueError("image_format must be 'jpg' or 'png'")
        return normalized

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def should_save(self, msg_time: rclpy.time.Time) -> bool:
        if self.max_images > 0 and self.saved_count >= self.max_images:
            return False

        frame_rule_enabled = self.save_every_n_frames > 0
        time_rule_enabled = self.save_interval_sec > 0.0

        if not frame_rule_enabled and not time_rule_enabled:
            return True

        frame_due = (
            frame_rule_enabled and self.frame_count % self.save_every_n_frames == 0
        )
        time_due = False
        if time_rule_enabled:
            if self.last_save_time is None:
                time_due = True
            else:
                elapsed_sec = (msg_time - self.last_save_time).nanoseconds / 1e9
                time_due = elapsed_sec >= self.save_interval_sec

        return frame_due or time_due

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)

        if not self.should_save(msg_time):
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().warn(f"Could not convert image: {error}")
            return

        stamp_text = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        filename = (
            f"black_circle_{stamp_text}_frame_{self.frame_count:06d}."
            f"{self.image_format}"
        )
        output_path = self.output_dir / filename

        params = []
        if self.image_format == "jpg":
            params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        elif self.image_format == "png":
            params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression]

        if not cv2.imwrite(str(output_path), image, params):
            self.get_logger().error(f"Failed to save image: {output_path}")
            return

        self.saved_count += 1
        self.last_save_time = msg_time
        self.write_metadata(filename, msg, image.shape)

        self.get_logger().info(
            "Saved %s (%d images total, frame %d)",
            str(output_path),
            self.saved_count,
            self.frame_count,
        )

        if self.max_images > 0 and self.saved_count >= self.max_images:
            self.get_logger().info("Reached max_images=%d; shutting down.", self.max_images)
            rclpy.shutdown()

    def write_metadata(self, filename: str, msg: Image, image_shape: tuple[int, ...]) -> None:
        height, width = image_shape[:2]
        pose = self.latest_pose
        pose_available = pose is not None

        row = {
            "filename": filename,
            "frame_index": self.frame_count,
            "saved_index": self.saved_count,
            "image_stamp_sec": msg.header.stamp.sec,
            "image_stamp_nanosec": msg.header.stamp.nanosec,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "image_topic": self.image_topic,
            "encoding": msg.encoding,
            "width": width,
            "height": height,
            "pose_available": pose_available,
            "pose_topic": self.pose_topic if self.save_pose_metadata else "",
            "pose_stamp_sec": "",
            "pose_stamp_nanosec": "",
            "pose_age_sec": "",
            "pose_x": "",
            "pose_y": "",
            "pose_z": "",
            "pose_qx": "",
            "pose_qy": "",
            "pose_qz": "",
            "pose_qw": "",
        }

        if pose_available:
            pose_stamp = rclpy.time.Time.from_msg(pose.header.stamp)
            image_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            position = pose.pose.pose.position
            orientation = pose.pose.pose.orientation
            row.update(
                {
                    "pose_stamp_sec": pose.header.stamp.sec,
                    "pose_stamp_nanosec": pose.header.stamp.nanosec,
                    "pose_age_sec": f"{(image_stamp - pose_stamp).nanoseconds / 1e9:.6f}",
                    "pose_x": f"{position.x:.9f}",
                    "pose_y": f"{position.y:.9f}",
                    "pose_z": f"{position.z:.9f}",
                    "pose_qx": f"{orientation.x:.9f}",
                    "pose_qy": f"{orientation.y:.9f}",
                    "pose_qz": f"{orientation.z:.9f}",
                    "pose_qw": f"{orientation.w:.9f}",
                }
            )

        self.metadata_writer.writerow(row)
        self.metadata_file.flush()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = BlackCircleTrainingImageCollector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
