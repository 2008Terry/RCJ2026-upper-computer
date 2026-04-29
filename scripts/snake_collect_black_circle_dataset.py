#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _ensure_import_path() -> None:
    script_dir = Path(__file__).resolve().parent
    source_helper_dir = script_dir.parent / "src"
    if source_helper_dir.exists():
        sys.path.insert(0, str(source_helper_dir))


_ensure_import_path()

from competition_robot import CompetitionRobot
from goto_point import GotoError


def _linspace_inclusive(start: float, stop: float, spacing: float) -> list[float]:
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    distance = abs(stop - start)
    steps = max(1, int(math.ceil(distance / spacing)))
    return [
        start + (stop - start) * (float(i) / float(steps))
        for i in range(steps + 1)
    ]


class SnakeBlackCircleDatasetCollector(CompetitionRobot):
    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("image_topic", "/white_line_hsv_input_remap_node/image_remapped")
        self.declare_parameter("output_dir", "dataset_raw/images")
        self.declare_parameter("metadata_path", "dataset_raw/snake_metadata.csv")
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("png_compression", 3)

        # The safe path bounds are computed from the field size, robot radius,
        # and a configurable extra wall buffer. Disable use_auto_safe_bounds to
        # use the explicit x/y min/max parameters below.
        self.declare_parameter("use_auto_safe_bounds", True)
        self.declare_parameter("map_width_m", 1.82)
        self.declare_parameter("map_height_m", 2.42)
        self.declare_parameter("robot_diameter_m", 0.21)
        self.declare_parameter("wall_buffer_m", 0.10)
        self.declare_parameter("x_min_m", -0.65)
        self.declare_parameter("x_max_m", 0.65)
        self.declare_parameter("y_min_m", -0.90)
        self.declare_parameter("y_max_m", 0.90)
        self.declare_parameter("column_spacing_m", 0.20)
        self.declare_parameter("waypoint_spacing_m", 0.30)

        self.declare_parameter("snake_goal_tolerance_m", 0.04)
        self.declare_parameter("snake_max_step_m", 0.20)
        self.declare_parameter("snake_settle_sec", 0.7)
        self.declare_parameter("snake_goto_timeout_sec", 35.0)
        self.declare_parameter("snake_max_iterations", 30)
        self.declare_parameter("snake_pose_wait_timeout_sec", 5.0)
        self.declare_parameter("snake_action_server_wait_sec", 10.0)

        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("motion_enable_on_start", True)
        self.declare_parameter("reset_yaw_on_start", False)
        self.declare_parameter("stop_on_exit", True)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("capture_while_moving", True)
        self.declare_parameter("moving_capture_interval_sec", 0.75)
        self.declare_parameter("images_per_waypoint", 2)
        self.declare_parameter("waypoint_capture_delay_sec", 0.15)
        self.declare_parameter("image_wait_timeout_sec", 3.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.metadata_path = Path(str(self.get_parameter("metadata_path").value)).expanduser()
        self.image_format = self._normalize_image_format(
            str(self.get_parameter("image_format").value)
        )
        self.jpeg_quality = max(0, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.png_compression = max(
            0, min(9, int(self.get_parameter("png_compression").value))
        )

        self.use_auto_safe_bounds = bool(self.get_parameter("use_auto_safe_bounds").value)
        self.map_width_m = self._positive_param("map_width_m")
        self.map_height_m = self._positive_param("map_height_m")
        self.robot_diameter_m = self._positive_param("robot_diameter_m")
        self.wall_buffer_m = self._non_negative_param("wall_buffer_m")
        self.path_wall_margin_m = (0.5 * self.robot_diameter_m) + self.wall_buffer_m

        if self.use_auto_safe_bounds:
            self.x_min_m = (-0.5 * self.map_width_m) + self.path_wall_margin_m
            self.x_max_m = (0.5 * self.map_width_m) - self.path_wall_margin_m
            self.y_min_m = (-0.5 * self.map_height_m) + self.path_wall_margin_m
            self.y_max_m = (0.5 * self.map_height_m) - self.path_wall_margin_m
        else:
            self.x_min_m = self._finite_float("x_min_m")
            self.x_max_m = self._finite_float("x_max_m")
            self.y_min_m = self._finite_float("y_min_m")
            self.y_max_m = self._finite_float("y_max_m")
        self.column_spacing_m = self._positive_param("column_spacing_m")
        self.waypoint_spacing_m = self._positive_param("waypoint_spacing_m")
        self.path_goal_tolerance_m = self._positive_param("snake_goal_tolerance_m")
        self.path_max_step_m = self._positive_param("snake_max_step_m")
        self.path_settle_sec = self._non_negative_param("snake_settle_sec")
        self.path_goto_timeout_sec = self._positive_param("snake_goto_timeout_sec")
        self.path_max_iterations = max(1, int(self.get_parameter("snake_max_iterations").value))
        self.path_pose_wait_timeout_sec = self._positive_param("snake_pose_wait_timeout_sec")
        self.path_action_server_wait_sec = self._positive_param("snake_action_server_wait_sec")

        self.start_delay_sec = self._non_negative_param("start_delay_sec")
        self.motion_enable_on_start = bool(self.get_parameter("motion_enable_on_start").value)
        self.reset_yaw_on_start = bool(self.get_parameter("reset_yaw_on_start").value)
        self.stop_on_exit = bool(self.get_parameter("stop_on_exit").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.capture_while_moving = bool(self.get_parameter("capture_while_moving").value)
        self.moving_capture_interval_sec = self._positive_param(
            "moving_capture_interval_sec"
        )
        self.images_per_waypoint = max(0, int(self.get_parameter("images_per_waypoint").value))
        self.waypoint_capture_delay_sec = self._non_negative_param(
            "waypoint_capture_delay_sec"
        )
        self.image_wait_timeout_sec = self._positive_param("image_wait_timeout_sec")

        if self.x_min_m > self.x_max_m:
            self.x_min_m, self.x_max_m = self.x_max_m, self.x_min_m
        if self.y_min_m > self.y_max_m:
            self.y_min_m, self.y_max_m = self.y_max_m, self.y_min_m
        if self.x_min_m >= self.x_max_m or self.y_min_m >= self.y_max_m:
            raise ValueError(
                "Safe path bounds collapsed. Reduce robot_diameter_m/wall_buffer_m "
                "or increase map_width_m/map_height_m."
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_file = self.metadata_path.open("a", newline="", encoding="utf-8")
        self.metadata_writer = csv.DictWriter(
            self.metadata_file,
            fieldnames=[
                "filename",
                "capture_index",
                "reason",
                "waypoint_index",
                "target_x_m",
                "target_y_m",
                "amcl_x_m",
                "amcl_y_m",
                "image_stamp_sec",
                "image_stamp_nanosec",
                "wall_time_utc",
                "image_topic",
                "encoding",
                "width",
                "height",
            ],
        )
        if self.metadata_path.stat().st_size == 0:
            self.metadata_writer.writeheader()
            self.metadata_file.flush()

        self.bridge = CvBridge()
        self.latest_image_msg: Optional[Image] = None
        self.latest_image_bgr = None
        self.latest_image_receive_time = 0.0
        self.capture_index = 0
        self.active_waypoint_index = -1
        self.active_target: Optional[tuple[float, float]] = None
        self.capture_active = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos,
        )
        self.capture_timer = None
        if self.capture_while_moving:
            self.capture_timer = self.create_timer(
                self.moving_capture_interval_sec,
                self.timer_capture_callback,
            )

        self.waypoints = self.build_snake_waypoints()
        self.get_logger().info(
            "Snake collector ready: %d waypoints, x=[%.3f, %.3f], y=[%.3f, %.3f], "
            "wall_margin=%.3f, column_spacing=%.3f, waypoint_spacing=%.3f, "
            "image_topic='%s'",
            len(self.waypoints),
            self.x_min_m,
            self.x_max_m,
            self.y_min_m,
            self.y_max_m,
            self.path_wall_margin_m,
            self.column_spacing_m,
            self.waypoint_spacing_m,
            self.image_topic,
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

    def _finite_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _positive_param(self, name: str) -> float:
        value = self._finite_float(name)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _non_negative_param(self, name: str) -> float:
        value = self._finite_float(name)
        if value < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return value

    def build_snake_waypoints(self) -> list[tuple[float, float]]:
        x_columns = _linspace_inclusive(self.x_max_m, self.x_min_m, self.column_spacing_m)
        waypoints: list[tuple[float, float]] = []

        for column_index, x_m in enumerate(x_columns):
            if column_index % 2 == 0:
                y_values = _linspace_inclusive(
                    self.y_max_m, self.y_min_m, self.waypoint_spacing_m
                )
            else:
                y_values = _linspace_inclusive(
                    self.y_min_m, self.y_max_m, self.waypoint_spacing_m
                )
            waypoints.extend((x_m, y_m) for y_m in y_values)

        return waypoints

    def image_callback(self, msg: Image) -> None:
        try:
            self.latest_image_bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().warn(f"Could not convert image: {error}")
            return
        self.latest_image_msg = msg
        self.latest_image_receive_time = time.monotonic()

    def timer_capture_callback(self) -> None:
        if not self.capture_active:
            return
        self.save_latest_image("moving")

    def wait_for_image(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.latest_image_msg is None:
            if time.monotonic() >= deadline:
                return False
            self._spin_once_until(deadline)
        return self.latest_image_msg is not None

    def save_latest_image(self, reason: str) -> bool:
        if self.latest_image_msg is None or self.latest_image_bgr is None:
            if not self.wait_for_image(self.image_wait_timeout_sec):
                self.get_logger().warn(
                    "No image received on '%s'; skipped %s capture.",
                    self.image_topic,
                    reason,
                )
                return False

        msg = self.latest_image_msg
        image = self.latest_image_bgr
        if msg is None or image is None:
            return False

        self.capture_index += 1
        stamp_text = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        filename = (
            f"snake_black_circle_{self.capture_index:05d}_{reason}_"
            f"wp_{self.active_waypoint_index:03d}_{stamp_text}.{self.image_format}"
        )
        output_path = self.output_dir / filename

        params = []
        if self.image_format == "jpg":
            params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        elif self.image_format == "png":
            params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression]

        if not cv2.imwrite(str(output_path), image, params):
            self.get_logger().error(f"Failed to save image: {output_path}")
            self.capture_index -= 1
            return False

        target_x = ""
        target_y = ""
        if self.active_target is not None:
            target_x = f"{self.active_target[0]:.9f}"
            target_y = f"{self.active_target[1]:.9f}"

        amcl_x = ""
        amcl_y = ""
        if self._latest_pose_xy is not None:
            amcl_x = f"{self._latest_pose_xy[0]:.9f}"
            amcl_y = f"{self._latest_pose_xy[1]:.9f}"

        height, width = image.shape[:2]
        self.metadata_writer.writerow(
            {
                "filename": filename,
                "capture_index": self.capture_index,
                "reason": reason,
                "waypoint_index": self.active_waypoint_index,
                "target_x_m": target_x,
                "target_y_m": target_y,
                "amcl_x_m": amcl_x,
                "amcl_y_m": amcl_y,
                "image_stamp_sec": msg.header.stamp.sec,
                "image_stamp_nanosec": msg.header.stamp.nanosec,
                "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                "image_topic": self.image_topic,
                "encoding": msg.encoding,
                "width": width,
                "height": height,
            }
        )
        self.metadata_file.flush()

        self.get_logger().info("Saved %s", str(output_path))
        return True

    def capture_at_waypoint(self) -> None:
        for image_index in range(self.images_per_waypoint):
            if self.waypoint_capture_delay_sec > 0.0:
                self._sleep_with_spin(self.waypoint_capture_delay_sec)
            self.save_latest_image(f"waypoint_{image_index + 1}")

    def log_dry_run_path(self) -> None:
        for index, (x_m, y_m) in enumerate(self.waypoints):
            self.get_logger().info(
                "Dry-run waypoint %03d/%03d: x=%.3f y=%.3f",
                index + 1,
                len(self.waypoints),
                x_m,
                y_m,
            )

    def prepare_for_motion(self) -> None:
        self.get_logger().info("Waiting %.3f s before starting.", self.start_delay_sec)
        self._sleep_with_spin(self.start_delay_sec)

        if not self.wait_for_image(self.image_wait_timeout_sec):
            raise RuntimeError(f"No images received on '{self.image_topic}'.")

        if self.motion_enable_on_start:
            self.motion_enable()
        if self.reset_yaw_on_start:
            self.reset_yaw()

    def run_snake_path(self) -> None:
        self.capture_active = True
        try:
            for index, (x_m, y_m) in enumerate(self.waypoints):
                self.active_waypoint_index = index
                self.active_target = (x_m, y_m)
                self.get_logger().info(
                    "Snake waypoint %03d/%03d: target=(%.3f, %.3f) m",
                    index + 1,
                    len(self.waypoints),
                    x_m,
                    y_m,
                )
                self.goto(
                    x_m,
                    y_m,
                    goal_tolerance_m=self.path_goal_tolerance_m,
                    max_step_m=self.path_max_step_m,
                    settle_sec=self.path_settle_sec,
                    max_iterations=self.path_max_iterations,
                    goto_timeout_sec=self.path_goto_timeout_sec,
                    pose_wait_timeout_sec=self.path_pose_wait_timeout_sec,
                    action_server_wait_sec=self.path_action_server_wait_sec,
                )
                self.capture_at_waypoint()
        finally:
            self.capture_active = False
            if self.stop_on_exit:
                try:
                    self.stop()
                except Exception as error:
                    self.get_logger().warn(f"Failed to send stop command: {error}")

        self.get_logger().info(
            "Snake collection complete: %d captures saved.",
            self.capture_index,
        )


def run_task(robot: SnakeBlackCircleDatasetCollector) -> None:
    """Run the black-circle dataset collection task."""

    if robot.dry_run:
        robot.log_dry_run_path()
        return

    robot.prepare_for_motion()
    robot.run_snake_path()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SnakeBlackCircleDatasetCollector()
        run_task(node)
    except KeyboardInterrupt:
        exit_code = 130
    except (GotoError, RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(f"Snake collection failed: {error}")
        else:
            print(f"Snake collection failed: {error}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
