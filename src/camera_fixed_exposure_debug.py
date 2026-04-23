#!/usr/bin/env python3

import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Image


class CameraFixedExposureDebugNode(Node):
    def __init__(self):
        super().__init__("camera_fixed_exposure_debug")

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("camera_node_name", "/camera")
        self.declare_parameter("window_name", "Camera Fixed Exposure Debug")
        self.declare_parameter("display_max_width", 1280)
        self.declare_parameter("display_max_height", 720)
        self.declare_parameter("exposure_time", 10000)
        self.declare_parameter("exposure_time_min", 100)
        self.declare_parameter("exposure_time_max", 40000)
        self.declare_parameter("exposure_time_step", 100)
        self.declare_parameter("exposure_time_mode", 1)
        self.declare_parameter("ae_enable", False)

        self.input_topic = self.get_parameter("input_topic").value
        self.camera_node_name = self.get_parameter("camera_node_name").value
        self.window_name = self.get_parameter("window_name").value
        self.controls_window_name = f"{self.window_name} Controls"
        self.display_max_width = max(1, int(self.get_parameter("display_max_width").value))
        self.display_max_height = max(
            1, int(self.get_parameter("display_max_height").value)
        )
        self.exposure_time_min = max(1, int(self.get_parameter("exposure_time_min").value))
        self.exposure_time_max = max(
            self.exposure_time_min,
            int(self.get_parameter("exposure_time_max").value),
        )
        self.exposure_time_step = max(
            1, int(self.get_parameter("exposure_time_step").value)
        )
        self.exposure_time_mode = int(self.get_parameter("exposure_time_mode").value)
        self.ae_enable = bool(self.get_parameter("ae_enable").value)

        initial_exposure_time = int(self.get_parameter("exposure_time").value)
        self.current_slider_exposure_time = self.clamp_exposure_time(initial_exposure_time)
        self.last_requested_exposure_time = None
        self.parameter_update_in_flight = False
        self.last_service_wait_log_time = 0.0

        display_available = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        if not display_available:
            raise RuntimeError(
                "camera_fixed_exposure_debug requires DISPLAY or WAYLAND_DISPLAY."
            )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.parameter_client = AsyncParameterClient(self, self.camera_node_name)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.controls_window_name, cv2.WINDOW_AUTOSIZE)

        self.slider_max = max(
            0,
            (self.exposure_time_max - self.exposure_time_min) // self.exposure_time_step,
        )
        slider_pos = self.exposure_time_to_slider(self.current_slider_exposure_time)
        cv2.createTrackbar(
            "ExposureTime [us]",
            self.controls_window_name,
            slider_pos,
            self.slider_max,
            lambda _value: None,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10,
        )
        self.gui_timer = self.create_timer(0.03, self.process_gui)
        self.parameter_timer = self.create_timer(0.1, self.sync_exposure_parameter)

        self.get_logger().info(
            "camera_fixed_exposure_debug started with "
            f"input_topic={self.input_topic}, camera_node_name={self.camera_node_name}, "
            f"initial_exposure_time={self.current_slider_exposure_time} us"
        )

    def clamp_exposure_time(self, value: int) -> int:
        return max(self.exposure_time_min, min(self.exposure_time_max, value))

    def exposure_time_to_slider(self, exposure_time: int) -> int:
        return max(
            0,
            min(
                self.slider_max,
                int(
                    round(
                        (self.clamp_exposure_time(exposure_time) - self.exposure_time_min)
                        / self.exposure_time_step
                    )
                ),
            ),
        )

    def slider_to_exposure_time(self, slider_pos: int) -> int:
        return self.clamp_exposure_time(
            self.exposure_time_min + slider_pos * self.exposure_time_step
        )

    def fit_within_bounds(self, frame):
        height, width = frame.shape[:2]
        if width <= 0 or height <= 0:
            return self.display_max_width, self.display_max_height

        width_scale = self.display_max_width / width
        height_scale = self.display_max_height / height
        scale = min(1.0, width_scale, height_scale)
        return max(1, int(math.floor(width * scale + 0.5))), max(
            1, int(math.floor(height * scale + 0.5))
        )

    def resize_window_to_fit_image(self, frame):
        fitted_width, fitted_height = self.fit_within_bounds(frame)
        cv2.resizeWindow(self.window_name, fitted_width, fitted_height)

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert image for debug view: {exc}")

    def read_slider_exposure_time(self) -> int:
        slider_pos = cv2.getTrackbarPos("ExposureTime [us]", self.controls_window_name)
        return self.slider_to_exposure_time(slider_pos)

    def sync_exposure_parameter(self):
        desired_exposure_time = self.read_slider_exposure_time()
        self.current_slider_exposure_time = desired_exposure_time

        if self.parameter_update_in_flight:
            return

        if self.last_requested_exposure_time == desired_exposure_time:
            return

        if not self.parameter_client.services_are_ready():
            now = time.monotonic()
            if now - self.last_service_wait_log_time >= 2.0:
                self.last_service_wait_log_time = now
                self.get_logger().info(
                    f"Waiting for parameter services from {self.camera_node_name}"
                )
            return

        parameters = [
            Parameter("ExposureTime", Parameter.Type.INTEGER, desired_exposure_time),
            Parameter("ExposureTimeMode", Parameter.Type.INTEGER, self.exposure_time_mode),
            Parameter("AeEnable", Parameter.Type.BOOL, self.ae_enable),
        ]

        self.parameter_update_in_flight = True
        future = self.parameter_client.set_parameters(parameters)
        future.add_done_callback(
            lambda completed_future: self.handle_parameter_update_result(
                completed_future, desired_exposure_time
            )
        )

    def handle_parameter_update_result(self, future, requested_exposure_time: int):
        self.parameter_update_in_flight = False

        try:
            results = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Failed to set exposure parameters on {self.camera_node_name}: {exc}"
            )
            return

        failed_reasons = [result.reason for result in results if not result.successful]
        if failed_reasons:
            self.get_logger().warn(
                "Camera node rejected exposure update: "
                + "; ".join(reason or "unknown reason" for reason in failed_reasons)
            )
            return

        self.last_requested_exposure_time = requested_exposure_time

    def build_status_frame(self):
        if self.latest_frame is not None:
            frame = self.latest_frame.copy()
        else:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Waiting for image frames...",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            frame,
            f"ExposureTime: {self.current_slider_exposure_time} us",
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"ExposureTimeMode={self.exposure_time_mode}  AeEnable={self.ae_enable}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return frame

    def process_gui(self):
        frame = self.build_status_frame()
        cv2.imshow(self.window_name, frame)
        self.resize_window_to_fit_image(frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        cv2.destroyWindow(self.controls_window_name)
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = CameraFixedExposureDebugNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
