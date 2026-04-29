#!/usr/bin/env python3

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from typing import Optional

import rclpy

from goto_point import GotoError, GotoNavigator, _format_number
from rcj_localization.msg import OrangeBallDetection
from rcj_localization.srv import Stm32Command


@dataclass(frozen=True)
class Stm32State:
    dx: float
    dy: float
    dtheta: float
    theta: float
    attempts: int
    message: str


@dataclass(frozen=True)
class Stm32Infrared:
    channel: int
    attempts: int
    message: str


@dataclass(frozen=True)
class RobotPose:
    x_m: float
    y_m: float
    yaw_rad: float
    yaw_deg: float


@dataclass(frozen=True)
class BallDetection:
    x_m: float
    y_m: float
    z_m: float
    local_x_m: float
    local_y_m: float
    local_z_m: float
    confidence: float
    stamp_sec: float


class CompetitionRobot(GotoNavigator):
    """High-level API for writing competition task logic in Python."""

    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("stm32_command_service", "/stm32/send_command")
        self.declare_parameter("ball_detection_topic", "/orange_ball_detector/detection")
        self.declare_parameter("command_call_timeout_sec", 1.0)
        self.declare_parameter("command_retry_delay_sec", 0.1)
        self.declare_parameter("command_service_wait_sec", 1.0)
        self.declare_parameter("motion_retry_timeout_sec", 20.0)

        self.stm32_command_service = str(
            self.get_parameter("stm32_command_service").value
        )
        self.ball_detection_topic = str(
            self.get_parameter("ball_detection_topic").value
        )
        self.command_call_timeout_sec = self._positive_float(
            "command_call_timeout_sec",
            float(self.get_parameter("command_call_timeout_sec").value),
        )
        self.command_retry_delay_sec = self._non_negative_float(
            "command_retry_delay_sec",
            float(self.get_parameter("command_retry_delay_sec").value),
        )
        self.command_service_wait_sec = self._positive_float(
            "command_service_wait_sec",
            float(self.get_parameter("command_service_wait_sec").value),
        )
        self.motion_retry_timeout_sec = self._positive_float(
            "motion_retry_timeout_sec",
            float(self.get_parameter("motion_retry_timeout_sec").value),
        )

        if not self.stm32_command_service:
            raise RuntimeError("Parameter 'stm32_command_service' must not be empty.")
        if not self.ball_detection_topic:
            raise RuntimeError("Parameter 'ball_detection_topic' must not be empty.")

        self._command_client = self.create_client(
            Stm32Command, self.stm32_command_service
        )
        self._latest_ball_detection: Optional[OrangeBallDetection] = None
        self._latest_ball_detection_time: Optional[float] = None
        self._ball_detection_sub = self.create_subscription(
            OrangeBallDetection,
            self.ball_detection_topic,
            self._ball_detection_callback,
            10,
        )
        self.get_logger().info(
            "CompetitionRobot ready: "
            f"command_service='{self.stm32_command_service}', "
            f"motion_action='{self.motion_action_name}', "
            f"ball_detection_topic='{self.ball_detection_topic}'"
        )

    def move(
        self,
        *,
        x_cm: float,
        y_cm: float,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        x_cm_value = float(x_cm)
        y_cm_value = float(y_cm)
        if not math.isfinite(x_cm_value) or not math.isfinite(y_cm_value):
            raise ValueError("x_cm and y_cm must be finite.")
        command = (
            "cmd_dis "
            f"{_format_number(x_cm_value)} "
            f"{_format_number(y_cm_value)}"
        )
        return self._send_motion_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )

    def turn(
        self,
        *,
        angle_deg: float,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        target_angle_deg = float(angle_deg)
        if not math.isfinite(target_angle_deg):
            raise ValueError("angle_deg must be finite.")
        command = f"cmd_turn {_format_number(target_angle_deg)}"
        return self._send_motion_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )

    def drive(
        self,
        *,
        speed_percent: int,
        move_angle_deg: float,
        head_lock: Optional[bool] = None,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        speed_percent_int = int(speed_percent)
        move_angle_deg_value = float(move_angle_deg)
        if speed_percent_int < 0 or speed_percent_int > 100:
            raise ValueError("speed_percent must be in range 0-100.")
        if not math.isfinite(move_angle_deg_value):
            raise ValueError("move_angle_deg must be finite.")

        command = (
            "cmd_dkmotor "
            f"{speed_percent_int} "
            f"{_format_number(move_angle_deg_value)}"
        )
        if head_lock is not None:
            if isinstance(head_lock, bool):
                head_lock_value = 1 if head_lock else 0
            else:
                head_lock_value = int(head_lock)
                if head_lock_value not in (0, 1):
                    raise ValueError("head_lock must be True/False or 0/1.")
            command += f" {head_lock_value}"

        self._send_command_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        return True

    def suck(
        self,
        *,
        speed_percent: int,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        speed_percent_int = int(speed_percent)
        if speed_percent_int < 0 or speed_percent_int > 100:
            raise ValueError("speed_percent must be in range 0-100.")
        self._send_command_until_success(
            f"cmd_suck {speed_percent_int}",
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        return True

    def read_infrared(self) -> Stm32Infrared:
        response = self._send_command_until_success("cmd_infred")
        match = re.search(r"channel=(\d+)", str(response.message))
        if match is None:
            raise RuntimeError(
                "STM32 infrared response did not include a channel: "
                f"{response.message}"
            )
        channel = int(match.group(1))
        if channel < 1 or channel > 7:
            raise RuntimeError(f"STM32 infrared channel out of range: {channel}")
        return Stm32Infrared(
            channel=channel,
            attempts=int(response.attempts),
            message=str(response.message),
        )

    def infrared_channel(self) -> int:
        return self.read_infrared().channel

    def set_infrared_mode(self, *, mode: str) -> bool:
        mode_text = str(mode).strip().lower()
        if mode_text not in ("pt", "tz"):
            raise ValueError("mode must be 'pt' or 'tz'.")
        self._send_command_until_success(f"cmd_infred_mode {mode_text}")
        return True

    def infrared_plain_mode(self) -> bool:
        return self.set_infrared_mode(mode="pt")

    def infrared_modulated_mode(self) -> bool:
        return self.set_infrared_mode(mode="tz")

    def suck_on(self, *, speed_percent: int = 100) -> bool:
        return self.suck(speed_percent=speed_percent)

    def suck_off(self) -> bool:
        return self.suck(speed_percent=0)

    def reset_yaw(self) -> bool:
        self._send_command_until_success("cmd_anglecal")
        return True

    def reset_mcu(self) -> bool:
        self._send_command_until_success("cmd_mcureset")
        return True

    def motion_enable(self) -> bool:
        self._send_command_until_success("cmd_conmotion 1")
        return True

    def motion_disable(self) -> bool:
        self._send_command_until_success("cmd_conmotion 0")
        return True

    def stop(self) -> bool:
        self._send_command_until_success("cmd_juststop")
        return True

    def request_state(self) -> Stm32State:
        response = self._send_command_until_success("cmd_request")
        return Stm32State(
            dx=float(response.dx),
            dy=float(response.dy),
            dtheta=float(response.dtheta),
            theta=float(response.theta),
            attempts=int(response.attempts),
            message=str(response.message),
        )

    def get_pose(self, *, timeout_sec: Optional[float] = None) -> RobotPose:
        x_m, y_m = self.wait_for_pose(timeout_sec=timeout_sec)
        if self._latest_pose_yaw_rad is None:
            raise GotoError(f"Timed out waiting for yaw from '{self.pose_topic}'.")
        yaw_rad = self._latest_pose_yaw_rad
        return RobotPose(
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            yaw_deg=math.degrees(yaw_rad),
        )

    def find_ball(
        self,
        *,
        timeout_sec: float = 1.0,
        min_confidence: float = 0.0,
    ) -> Optional[BallDetection]:
        timeout = self._non_negative_float("timeout_sec", timeout_sec)
        min_confidence_value = self._non_negative_float(
            "min_confidence", min_confidence
        )
        start_time = time.monotonic()
        deadline = time.monotonic() + timeout
        min_detection_time = None if timeout == 0.0 else start_time

        while rclpy.ok():
            detection = self._current_ball_detection(
                min_confidence_value, min_detection_time=min_detection_time
            )
            if detection is not None:
                return detection
            if timeout == 0.0 or time.monotonic() >= deadline:
                return None
            self._spin_once_until(deadline)

        raise RuntimeError("ROS shutdown while waiting for orange ball detection.")

    def sleep(self, *, duration_sec: float) -> None:
        self.stop()
        self._sleep_with_spin(
            self._non_negative_float("duration_sec", duration_sec)
        )

    def _ball_detection_callback(self, msg: OrangeBallDetection) -> None:
        self._latest_ball_detection = msg
        self._latest_ball_detection_time = time.monotonic()

    def _current_ball_detection(
        self,
        min_confidence: float,
        min_detection_time: Optional[float] = None,
    ) -> Optional[BallDetection]:
        msg = self._latest_ball_detection
        if (
            msg is None
            or self._latest_ball_detection_time is None
            or self._latest_pose_yaw_rad is None
            or not msg.detected
            or float(msg.confidence) < min_confidence
            or (
                min_detection_time is not None
                and self._latest_ball_detection_time < min_detection_time
            )
        ):
            return None

        return BallDetection(
            *self._ball_base_link_to_map_delta(
                float(msg.ball_center_m.x),
                float(msg.ball_center_m.y),
                float(msg.ball_center_m.z),
            ),
            local_x_m=float(msg.ball_center_m.x),
            local_y_m=float(msg.ball_center_m.y),
            local_z_m=float(msg.ball_center_m.z),
            confidence=float(msg.confidence),
            stamp_sec=float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9,
        )

    def _ball_base_link_to_map_delta(
        self, local_x_m: float, local_y_m: float, local_z_m: float
    ) -> tuple[float, float, float]:
        if self._latest_pose_yaw_rad is None:
            return math.nan, math.nan, math.nan
        cos_yaw = math.cos(self._latest_pose_yaw_rad)
        sin_yaw = math.sin(self._latest_pose_yaw_rad)
        return (
            (local_x_m * cos_yaw) - (local_y_m * sin_yaw),
            (local_x_m * sin_yaw) + (local_y_m * cos_yaw),
            local_z_m,
        )

    def _send_motion_until_success(
        self,
        command: str,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        retry_delay = self._retry_delay(retry_delay_sec)
        command_timeout = self._positive_float(
            "timeout_sec",
            self.motion_retry_timeout_sec
            if timeout_sec is None
            else timeout_sec,
        )
        attempt = 1

        while rclpy.ok():
            self._wait_for_motion_server()
            self.get_logger().info(
                f"STM32 motion attempt {attempt}: {command}"
            )
            try:
                if self._send_motion_command(command, timeout_sec=command_timeout):
                    return True
            except Exception as error:
                self.get_logger().warn(
                    f"STM32 motion attempt {attempt} failed: {command}; {error}"
                )

            attempt += 1
            self._sleep_with_spin(retry_delay)

        raise GotoError(f"ROS shutdown before STM32 motion succeeded: {command}")

    def _send_command_until_success(
        self,
        command: str,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> Stm32Command.Response:
        retry_delay = self._retry_delay(retry_delay_sec)
        command_timeout = self._positive_float(
            "timeout_sec",
            self.command_call_timeout_sec if timeout_sec is None else timeout_sec,
        )
        attempt = 1

        while rclpy.ok():
            if not self._command_client.wait_for_service(
                timeout_sec=self.command_service_wait_sec
            ):
                self.get_logger().warn(
                    "Waiting for STM32 command service "
                    f"'{self.stm32_command_service}'."
                )
                self._sleep_with_spin(retry_delay)
                continue

            self.get_logger().info(f"STM32 command attempt {attempt}: {command}")
            try:
                response = self._send_command_once(
                    command, timeout_sec=command_timeout
                )
            except Exception as error:
                response = None
                self.get_logger().warn(
                    f"STM32 command attempt {attempt} raised: {command}; {error}"
                )
            if response is not None and response.success:
                self.get_logger().info(
                    f"STM32 command ok: {command} "
                    f"(status={response.status}, attempts={response.attempts})"
                )
                return response

            if response is None:
                self.get_logger().warn(
                    f"STM32 command timed out locally: {command}"
                )
            else:
                self.get_logger().warn(
                    f"STM32 command failed: {command} "
                    f"(status={response.status}, attempts={response.attempts}, "
                    f"message={response.message})"
                )

            attempt += 1
            self._sleep_with_spin(retry_delay)

        raise RuntimeError(f"ROS shutdown before STM32 command succeeded: {command}")

    def _send_command_once(
        self, command: str, timeout_sec: float
    ) -> Optional[Stm32Command.Response]:
        request = Stm32Command.Request()
        request.command = command
        future = self._command_client.call_async(request)
        if not self._spin_until_future_done(future, timeout_sec):
            return None
        return future.result()

    def _wait_for_motion_server(self) -> None:
        while rclpy.ok():
            if self._motion_client.wait_for_server(
                timeout_sec=self.action_server_wait_sec
            ):
                return
            self.get_logger().warn(
                f"Waiting for STM32 motion action '{self.motion_action_name}'."
            )
            self._sleep_with_spin(self.command_retry_delay_sec)

        raise GotoError(
            f"ROS shutdown while waiting for motion action '{self.motion_action_name}'."
        )

    def _retry_delay(self, retry_delay_sec: Optional[float]) -> float:
        return self._non_negative_float(
            "retry_delay_sec",
            self.command_retry_delay_sec
            if retry_delay_sec is None
            else retry_delay_sec,
        )
