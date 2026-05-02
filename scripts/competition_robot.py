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

    @property
    def is_behind(self) -> bool:
        return self.channel == -1

    @property
    def has_direction(self) -> bool:
        return 1 <= self.channel <= 7


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
    absolute_x_m: float
    absolute_y_m: float
    absolute_z_m: float
    base_x_m: float
    base_y_m: float
    base_z_m: float
    local_x_m: float
    local_y_m: float
    local_z_m: float
    angle_deg: float
    absolute_angle_deg: float
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
        self.declare_parameter("yaw_zero_map_degrees", 0.0)

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
        self.yaw_zero_map_degrees = float(
            self.get_parameter("yaw_zero_map_degrees").value
        )

        if not self.stm32_command_service:
            raise RuntimeError("Parameter 'stm32_command_service' must not be empty.")
        if not self.ball_detection_topic:
            raise RuntimeError("Parameter 'ball_detection_topic' must not be empty.")
        if not math.isfinite(self.yaw_zero_map_degrees):
            raise RuntimeError("Parameter 'yaw_zero_map_degrees' must be finite.")

        self._command_client = self.create_client(
            Stm32Command, self.stm32_command_service
        )
        self._latest_ball_detection: Optional[OrangeBallDetection] = None
        self._latest_ball_detection_time: Optional[float] = None
        self._ball_sucked_required_count = 3
        self._ball_sucked_sample_interval_sec = 0.05
        self._ball_sucked_detected_streak = 0
        self._ball_sucked_confirmed = False
        self._ball_sucked_last_sample_time: Optional[float] = None
        self._ball_sucked_state = "empty"
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
            f"ball_detection_topic='{self.ball_detection_topic}', "
            f"yaw_zero_map_degrees={self.yaw_zero_map_degrees:.3f}"
        )

    def move(
        self,
        *,
        x_cm: float,
        y_cm: float,
        speed_profile: int = 1,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        x_cm_value = float(x_cm)
        y_cm_value = float(y_cm)
        if not math.isfinite(x_cm_value) or not math.isfinite(y_cm_value):
            raise ValueError("x_cm and y_cm must be finite.")
        speed_profile_int = int(speed_profile)
        if speed_profile_int not in (0, 1, 2):
            raise ValueError("speed_profile must be 0, 1, or 2.")
        command = (
            "cmd_dis "
            f"{_format_number(x_cm_value)} "
            f"{_format_number(y_cm_value)} "
            f"{speed_profile_int}"
        )
        result = self._send_motion_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        self._wait_after(wait_after)
        return result

    def turn(
        self,
        *,
        angle_deg: float,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        target_angle_deg = float(angle_deg)
        if not math.isfinite(target_angle_deg):
            raise ValueError("angle_deg must be finite.")
        command = f"cmd_turn {_format_number(target_angle_deg)}"
        result = self._send_motion_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        self._wait_after(wait_after)
        return result

    def turn_to_point(
        self,
        *,
        x_m: float,
        y_m: float,
        min_distance_m: float = 0.01,
        pose_wait_timeout_sec: Optional[float] = None,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        target_x = float(x_m)
        target_y = float(y_m)
        min_distance = self._non_negative_float("min_distance_m", min_distance_m)
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            raise ValueError("x_m and y_m must be finite.")

        current_x, current_y = self.wait_for_pose(
            timeout_sec=(
                self.pose_wait_timeout_sec
                if pose_wait_timeout_sec is None
                else pose_wait_timeout_sec
            )
        )
        dx_map = target_x - current_x
        dy_map = target_y - current_y
        distance_m = math.hypot(dx_map, dy_map)
        if distance_m <= min_distance:
            self.get_logger().warn(
                "Turn-to-point skipped because target is too close: "
                f"target=({target_x:.3f}, {target_y:.3f}) m, "
                f"current=({current_x:.3f}, {current_y:.3f}) m, "
                f"distance={distance_m:.3f} m"
            )
            self._wait_after(wait_sec)
            return False

        ros_map_angle_deg = math.degrees(math.atan2(dy_map, dx_map))
        stm32_yaw_deg = self._normalize_angle_deg(
            ros_map_angle_deg - 90.0 - self.yaw_zero_map_degrees
        )
        self.get_logger().info(
            "Turn-to-point target: "
            f"target=({target_x:.3f}, {target_y:.3f}) m, "
            f"current=({current_x:.3f}, {current_y:.3f}) m, "
            f"ros_angle={ros_map_angle_deg:.3f} deg, "
            f"stm32_yaw={stm32_yaw_deg:.3f} deg"
        )

        return self.turn(
            angle_deg=stm32_yaw_deg,
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
            wait_sec=wait_sec,
        )

    def drive(
        self,
        *,
        speed_percent: int,
        move_angle_deg: float,
        head_lock: Optional[bool] = None,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
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
        self._wait_after(wait_after)
        return True

    def suck(
        self,
        *,
        speed_percent: int,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        speed_percent_int = int(speed_percent)
        if speed_percent_int < 0 or speed_percent_int > 100:
            raise ValueError("speed_percent must be in range 0-100.")
        self._send_command_until_success(
            f"cmd_suck {speed_percent_int}",
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        self._wait_after(wait_after)
        return True

    def read_infrared(self, *, wait_sec: float = 0.0) -> Stm32Infrared:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        response = self._send_command_until_success("cmd_infred")
        match = re.search(r"channel=(-?\d+)", str(response.message))
        if match is None:
            raise RuntimeError(
                "STM32 infrared response did not include a channel: "
                f"{response.message}"
            )
        channel = int(match.group(1))
        if channel != -1 and (channel < 1 or channel > 7):
            raise RuntimeError(f"STM32 infrared channel out of range: {channel}")
        result = Stm32Infrared(
            channel=channel,
            attempts=int(response.attempts),
            message=str(response.message),
        )
        self._wait_after(wait_after)
        return result

    def infrared_channel(self, *, wait_sec: float = 0.0) -> int:
        return self.read_infrared(wait_sec=wait_sec).channel

    def set_infrared_mode(self, *, mode: str, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        mode_text = str(mode).strip().lower()
        if mode_text not in ("pt", "tz"):
            raise ValueError("mode must be 'pt' or 'tz'.")
        self._send_command_until_success(f"cmd_infred_mode {mode_text}")
        self._wait_after(wait_after)
        return True

    def infrared_plain_mode(self, *, wait_sec: float = 0.0) -> bool:
        return self.set_infrared_mode(mode="pt", wait_sec=wait_sec)

    def infrared_modulated_mode(self, *, wait_sec: float = 0.0) -> bool:
        return self.set_infrared_mode(mode="tz", wait_sec=wait_sec)

    def suck_on(self, *, speed_percent: int = 100, wait_sec: float = 0.0) -> bool:
        return self.suck(speed_percent=speed_percent, wait_sec=wait_sec)

    def suck_off(self, *, wait_sec: float = 0.0) -> bool:
        return self.suck(speed_percent=0, wait_sec=wait_sec)

    def is_ball_detected(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        response = self._send_command_until_success("cmd_xqcx")
        match = re.search(r"detected=(0|1)", str(response.message))
        if match is None:
            raise RuntimeError(
                "STM32 ball-detection response did not include detected=0/1: "
                f"{response.message}"
            )
        detected = match.group(1) == "1"
        self._wait_after(wait_after)
        return detected

    def reset_ball_sucked_detector(
        self,
        *,
        required_detected_count: int = 3,
        sample_interval_sec: float = 0.05,
    ) -> None:
        self._ball_sucked_required_count = self._positive_int(
            "required_detected_count", required_detected_count
        )
        self._ball_sucked_sample_interval_sec = self._non_negative_float(
            "sample_interval_sec", sample_interval_sec
        )
        self._ball_sucked_detected_streak = 0
        self._ball_sucked_confirmed = False
        self._ball_sucked_last_sample_time = None
        self._ball_sucked_state = "empty"

    def poll_ball_sucked(self, *, stop_on_success: bool = False) -> bool:
        if self._ball_sucked_confirmed:
            return True

        now = time.monotonic()
        if (
            self._ball_sucked_last_sample_time is not None
            and now - self._ball_sucked_last_sample_time
            < self._ball_sucked_sample_interval_sec
        ):
            return False

        self._ball_sucked_last_sample_time = now
        detected = self.is_ball_detected()
        if detected:
            self._ball_sucked_detected_streak += 1
            self._ball_sucked_state = "candidate"
            if (
                self._ball_sucked_detected_streak
                >= self._ball_sucked_required_count
            ):
                self._ball_sucked_state = "confirmed"
                self._ball_sucked_confirmed = True
                if stop_on_success:
                    self.stop()
                self.get_logger().info(
                    "Suction ball state confirmed: "
                    f"state={self._ball_sucked_state}, "
                    f"detected_streak={self._ball_sucked_detected_streak}"
                )
                return True
        else:
            self._ball_sucked_detected_streak = 0
            self._ball_sucked_state = "empty"

        return False

    def wait_for_ball_sucked(
        self,
        *,
        timeout_sec: float = 3.0,
        required_detected_count: int = 3,
        sample_interval_sec: float = 0.05,
        stop_on_success: bool = False,
        wait_sec: float = 0.0,
    ) -> bool:
        timeout = self._non_negative_float("timeout_sec", timeout_sec)
        required_count = self._positive_int(
            "required_detected_count", required_detected_count
        )
        sample_interval = self._non_negative_float(
            "sample_interval_sec", sample_interval_sec
        )
        wait_after = self._non_negative_float("wait_sec", wait_sec)

        deadline = time.monotonic() + timeout
        detected_streak = 0
        state = "empty"

        while rclpy.ok():
            detected = self.is_ball_detected()
            if detected:
                detected_streak += 1
                state = "candidate"
                if detected_streak >= required_count:
                    state = "confirmed"
                    if stop_on_success:
                        self.stop()
                    self.get_logger().info(
                        "Suction ball state confirmed: "
                        f"state={state}, detected_streak={detected_streak}"
                    )
                    self._wait_after(wait_after)
                    return True
            else:
                detected_streak = 0
                state = "empty"

            if timeout == 0.0 or time.monotonic() >= deadline:
                self.get_logger().info(
                    "Suction ball state not confirmed: "
                    f"state={state}, detected_streak={detected_streak}"
                )
                self._wait_after(wait_after)
                return False

            self.timer(
                duration_sec=min(sample_interval, self._remaining_time(deadline))
            )

        raise RuntimeError("ROS shutdown while waiting for suction ball state.")

    def wait_for_ball_released(
        self,
        *,
        timeout_sec: float = 3.0,
        required_empty_count: int = 3,
        sample_interval_sec: float = 0.05,
        stop_on_success: bool = False,
        wait_sec: float = 0.0,
    ) -> bool:
        timeout = self._non_negative_float("timeout_sec", timeout_sec)
        required_count = self._positive_int(
            "required_empty_count", required_empty_count
        )
        sample_interval = self._non_negative_float(
            "sample_interval_sec", sample_interval_sec
        )
        wait_after = self._non_negative_float("wait_sec", wait_sec)

        deadline = time.monotonic() + timeout
        empty_streak = 0
        state = "sucked"

        while rclpy.ok():
            detected = self.is_ball_detected()
            if not detected:
                empty_streak += 1
                state = "release_candidate"
                if empty_streak >= required_count:
                    state = "released"
                    if stop_on_success:
                        self.stop()
                    self.get_logger().info(
                        "Suction release state confirmed: "
                        f"state={state}, empty_streak={empty_streak}"
                    )
                    self._wait_after(wait_after)
                    return True
            else:
                empty_streak = 0
                state = "sucked"

            if timeout == 0.0 or time.monotonic() >= deadline:
                self.get_logger().info(
                    "Suction release state not confirmed: "
                    f"state={state}, empty_streak={empty_streak}"
                )
                self._wait_after(wait_after)
                return False

            self.timer(
                duration_sec=min(sample_interval, self._remaining_time(deadline))
            )

        raise RuntimeError("ROS shutdown while waiting for suction release state.")

    def set_relay(self, *, enabled: bool, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        enabled_value = 1 if bool(enabled) else 0
        self._send_command_until_success(f"cmd_dct {enabled_value}")
        self._wait_after(wait_after)
        return True

    def relay_on(self, *, wait_sec: float = 0.0) -> bool:
        return self.set_relay(enabled=True, wait_sec=wait_sec)

    def relay_off(self, *, wait_sec: float = 0.0) -> bool:
        return self.set_relay(enabled=False, wait_sec=wait_sec)

    def reset_yaw(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._send_command_until_success("cmd_anglecal")
        self._wait_after(wait_after)
        return True

    def reset_mcu(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._send_command_until_success("cmd_mcureset")
        self._wait_after(wait_after)
        return True

    def motion_enable(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._send_command_until_success("cmd_conmotion 1")
        self._wait_after(wait_after)
        return True

    def motion_disable(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._send_command_until_success("cmd_conmotion 0")
        self._wait_after(wait_after)
        return True

    def stop(self, *, wait_sec: float = 0.0) -> bool:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._send_command_until_success("cmd_juststop")
        self._wait_after(wait_after)
        return True

    def request_state(self, *, wait_sec: float = 0.0) -> Stm32State:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        response = self._send_command_until_success("cmd_request")
        result = Stm32State(
            dx=float(response.dx),
            dy=float(response.dy),
            dtheta=float(response.dtheta),
            theta=float(response.theta),
            attempts=int(response.attempts),
            message=str(response.message),
        )
        self._wait_after(wait_after)
        return result

    def get_pose(
        self,
        *,
        timeout_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> RobotPose:
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        x_m, y_m = self.wait_for_pose(timeout_sec=timeout_sec)
        if self._latest_pose_yaw_rad is None:
            raise GotoError(f"Timed out waiting for yaw from '{self.pose_topic}'.")
        yaw_rad = self._latest_pose_yaw_rad
        result = RobotPose(
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            yaw_deg=math.degrees(yaw_rad),
        )
        self._wait_after(wait_after)
        return result

    def find_ball(
        self,
        *,
        timeout_sec: float = 1.0,
        min_confidence: float = 0.0,
        wait_sec: float = 0.0,
    ) -> Optional[BallDetection]:
        timeout = self._non_negative_float("timeout_sec", timeout_sec)
        min_confidence_value = self._non_negative_float(
            "min_confidence", min_confidence
        )
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        start_time = time.monotonic()
        deadline = time.monotonic() + timeout
        min_detection_time = None if timeout == 0.0 else start_time

        while rclpy.ok():
            detection = self._current_ball_detection(
                min_confidence_value, min_detection_time=min_detection_time
            )
            if detection is not None:
                self._wait_after(wait_after)
                return detection
            if timeout == 0.0 or time.monotonic() >= deadline:
                self._wait_after(wait_after)
                return None
            self._spin_once_until(deadline)

        raise RuntimeError("ROS shutdown while waiting for orange ball detection.")

    def goto_ball_standoff(
        self,
        ball: Optional[BallDetection] = None,
        *,
        stand_off_m: float = 0.12,
        min_confidence: float = 0.5,
        detection_timeout_sec: float = 1.0,
        goal_tolerance_m: float = 0.04,
        max_step_m: float = 0.35,
        speed_profile: int = 1,
        wait_sec: float = 0.0,
    ) -> bool:
        """Go to a map target that stops stand_off_m before the detected ball."""

        stand_off = self._non_negative_float("stand_off_m", stand_off_m)
        goal_tolerance = self._positive_float(
            "goal_tolerance_m", goal_tolerance_m
        )
        speed_profile_int = int(speed_profile)
        if speed_profile_int not in (0, 1, 2):
            raise ValueError("speed_profile must be 0, 1, or 2.")

        if ball is None:
            ball = self.find_ball(
                timeout_sec=detection_timeout_sec,
                min_confidence=min_confidence,
            )
        if ball is None:
            return False

        distance_to_ball_m = math.hypot(ball.x_m, ball.y_m)
        if distance_to_ball_m <= 1e-6:
            return False

        if distance_to_ball_m <= stand_off + goal_tolerance:
            return True

        direction_x = ball.x_m / distance_to_ball_m
        direction_y = ball.y_m / distance_to_ball_m
        target_x_m = ball.absolute_x_m - direction_x * stand_off
        target_y_m = ball.absolute_y_m - direction_y * stand_off

        self.goto(
            x_m=target_x_m,
            y_m=target_y_m,
            goal_tolerance_m=goal_tolerance,
            max_step_m=max_step_m,
            speed_profile=speed_profile_int,
            wait_sec=wait_sec,
        )
        return True

    def spin_find_ball(
        self,
        *,
        step_deg: float = 20.0,
        direction: int = 1,
        max_turn_deg: float = 360.0,
        min_confidence: float = 0.5,
        detection_timeout_sec: float = 0.25,
        settle_sec: float = 0.2,
        confirm_settle_sec: float = 0.5,
        confirm_timeout_sec: float = 1.0,
        wait_sec: float = 0.0,
    ) -> Optional[BallDetection]:
        """Search for a ball by turning in place, then re-read after settling."""

        step = self._positive_float("step_deg", step_deg)
        max_turn = self._positive_float("max_turn_deg", max_turn_deg)
        detection_timeout = self._non_negative_float(
            "detection_timeout_sec", detection_timeout_sec
        )
        settle = self._non_negative_float("settle_sec", settle_sec)
        confirm_settle = self._non_negative_float(
            "confirm_settle_sec", confirm_settle_sec
        )
        confirm_timeout = self._non_negative_float(
            "confirm_timeout_sec", confirm_timeout_sec
        )
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        direction_value = int(direction)
        if direction_value == 0:
            raise ValueError("direction must be positive or negative.")
        direction_sign = 1.0 if direction_value > 0 else -1.0

        ball = self.find_ball(
            timeout_sec=detection_timeout,
            min_confidence=min_confidence,
        )
        if ball is not None:
            result = self._confirm_ball_after_stop(
                ball,
                min_confidence=min_confidence,
                settle_sec=confirm_settle,
                timeout_sec=confirm_timeout,
            )
            self._wait_after(wait_after)
            return result

        start_yaw = self.request_state().theta
        turn_count = int(math.ceil(max_turn / step))

        for turn_index in range(1, turn_count + 1):
            swept_deg = min(step * turn_index, max_turn)
            target_yaw = self._normalize_angle_deg(
                start_yaw + direction_sign * swept_deg
            )
            self.turn(angle_deg=target_yaw)
            self.timer(duration_sec=settle)

            ball = self.find_ball(
                timeout_sec=detection_timeout,
                min_confidence=min_confidence,
            )
            if ball is not None:
                result = self._confirm_ball_after_stop(
                    ball,
                    min_confidence=min_confidence,
                    settle_sec=confirm_settle,
                    timeout_sec=confirm_timeout,
                )
                self._wait_after(wait_after)
                return result

        self._wait_after(wait_after)
        return None

    def _confirm_ball_after_stop(
        self,
        fallback_ball: BallDetection,
        *,
        min_confidence: float,
        settle_sec: float,
        timeout_sec: float,
    ) -> BallDetection:
        self.stop()
        self.timer(duration_sec=settle_sec)
        confirmed_ball = self.find_ball(
            timeout_sec=timeout_sec,
            min_confidence=min_confidence,
        )
        if confirmed_ball is not None:
            return confirmed_ball
        return fallback_ball

    def timer(self, *, duration_sec: float, wait_sec: float = 0.0) -> None:
        duration = self._non_negative_float("duration_sec", duration_sec)
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self._sleep_with_spin(duration)
        self._wait_after(wait_after)

    def sleep(self, *, duration_sec: float, wait_sec: float = 0.0) -> None:
        duration = self._non_negative_float("duration_sec", duration_sec)
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        self.stop()
        self._sleep_with_spin(duration)
        self._wait_after(wait_after)

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
            or self._latest_pose_xy is None
            or self._latest_pose_yaw_rad is None
            or not msg.detected
            or float(msg.confidence) < min_confidence
            or (
                min_detection_time is not None
                and self._latest_ball_detection_time < min_detection_time
            )
        ):
            return None

        local_x_m = float(msg.ball_center_m.x)
        local_y_m = float(msg.ball_center_m.y)
        local_z_m = float(msg.ball_center_m.z)
        base_x_m, base_y_m, base_z_m = self._ball_detector_to_base_link(
            local_x_m,
            local_y_m,
            local_z_m,
        )
        map_x_m, map_y_m, map_z_m = self._ball_base_link_to_map_delta(
            base_x_m,
            base_y_m,
            base_z_m,
        )
        robot_x_m, robot_y_m = self._latest_pose_xy

        return BallDetection(
            x_m=map_x_m,
            y_m=map_y_m,
            z_m=map_z_m,
            absolute_x_m=robot_x_m + map_x_m,
            absolute_y_m=robot_y_m + map_y_m,
            absolute_z_m=map_z_m,
            base_x_m=base_x_m,
            base_y_m=base_y_m,
            base_z_m=base_z_m,
            local_x_m=local_x_m,
            local_y_m=local_y_m,
            local_z_m=local_z_m,
            angle_deg=self._ball_angle_deg(base_x_m, base_y_m),
            absolute_angle_deg=self._ball_absolute_angle_deg(
                base_x_m,
                base_y_m,
            ),
            confidence=float(msg.confidence),
            stamp_sec=float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9,
        )

    @staticmethod
    def _ball_detector_to_base_link(
        detector_x_m: float, detector_y_m: float, detector_z_m: float
    ) -> tuple[float, float, float]:
        # See docs/coordinate_frames.md. Detector +x is image-down, so robot
        # forward/base_link +x is detector -x. Detector +y is image-left.
        return -detector_x_m, detector_y_m, detector_z_m

    @staticmethod
    def _ball_angle_deg(base_x_m: float, base_y_m: float) -> float:
        return math.degrees(math.atan2(base_y_m, base_x_m))

    def _ball_ros_map_angle_deg(self, base_x_m: float, base_y_m: float) -> float:
        if self._latest_pose_yaw_rad is None:
            return math.nan
        robot_yaw_deg = math.degrees(self._latest_pose_yaw_rad)
        return self._normalize_angle_deg(
            robot_yaw_deg + self._ball_angle_deg(base_x_m, base_y_m)
        )

    def _ball_absolute_angle_deg(self, base_x_m: float, base_y_m: float) -> float:
        ros_map_angle_deg = self._ball_ros_map_angle_deg(
            base_x_m,
            base_y_m,
        )
        return self._normalize_angle_deg(
            ros_map_angle_deg - 90.0 - self.yaw_zero_map_degrees
        )

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

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
            self.get_logger().debug(
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

            self.get_logger().debug(f"STM32 command attempt {attempt}: {command}")
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
                self.get_logger().debug(
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
