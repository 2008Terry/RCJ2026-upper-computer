#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from rcj_localization.action import Stm32Motion


class GotoError(RuntimeError):
    """Raised when a goto target cannot be reached safely."""


def _format_number(value: float) -> str:
    if math.isclose(value, round(value), abs_tol=1e-9):
        return str(int(round(value)))
    return f"{value:.6f}".rstrip("0").rstrip(".")


class GotoNavigator(Node):
    def __init__(self) -> None:
        super().__init__("goto_navigator")

        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("motion_action_name", "/stm32/motion")
        self.declare_parameter("goal_tolerance_m", 0.02)
        self.declare_parameter("max_step_m", 0.80)
        self.declare_parameter("settle_sec", 0.7)
        self.declare_parameter("max_iterations", 25)
        self.declare_parameter("goto_timeout_sec", 40.0)
        self.declare_parameter("pose_wait_timeout_sec", 5.0)
        self.declare_parameter("action_server_wait_sec", 10.0)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.motion_action_name = str(self.get_parameter("motion_action_name").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.max_step_m = float(self.get_parameter("max_step_m").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.goto_timeout_sec = float(self.get_parameter("goto_timeout_sec").value)
        self.pose_wait_timeout_sec = float(
            self.get_parameter("pose_wait_timeout_sec").value
        )
        self.action_server_wait_sec = float(
            self.get_parameter("action_server_wait_sec").value
        )

        self._validate_parameters()

        self._latest_pose_xy: Optional[Tuple[float, float]] = None
        self._latest_pose_yaw_rad: Optional[float] = None
        self._latest_pose_received_time: Optional[float] = None
        self._motion_in_flight = False

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self._pose_callback,
            10,
        )
        self._motion_client = ActionClient(
            self, Stm32Motion, self.motion_action_name
        )

        self.get_logger().info(
            f"GotoNavigator ready: pose_topic='{self.pose_topic}', "
            f"motion_action='{self.motion_action_name}', "
            f"tolerance={self.goal_tolerance_m:.3f} m, "
            f"max_step={self.max_step_m:.3f} m, "
            f"settle={self.settle_sec:.3f} s, "
            f"max_iterations={self.max_iterations}, "
            f"timeout={self.goto_timeout_sec:.3f} s"
        )

    def goto(
        self,
        *,
        x_m: float,
        y_m: float,
        tolerance_m: Optional[float] = None,
        goal_tolerance_m: Optional[float] = None,
        max_step_m: Optional[float] = None,
        settle_sec: Optional[float] = None,
        max_iterations: Optional[int] = None,
        goto_timeout_sec: Optional[float] = None,
        pose_wait_timeout_sec: Optional[float] = None,
        action_server_wait_sec: Optional[float] = None,
        wait_sec: float = 0.0,
    ) -> bool:
        target_x = float(x_m)
        target_y = float(y_m)
        if tolerance_m is not None and goal_tolerance_m is not None:
            raise ValueError(
                "Use either tolerance_m or goal_tolerance_m, not both."
            )
        selected_tolerance_m = (
            goal_tolerance_m if goal_tolerance_m is not None else tolerance_m
        )
        goal_tolerance = self._positive_float(
            "goal_tolerance_m",
            self.goal_tolerance_m
            if selected_tolerance_m is None
            else selected_tolerance_m,
        )
        max_step = self._positive_float(
            "max_step_m", self.max_step_m if max_step_m is None else max_step_m
        )
        settle = self._non_negative_float(
            "settle_sec", self.settle_sec if settle_sec is None else settle_sec
        )
        iteration_limit = self._positive_int(
            "max_iterations",
            self.max_iterations if max_iterations is None else max_iterations,
        )
        goto_timeout = self._positive_float(
            "goto_timeout_sec",
            self.goto_timeout_sec if goto_timeout_sec is None else goto_timeout_sec,
        )
        pose_wait_timeout = self._positive_float(
            "pose_wait_timeout_sec",
            self.pose_wait_timeout_sec
            if pose_wait_timeout_sec is None
            else pose_wait_timeout_sec,
        )
        action_wait_timeout = self._positive_float(
            "action_server_wait_sec",
            self.action_server_wait_sec
            if action_server_wait_sec is None
            else action_server_wait_sec,
        )
        wait_after = self._non_negative_float("wait_sec", wait_sec)
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            raise ValueError("goto target coordinates must be finite.")

        deadline = time.monotonic() + goto_timeout
        self.get_logger().info(
            f"Goto target start: target=({target_x:.3f}, {target_y:.3f}) m, "
            f"tolerance={goal_tolerance:.3f} m"
        )

        self._ensure_action_server_ready(deadline, action_wait_timeout)
        last_distance = math.inf

        for iteration in range(1, iteration_limit + 1):
            current_x, current_y = self.wait_for_pose(
                timeout_sec=self._bounded_timeout(deadline, pose_wait_timeout)
            )
            dx_map = target_x - current_x
            dy_map = target_y - current_y
            distance = math.hypot(dx_map, dy_map)
            last_distance = distance

            if distance <= goal_tolerance:
                self.get_logger().info(
                    f"Goto target reached: target=({target_x:.3f}, "
                    f"{target_y:.3f}) m, current=({current_x:.3f}, "
                    f"{current_y:.3f}) m, error={distance:.3f} m"
                )
                self._wait_after(wait_after)
                return True

            if time.monotonic() >= deadline:
                raise GotoError(
                    "Goto timed out before target was reached: "
                    f"target=({target_x:.3f}, {target_y:.3f}) m, "
                    f"last_error={distance:.3f} m."
                )

            step_dx_map, step_dy_map = self._limit_step(dx_map, dy_map, max_step)
            step_dx_stm32_m, step_dy_stm32_m = self._map_delta_to_stm32_delta(
                step_dx_map, step_dy_map
            )
            step_dx_stm32_cm = step_dx_stm32_m * 100.0
            step_dy_stm32_cm = step_dy_stm32_m * 100.0
            command = (
                "cmd_dis "
                f"{_format_number(step_dx_stm32_cm)} "
                f"{_format_number(step_dy_stm32_cm)} "
                "1"
            )

            self.get_logger().info(
                f"Goto iteration {iteration}/{iteration_limit}: "
                f"current=({current_x:.3f}, {current_y:.3f}) m, "
                f"target=({target_x:.3f}, {target_y:.3f}) m, "
                f"map_delta=({step_dx_map:.3f}, {step_dy_map:.3f}) m, "
                f"stm32_delta=({step_dx_stm32_cm:.3f}, "
                f"{step_dy_stm32_cm:.3f}) cm, error={distance:.3f} m, "
                f"command='{command}'"
            )

            motion_ok = self._send_motion_command(
                command,
                timeout_sec=self._remaining_time(deadline),
            )
            if not motion_ok:
                self.get_logger().warn(
                    f"Motion command did not succeed; settling for {settle:.3f} s "
                    "before replanning from the latest pose."
                )

            self._sleep_with_spin(settle)

        current_x, current_y = self.wait_for_pose(timeout_sec=0.1)
        final_distance = math.hypot(target_x - current_x, target_y - current_y)
        if final_distance <= goal_tolerance:
            self.get_logger().info(
                f"Goto target reached after final settle: "
                f"error={final_distance:.3f} m"
            )
            self._wait_after(wait_after)
            return True

        raise GotoError(
            "Goto iteration limit reached: "
            f"target=({target_x:.3f}, {target_y:.3f}) m, "
            f"last_error={final_distance:.3f} m "
            f"(previous_error={last_distance:.3f} m)."
        )

    def wait_for_pose(self, timeout_sec: Optional[float] = None) -> Tuple[float, float]:
        deadline = None
        if timeout_sec is not None:
            deadline = time.monotonic() + max(0.0, float(timeout_sec))

        while rclpy.ok():
            if self._latest_pose_xy is not None:
                return self._latest_pose_xy

            if deadline is not None and time.monotonic() >= deadline:
                raise GotoError(
                    f"Timed out waiting for pose on topic '{self.pose_topic}'."
                )

            self._spin_once_until(deadline)

        raise GotoError("ROS shutdown while waiting for pose.")

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = float(position.x)
        y = float(position.y)
        yaw_rad = self._quaternion_to_yaw_rad(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw_rad):
            self.get_logger().warn("Ignoring non-finite /amcl_pose.")
            return
        self._latest_pose_xy = (x, y)
        self._latest_pose_yaw_rad = yaw_rad
        self._latest_pose_received_time = time.monotonic()

    def _quaternion_to_yaw_rad(
        self, x: float, y: float, z: float, w: float
    ) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _validate_parameters(self) -> None:
        if not self.pose_topic:
            raise RuntimeError("Parameter 'pose_topic' must not be empty.")
        if not self.motion_action_name:
            raise RuntimeError("Parameter 'motion_action_name' must not be empty.")
        if not math.isfinite(self.goal_tolerance_m) or self.goal_tolerance_m <= 0.0:
            raise RuntimeError("Parameter 'goal_tolerance_m' must be positive.")
        if not math.isfinite(self.max_step_m) or self.max_step_m <= 0.0:
            raise RuntimeError("Parameter 'max_step_m' must be positive.")
        if not math.isfinite(self.settle_sec) or self.settle_sec < 0.0:
            raise RuntimeError("Parameter 'settle_sec' must be non-negative.")
        if self.max_iterations <= 0:
            raise RuntimeError("Parameter 'max_iterations' must be positive.")
        if not math.isfinite(self.goto_timeout_sec) or self.goto_timeout_sec <= 0.0:
            raise RuntimeError("Parameter 'goto_timeout_sec' must be positive.")
        if (
            not math.isfinite(self.pose_wait_timeout_sec)
            or self.pose_wait_timeout_sec <= 0.0
        ):
            raise RuntimeError("Parameter 'pose_wait_timeout_sec' must be positive.")
        if (
            not math.isfinite(self.action_server_wait_sec)
            or self.action_server_wait_sec <= 0.0
        ):
            raise RuntimeError("Parameter 'action_server_wait_sec' must be positive.")

    def _ensure_action_server_ready(
        self, deadline: float, action_server_wait_sec: float
    ) -> None:
        wait_sec = self._bounded_timeout(deadline, action_server_wait_sec)
        if not self._motion_client.wait_for_server(timeout_sec=wait_sec):
            raise GotoError(
                f"Timed out waiting for motion action server "
                f"'{self.motion_action_name}'."
            )

    def _send_motion_command(self, command: str, timeout_sec: float) -> bool:
        if self._motion_in_flight:
            raise GotoError(
                "Refusing to send a motion command while another one is in flight."
            )

        command_deadline = time.monotonic() + max(0.0, float(timeout_sec))
        goal = Stm32Motion.Goal()
        goal.command = command

        self._motion_in_flight = True
        try:
            goal_future = self._motion_client.send_goal_async(goal)
            if not self._spin_until_future_done(
                goal_future, self._remaining_time(command_deadline)
            ):
                raise GotoError(
                    f"Timed out sending motion goal '{command}'. "
                    "No new command will be sent while the goal state is unknown."
                )

            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"Motion goal rejected: {command}")
                return False

            result_future = goal_handle.get_result_async()
            if not self._spin_until_future_done(
                result_future, self._remaining_time(command_deadline)
            ):
                raise GotoError(
                    f"Timed out waiting for motion result for '{command}'. "
                    "No new command will be sent while the goal state is unknown."
                )

            result_response = result_future.result()
            result = result_response.result
            if result.success:
                self.get_logger().info(
                    f"Motion command ok: {command} "
                    f"(status={result.status}, attempts={result.attempts})"
                )
                return True

            self.get_logger().warn(
                f"Motion command failed: {command} "
                f"(status={result.status}, attempts={result.attempts}, "
                f"message={result.message})"
            )
            return False
        finally:
            self._motion_in_flight = False

    def _limit_step(
        self, dx_map: float, dy_map: float, max_step_m: float
    ) -> Tuple[float, float]:
        distance = math.hypot(dx_map, dy_map)
        if distance <= max_step_m:
            return dx_map, dy_map
        scale = max_step_m / distance
        return dx_map * scale, dy_map * scale

    def _map_delta_to_stm32_delta(
        self, dx_map_m: float, dy_map_m: float
    ) -> Tuple[float, float]:
        # Map frame: +x is right, +y is up.
        # STM32 cmd_dis frame: +x is map-left, +y is map-down.
        # Both frames are field-fixed; robot yaw is not part of this conversion.
        return -dx_map_m, -dy_map_m

    def _spin_until_future_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return future.done()
            self._spin_once_until(deadline)
        return future.done()

    def _sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once_until(deadline)

    def _wait_after(self, wait_sec: float) -> None:
        self._sleep_with_spin(self._non_negative_float("wait_sec", wait_sec))

    def _spin_once_until(self, deadline: Optional[float]) -> None:
        timeout_sec = 0.05
        if deadline is not None:
            timeout_sec = max(0.0, min(timeout_sec, deadline - time.monotonic()))
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _bounded_timeout(self, deadline: float, preferred_timeout_sec: float) -> float:
        return max(0.0, min(float(preferred_timeout_sec), self._remaining_time(deadline)))

    def _remaining_time(self, deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def _positive_float(self, name: str, value: float) -> float:
        converted = float(value)
        if not math.isfinite(converted) or converted <= 0.0:
            raise ValueError(f"{name} must be a positive finite number.")
        return converted

    def _non_negative_float(self, name: str, value: float) -> float:
        converted = float(value)
        if not math.isfinite(converted) or converted < 0.0:
            raise ValueError(f"{name} must be a non-negative finite number.")
        return converted

    def _positive_int(self, name: str, value: int) -> int:
        converted = int(value)
        if converted <= 0:
            raise ValueError(f"{name} must be a positive integer.")
        return converted
