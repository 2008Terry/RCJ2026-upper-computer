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
        self.declare_parameter("goal_tolerance_m", 0.03)
        self.declare_parameter("max_step_m", 0.40)
        self.declare_parameter("settle_sec", 0.5)
        self.declare_parameter("max_iterations", 20)
        self.declare_parameter("goto_timeout_sec", 30.0)
        self.declare_parameter("pose_wait_timeout_sec", 5.0)
        self.declare_parameter("action_server_wait_sec", 2.0)
        self.declare_parameter("yaw_zero_map_degrees", 0.0)

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
        self.yaw_zero_map_degrees = float(
            self.get_parameter("yaw_zero_map_degrees").value
        )

        self._validate_parameters()

        self._latest_pose_xy: Optional[Tuple[float, float]] = None
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
            "GotoNavigator ready: pose_topic='%s', motion_action='%s', "
            "tolerance=%.3f m, max_step=%.3f m, settle=%.3f s, "
            "max_iterations=%d, timeout=%.3f s, yaw_zero_map_degrees=%.3f",
            self.pose_topic,
            self.motion_action_name,
            self.goal_tolerance_m,
            self.max_step_m,
            self.settle_sec,
            self.max_iterations,
            self.goto_timeout_sec,
            self.yaw_zero_map_degrees,
        )

    def goto(
        self,
        target_x_m: float,
        target_y_m: float,
        tolerance_m: Optional[float] = None,
    ) -> bool:
        target_x = float(target_x_m)
        target_y = float(target_y_m)
        tolerance = self.goal_tolerance_m if tolerance_m is None else float(tolerance_m)
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            raise ValueError("goto target coordinates must be finite.")
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("goto tolerance must be a positive finite number.")

        deadline = time.monotonic() + self.goto_timeout_sec
        self.get_logger().info(
            "Goto target start: target=(%.3f, %.3f) m, tolerance=%.3f m",
            target_x,
            target_y,
            tolerance,
        )

        self._ensure_action_server_ready(deadline)
        last_distance = math.inf

        for iteration in range(1, self.max_iterations + 1):
            current_x, current_y = self.wait_for_pose(
                timeout_sec=self._bounded_timeout(deadline, self.pose_wait_timeout_sec)
            )
            dx_map = target_x - current_x
            dy_map = target_y - current_y
            distance = math.hypot(dx_map, dy_map)
            last_distance = distance

            if distance <= tolerance:
                self.get_logger().info(
                    "Goto target reached: target=(%.3f, %.3f) m, "
                    "current=(%.3f, %.3f) m, error=%.3f m",
                    target_x,
                    target_y,
                    current_x,
                    current_y,
                    distance,
                )
                return True

            if time.monotonic() >= deadline:
                raise GotoError(
                    "Goto timed out before target was reached: "
                    f"target=({target_x:.3f}, {target_y:.3f}) m, "
                    f"last_error={distance:.3f} m."
                )

            step_dx_map, step_dy_map = self._limit_step(dx_map, dy_map)
            step_dx_stm32_m, step_dy_stm32_m = self._map_delta_to_stm32_delta(
                step_dx_map, step_dy_map
            )
            command = (
                "cmd_dis "
                f"{_format_number(step_dx_stm32_m * 100.0)} "
                f"{_format_number(step_dy_stm32_m * 100.0)}"
            )

            self.get_logger().info(
                "Goto iteration %d/%d: current=(%.3f, %.3f) m, "
                "target=(%.3f, %.3f) m, error=%.3f m, command='%s'",
                iteration,
                self.max_iterations,
                current_x,
                current_y,
                target_x,
                target_y,
                distance,
                command,
            )

            motion_ok = self._send_motion_command(
                command,
                timeout_sec=self._remaining_time(deadline),
            )
            if not motion_ok:
                self.get_logger().warn(
                    "Motion command did not succeed; settling for %.3f s "
                    "before replanning from the latest pose.",
                    self.settle_sec,
                )

            self._sleep_with_spin(self.settle_sec)

        current_x, current_y = self.wait_for_pose(timeout_sec=0.1)
        final_distance = math.hypot(target_x - current_x, target_y - current_y)
        if final_distance <= tolerance:
            self.get_logger().info(
                "Goto target reached after final settle: error=%.3f m",
                final_distance,
            )
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
        x = float(position.x)
        y = float(position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            self.get_logger().warn("Ignoring non-finite /amcl_pose position.")
            return
        self._latest_pose_xy = (x, y)
        self._latest_pose_received_time = time.monotonic()

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
        if not math.isfinite(self.yaw_zero_map_degrees):
            raise RuntimeError("Parameter 'yaw_zero_map_degrees' must be finite.")

    def _ensure_action_server_ready(self, deadline: float) -> None:
        wait_sec = self._bounded_timeout(deadline, self.action_server_wait_sec)
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
                self.get_logger().warn("Motion goal rejected: %s", command)
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
                    "Motion command ok: %s (status=%s, attempts=%d)",
                    command,
                    result.status,
                    result.attempts,
                )
                return True

            self.get_logger().warn(
                "Motion command failed: %s (status=%s, attempts=%d, message=%s)",
                command,
                result.status,
                result.attempts,
                result.message,
            )
            return False
        finally:
            self._motion_in_flight = False

    def _limit_step(self, dx_map: float, dy_map: float) -> Tuple[float, float]:
        distance = math.hypot(dx_map, dy_map)
        if distance <= self.max_step_m:
            return dx_map, dy_map
        scale = self.max_step_m / distance
        return dx_map * scale, dy_map * scale

    def _map_delta_to_stm32_delta(
        self, dx_map_m: float, dy_map_m: float
    ) -> Tuple[float, float]:
        # Match amcl_fusion.cpp: STM32 dx is a fixed field axis, not current yaw.
        axis_rad = math.radians(180.0 + self.yaw_zero_map_degrees)
        cos_axis = math.cos(axis_rad)
        sin_axis = math.sin(axis_rad)
        dx_stm32_m = (cos_axis * dx_map_m) + (sin_axis * dy_map_m)
        dy_stm32_m = (-sin_axis * dx_map_m) + (cos_axis * dy_map_m)
        return dx_stm32_m, dy_stm32_m

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

    def _spin_once_until(self, deadline: Optional[float]) -> None:
        timeout_sec = 0.05
        if deadline is not None:
            timeout_sec = max(0.0, min(timeout_sec, deadline - time.monotonic()))
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _bounded_timeout(self, deadline: float, preferred_timeout_sec: float) -> float:
        return max(0.0, min(float(preferred_timeout_sec), self._remaining_time(deadline)))

    def _remaining_time(self, deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())
