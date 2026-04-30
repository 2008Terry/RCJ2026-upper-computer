#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rcj_localization.action import Stm32Motion


def _format_number(value: float) -> str:
    if math.isclose(value, round(value), abs_tol=1e-9):
        return str(int(round(value)))
    return f"{value:.6f}".rstrip("0").rstrip(".")


class Stm32SquareLoopNode(Node):
    def __init__(self) -> None:
        super().__init__("stm32_square_loop_node")

        self.declare_parameter("motion_action_name", "/stm32/motion")
        self.declare_parameter("side_cm", 20.0)
        self.declare_parameter("pause_sec", 0.2)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("action_wait_timeout_sec", 1.0)
        self.declare_parameter("retry_on_failure", True)
        self.declare_parameter("max_cycles", 0)

        self._motion_action_name = self.get_parameter("motion_action_name").value
        side_cm = float(self.get_parameter("side_cm").value)
        self._pause_sec = max(0.0, float(self.get_parameter("pause_sec").value))
        self._action_wait_timeout_sec = max(
            0.1, float(self.get_parameter("action_wait_timeout_sec").value)
        )
        self._retry_on_failure = bool(self.get_parameter("retry_on_failure").value)
        self._max_cycles = max(0, int(self.get_parameter("max_cycles").value))

        if side_cm <= 0.0:
            raise RuntimeError("side_cm must be positive")

        side = _format_number(side_cm)
        minus_side = _format_number(-side_cm)
        # STM32 cmd_dis axes are field-fixed: +x map-left, +y map-down.
        self._commands = [
            f"cmd_dis {side} 0 1",
            f"cmd_dis 0 {side} 1",
            f"cmd_dis {minus_side} 0 1",
            f"cmd_dis 0 {minus_side} 1",
        ]

        self._client = ActionClient(self, Stm32Motion, self._motion_action_name)
        self._pending_goal_future: Optional[rclpy.task.Future] = None
        self._pending_result_future: Optional[rclpy.task.Future] = None
        self._pending_command: Optional[str] = None
        self._next_send_time = time.monotonic() + max(
            0.0, float(self.get_parameter("start_delay_sec").value)
        )
        self._command_index = 0
        self._cycles_completed = 0

        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Square loop ready: side={side_cm:.3f} cm, "
            f"commands={self._commands}, motion_action='{self._motion_action_name}'"
        )

    def _tick(self) -> None:
        if self._max_cycles > 0 and self._cycles_completed >= self._max_cycles:
            return

        if not self._client.server_is_ready():
            self._client.wait_for_server(timeout_sec=self._action_wait_timeout_sec)
            return

        if self._pending_goal_future is None and self._pending_result_future is None:
            if time.monotonic() < self._next_send_time:
                return
            self._send_current_command()
            return

        if self._pending_goal_future is not None:
            if not self._pending_goal_future.done():
                return
            self._handle_goal_response()
            return

        if self._pending_result_future is None or not self._pending_result_future.done():
            return

        try:
            result_response = self._pending_result_future.result()
            response = result_response.result
        except Exception as error:  # pragma: no cover - depends on ROS runtime
            self.get_logger().error(f"STM32 motion action failed: {error}")
            self._finish_current_command(False, "action_error", str(error))
            return

        self._finish_current_command(response.success, response.status, response.message, response.attempts)

    def _handle_goal_response(self) -> None:
        command = self._pending_command or self._commands[self._command_index]
        try:
            goal_handle = self._pending_goal_future.result()
        except Exception as error:  # pragma: no cover - depends on ROS runtime
            self.get_logger().error(f"STM32 motion goal request failed: {error}")
            self._finish_current_command(False, "goal_error", str(error))
            return

        self._pending_goal_future = None
        if not goal_handle.accepted:
            self.get_logger().warn(f"STM32 motion goal rejected: {command}")
            self._finish_current_command(False, "goal_rejected", "Motion action goal was rejected.")
            return

        self._pending_result_future = goal_handle.get_result_async()

    def _finish_current_command(
        self,
        success: bool,
        status: str,
        message: str,
        attempts: int = 0,
    ) -> None:
        command = self._pending_command or self._commands[self._command_index]
        if success:
            self.get_logger().info(
                f"STM32 command ok: {command} (attempts={attempts})"
            )
            self._advance_command()
        else:
            self.get_logger().warn(
                f"STM32 command failed: {command} "
                f"status={status} message={message}"
            )
            if not self._retry_on_failure:
                self._advance_command()

        self._pending_goal_future = None
        self._pending_result_future = None
        self._pending_command = None
        self._next_send_time = time.monotonic() + self._pause_sec

    def _send_current_command(self) -> None:
        if self._pending_goal_future is not None or self._pending_result_future is not None:
            return

        command = self._commands[self._command_index]
        goal = Stm32Motion.Goal()
        goal.command = command
        self.get_logger().info(f"Sending STM32 command: {command}")
        self._pending_command = command
        self._pending_goal_future = self._client.send_goal_async(goal)

    def _advance_command(self) -> None:
        self._command_index = (self._command_index + 1) % len(self._commands)
        if self._command_index == 0:
            self._cycles_completed += 1
            if self._max_cycles > 0 and self._cycles_completed >= self._max_cycles:
                self.get_logger().info(
                    f"Square loop completed {self._cycles_completed} cycle(s)."
                )
            else:
                self.get_logger().info(
                    f"Square loop completed {self._cycles_completed} cycle(s); "
                    "continuing."
                )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = Stm32SquareLoopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
