#!/usr/bin/env python3

from __future__ import annotations

from typing import Any, Optional

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


def start_signal_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class CompetitionStartGate:
    """Wait for the launch-side start key after the runner is preflighted."""

    def __init__(self, node: Any, task_name: str) -> None:
        self._node = node
        self._task_name = task_name
        self._started = False
        self._subscription: Optional[Any] = None

        self._node.declare_parameter("wait_for_start", False)
        self._node.declare_parameter("start_topic", "/competition/start")

        self.wait_for_start = bool(
            self._node.get_parameter("wait_for_start").value
        )
        self.start_topic = str(self._node.get_parameter("start_topic").value)

        if self.wait_for_start:
            self._subscription = self._node.create_subscription(
                Empty,
                self.start_topic,
                self._start_callback,
                start_signal_qos(),
            )

    def _start_callback(self, _msg: Empty) -> None:
        if self._started:
            return
        self._started = True
        self._node.get_logger().info(
            f"{self._task_name}: start signal received on '{self.start_topic}'."
        )

    def wait(self) -> None:
        if not self.wait_for_start:
            return

        self._node.get_logger().info(
            f"{self._task_name}: preflight complete. Press s in the launch "
            "terminal to start task logic."
        )
        while rclpy.ok() and not self._started:
            rclpy.spin_once(self._node, timeout_sec=0.05)

        if not rclpy.ok():
            raise RuntimeError(
                f"ROS shutdown while {self._task_name} waited for start signal."
            )


def wait_for_ready_and_start(robot: Any, task_name: str) -> None:
    gate = CompetitionStartGate(robot, task_name)
    robot.wait_until_ready()
    gate.wait()
