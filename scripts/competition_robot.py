#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy

from goto_point import GotoError, GotoNavigator, _format_number
from rcj_localization.srv import Stm32Command


@dataclass(frozen=True)
class Stm32State:
    dx: float
    dy: float
    dtheta: float
    theta: float
    attempts: int
    message: str


class CompetitionRobot(GotoNavigator):
    """High-level API for writing competition task logic in Python."""

    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("stm32_command_service", "/stm32/send_command")
        self.declare_parameter("command_call_timeout_sec", 1.0)
        self.declare_parameter("command_retry_delay_sec", 0.1)
        self.declare_parameter("command_service_wait_sec", 1.0)
        self.declare_parameter("motion_retry_timeout_sec", 20.0)

        self.stm32_command_service = str(
            self.get_parameter("stm32_command_service").value
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

        self._command_client = self.create_client(
            Stm32Command, self.stm32_command_service
        )
        self.get_logger().info(
            "CompetitionRobot ready: "
            f"command_service='{self.stm32_command_service}', "
            f"motion_action='{self.motion_action_name}'"
        )

    def turn(
        self,
        degrees: float,
        retry_delay_sec: Optional[float] = None,
        motion_timeout_sec: Optional[float] = None,
    ) -> bool:
        yaw_degrees = float(degrees)
        if not math.isfinite(yaw_degrees):
            raise ValueError("turn degrees must be finite.")
        command = f"cmd_turn {_format_number(yaw_degrees)}"
        return self._send_motion_until_success(
            command,
            retry_delay_sec=retry_delay_sec,
            motion_timeout_sec=motion_timeout_sec,
        )

    def suck(
        self,
        speed: int,
        retry_delay_sec: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        speed_int = int(speed)
        if speed_int < 0 or speed_int > 100:
            raise ValueError("suck speed must be in range 0-100.")
        self._send_command_until_success(
            f"cmd_suck {speed_int}",
            retry_delay_sec=retry_delay_sec,
            timeout_sec=timeout_sec,
        )
        return True

    def suck_on(self, speed: int = 100) -> bool:
        return self.suck(speed)

    def suck_off(self) -> bool:
        return self.suck(0)

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

    def sleep(self, seconds: float) -> None:
        self._sleep_with_spin(self._non_negative_float("seconds", seconds))

    def _send_motion_until_success(
        self,
        command: str,
        retry_delay_sec: Optional[float] = None,
        motion_timeout_sec: Optional[float] = None,
    ) -> bool:
        retry_delay = self._retry_delay(retry_delay_sec)
        command_timeout = self._positive_float(
            "motion_timeout_sec",
            self.motion_retry_timeout_sec
            if motion_timeout_sec is None
            else motion_timeout_sec,
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
