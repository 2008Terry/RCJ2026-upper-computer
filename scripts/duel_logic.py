from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy


STATE_DEFENSE = "回防"
STATE_FIND_BALL = "找球"
STATE_KICK_BALL = "踢球"

# Field assumptions for the default competition setup:
# - own half is map -y
# - attack direction is map +y, STM32 yaw 0 deg
ATTACK_YAW_DEG = 0.0
DEFENSE_POINT_X_M = 0.0
DEFENSE_POINT_Y_M = -0.65

FIELD_X_MIN_M = -0.72
FIELD_X_MAX_M = 0.72
FIELD_Y_MIN_M = -1.00
FIELD_Y_MAX_M = 1.00

VISION_TIMEOUT_SEC = 0.04
MIN_BALL_CONFIDENCE = 0.5
POSE_TIMEOUT_SEC = 0.2
LOOP_SLEEP_SEC = 0.04
DRIVE_RESEND_SEC = 0.16
ANGLE_RESEND_DELTA_DEG = 6.0

DEFENSE_TOLERANCE_M = 0.08
DEFENSE_SPEED_PERCENT = 18
CHASE_SPEED_PERCENT = 24
IR_CHASE_SPEED_PERCENT = 18
KICK_DRIVE_SPEED_PERCENT = 20

CHASE_BEHIND_OFFSET_M = 0.24
CHASE_BEHIND_TOLERANCE_M = 0.08
KICK_DISTANCE_M = 0.28
KICK_ANGLE_DEG = 18.0
KICK_SUCK_SPEED_PERCENT = 60

# Tune this if the BE-1732 physical channel order is mounted differently.
IR_CHANNEL_TO_MOVE_ANGLE_DEG = {
    1: -90.0,
    2: -60.0,
    3: -30.0,
    4: 0.0,
    5: 30.0,
    6: 60.0,
    7: 90.0,
}


@dataclass(frozen=True)
class BallCue:
    ball: Optional[Any]
    infrared_channel: Optional[int]

    @property
    def source(self) -> str:
        if self.ball is not None:
            return "vision"
        return "infrared"


class StateReporter:
    def __init__(self, robot: Any) -> None:
        self._robot = robot
        self._state: Optional[str] = None

    def set(self, state: str, detail: str = "") -> None:
        if state == self._state:
            return
        self._state = state
        suffix = f": {detail}" if detail else ""
        self._robot.get_logger().info(f"状态机 -> {state}{suffix}")


class DriveCache:
    def __init__(self) -> None:
        self._last_speed: Optional[int] = None
        self._last_angle_deg: Optional[float] = None
        self._last_sent_time = 0.0

    def drive(self, robot: Any, *, speed_percent: int, move_angle_deg: float) -> None:
        angle = _normalize_angle_deg(move_angle_deg)
        now = time.monotonic()
        should_resend = (
            self._last_speed != speed_percent
            or self._last_angle_deg is None
            or abs(_normalize_angle_deg(angle - self._last_angle_deg))
            >= ANGLE_RESEND_DELTA_DEG
            or now - self._last_sent_time >= DRIVE_RESEND_SEC
        )
        if not should_resend:
            return

        robot.drive(
            speed_percent=speed_percent,
            move_angle_deg=angle,
            head_lock=True,
        )
        self._last_speed = speed_percent
        self._last_angle_deg = angle
        self._last_sent_time = now

    def stop(self, robot: Any) -> None:
        if self._last_speed == 0:
            return
        robot.stop()
        self._last_speed = 0
        self._last_angle_deg = None
        self._last_sent_time = time.monotonic()


class SuckCache:
    def __init__(self) -> None:
        self._last_speed: Optional[int] = None

    def set(self, robot: Any, speed_percent: int) -> None:
        if self._last_speed == speed_percent:
            return
        robot.suck(speed_percent=speed_percent)
        self._last_speed = speed_percent


def run_task(robot: Any) -> None:
    """Run a head-lock duel state machine until ROS shuts down or Ctrl-C."""

    reporter = StateReporter(robot)
    drive_cache = DriveCache()
    suck_cache = SuckCache()

    robot.motion_enable()
    robot.infrared_modulated_mode()
    robot.suck_off()
    robot.turn(angle_deg=ATTACK_YAW_DEG)

    try:
        while rclpy.ok():
            cue = _sense_ball(robot)

            if cue is None:
                reporter.set(STATE_DEFENSE)
                suck_cache.set(robot, 0)
                _handle_defense(robot, drive_cache)
            elif cue.ball is not None and _should_kick(cue.ball):
                reporter.set(STATE_KICK_BALL, "ball close and centered")
                _handle_kick(robot, drive_cache, suck_cache)
            else:
                reporter.set(STATE_FIND_BALL, cue.source)
                suck_cache.set(robot, 0)
                _handle_find_ball(robot, drive_cache, cue)

            robot.timer(duration_sec=LOOP_SLEEP_SEC)
    finally:
        _safe_stop(robot, drive_cache, suck_cache)


def _sense_ball(robot: Any) -> Optional[BallCue]:
    ball = robot.find_ball(
        timeout_sec=VISION_TIMEOUT_SEC,
        min_confidence=MIN_BALL_CONFIDENCE,
    )
    if ball is not None:
        return BallCue(ball=ball, infrared_channel=None)

    channel = _read_infrared_channel(robot)
    if channel is not None:
        return BallCue(ball=None, infrared_channel=channel)
    return None


def _read_infrared_channel(robot: Any) -> Optional[int]:
    try:
        channel = robot.infrared_channel()
    except Exception as error:
        robot.get_logger().warn(f"Infrared read skipped: {error}")
        return None

    if channel not in IR_CHANNEL_TO_MOVE_ANGLE_DEG:
        return None
    return channel


def _handle_defense(robot: Any, drive_cache: DriveCache) -> None:
    reached = _drive_towards_map_point(
        robot,
        drive_cache,
        target_x_m=DEFENSE_POINT_X_M,
        target_y_m=DEFENSE_POINT_Y_M,
        speed_percent=DEFENSE_SPEED_PERCENT,
        tolerance_m=DEFENSE_TOLERANCE_M,
    )
    if reached:
        drive_cache.stop(robot)


def _handle_find_ball(robot: Any, drive_cache: DriveCache, cue: BallCue) -> None:
    if cue.ball is None:
        channel = cue.infrared_channel
        if channel is None:
            drive_cache.stop(robot)
            return
        drive_cache.drive(
            robot,
            speed_percent=IR_CHASE_SPEED_PERCENT,
            move_angle_deg=IR_CHANNEL_TO_MOVE_ANGLE_DEG[channel],
        )
        return

    ball = cue.ball
    target_x_m = _clamp(ball.absolute_x_m, FIELD_X_MIN_M, FIELD_X_MAX_M)
    target_y_m = _clamp(
        ball.absolute_y_m - CHASE_BEHIND_OFFSET_M,
        FIELD_Y_MIN_M,
        FIELD_Y_MAX_M,
    )

    reached_behind_point = _drive_towards_map_point(
        robot,
        drive_cache,
        target_x_m=target_x_m,
        target_y_m=target_y_m,
        speed_percent=CHASE_SPEED_PERCENT,
        tolerance_m=CHASE_BEHIND_TOLERANCE_M,
    )
    if reached_behind_point:
        drive_cache.drive(
            robot,
            speed_percent=CHASE_SPEED_PERCENT,
            move_angle_deg=ball.angle_deg,
        )


def _handle_kick(
    robot: Any,
    drive_cache: DriveCache,
    suck_cache: SuckCache,
) -> None:
    suck_cache.set(robot, KICK_SUCK_SPEED_PERCENT)
    drive_cache.drive(
        robot,
        speed_percent=KICK_DRIVE_SPEED_PERCENT,
        move_angle_deg=0.0,
    )


def _drive_towards_map_point(
    robot: Any,
    drive_cache: DriveCache,
    *,
    target_x_m: float,
    target_y_m: float,
    speed_percent: int,
    tolerance_m: float,
) -> bool:
    pose = robot.get_pose(timeout_sec=POSE_TIMEOUT_SEC)
    dx_m = target_x_m - pose.x_m
    dy_m = target_y_m - pose.y_m
    distance_m = math.hypot(dx_m, dy_m)
    if distance_m <= tolerance_m:
        return True

    map_angle_deg = math.degrees(math.atan2(dy_m, dx_m))
    robot_relative_angle_deg = _normalize_angle_deg(map_angle_deg - pose.yaw_deg)
    drive_cache.drive(
        robot,
        speed_percent=speed_percent,
        move_angle_deg=robot_relative_angle_deg,
    )
    return False


def _should_kick(ball: Any) -> bool:
    return (
        0.0 < ball.base_x_m <= KICK_DISTANCE_M
        and abs(ball.angle_deg) <= KICK_ANGLE_DEG
    )


def _safe_stop(robot: Any, drive_cache: DriveCache, suck_cache: SuckCache) -> None:
    for action in (
        lambda: drive_cache.stop(robot),
        lambda: suck_cache.set(robot, 0),
    ):
        try:
            action()
        except Exception as error:
            robot.get_logger().warn(f"Shutdown cleanup skipped: {error}")


def _normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
