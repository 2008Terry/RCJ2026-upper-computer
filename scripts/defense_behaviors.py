from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional

from infrared_smoothing import InfraredAngleFilter


# Default field setup:
# - own goal is behind map -y
# - robot faces map +y while defending, which is STM32 yaw 0 deg
DEFENSE_HEAD_YAW_DEG = 0.0
DEFENSE_LINE_CENTER_X_M = 0.0
DEFENSE_LINE_Y_M = -0.65
DEFENSE_LINE_HALF_LENGTH_M = 0.35

VISION_TIMEOUT_SEC = 0.04
MIN_BALL_CONFIDENCE = 0.5
POSE_TIMEOUT_SEC = 0.2
LOOP_SLEEP_SEC = 0.04

DRIVE_RESEND_SEC = 0.14
ANGLE_RESEND_DELTA_DEG = 5.0

VISION_TRACK_SPEED_PERCENT = 20
IR_TRACK_MIN_SPEED_PERCENT = 12
IR_TRACK_MAX_SPEED_PERCENT = 18
RETURN_SPEED_PERCENT = 16

VISION_X_TOLERANCE_M = 0.035
LINE_Y_TOLERANCE_M = 0.045
NEUTRAL_TOLERANCE_M = 0.045

IR_DEADBAND_DEG = 8.0
IR_HISTORY_SIZE = 8
IR_EWMA_ALPHA = 0.35
IR_CLEAR_AFTER_MISSES = 3
IR_STEP_MIN_M = 0.06
IR_STEP_MAX_M = 0.20

# Tune this if the BE-1732 physical channel order is mounted differently.
IR_CHANNEL_TO_BEARING_DEG = {
    1: -90.0,
    2: -60.0,
    3: -30.0,
    4: 0.0,
    5: 30.0,
    6: 60.0,
    7: 90.0,
}


@dataclass(frozen=True)
class DefenseCue:
    ball: Optional[Any]
    infrared_channel: Optional[int]
    infrared_angle_deg: Optional[float]

    @property
    def source(self) -> str:
        if self.ball is not None:
            return "vision"
        if self.infrared_angle_deg is not None:
            return "infrared"
        return "none"


class DefenseRuntime:
    def __init__(self, robot: Any) -> None:
        self.drive = HeadLockDriveCache()
        self.infrared = InfraredAngleFilter(
            channel_to_angle_deg=IR_CHANNEL_TO_BEARING_DEG,
            history_size=IR_HISTORY_SIZE,
            ewma_alpha=IR_EWMA_ALPHA,
            clear_after_misses=IR_CLEAR_AFTER_MISSES,
        )
        self._robot = robot
        self._state: Optional[str] = None

    def report_state(self, state: str, detail: str = "") -> None:
        if state == self._state:
            return
        self._state = state
        suffix = f": {detail}" if detail else ""
        self._robot.get_logger().info(f"Defense state -> {state}{suffix}")


class HeadLockDriveCache:
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


def prepare_defense(robot: Any) -> None:
    robot.motion_enable()
    robot.infrared_modulated_mode()
    robot.suck_off()
    robot.turn(angle_deg=DEFENSE_HEAD_YAW_DEG)


def sense_defense_cue(
    robot: Any,
    infrared_filter: InfraredAngleFilter,
) -> DefenseCue:
    ball = robot.find_ball(
        timeout_sec=VISION_TIMEOUT_SEC,
        min_confidence=MIN_BALL_CONFIDENCE,
    )
    if ball is not None:
        return DefenseCue(
            ball=ball,
            infrared_channel=None,
            infrared_angle_deg=None,
        )

    channel = _read_infrared_channel(robot)
    if channel is None:
        infrared_filter.mark_missed()
        return DefenseCue(
            ball=None,
            infrared_channel=None,
            infrared_angle_deg=None,
        )

    angle = infrared_filter.update(channel)
    return DefenseCue(
        ball=None,
        infrared_channel=channel,
        infrared_angle_deg=angle,
    )


def handle_vision_defense(
    robot: Any,
    drive_cache: HeadLockDriveCache,
    ball: Any,
) -> bool:
    target_x_m = clamp_to_defense_line(ball.absolute_x_m)
    return _drive_towards_map_point(
        robot,
        drive_cache,
        target_x_m=target_x_m,
        target_y_m=DEFENSE_LINE_Y_M,
        speed_percent=VISION_TRACK_SPEED_PERCENT,
        x_tolerance_m=VISION_X_TOLERANCE_M,
        y_tolerance_m=LINE_Y_TOLERANCE_M,
    )


def handle_infrared_defense(
    robot: Any,
    drive_cache: HeadLockDriveCache,
    angle_deg: float,
) -> bool:
    pose = robot.get_pose(timeout_sec=POSE_TIMEOUT_SEC)
    bounded_x_m = clamp_to_defense_line(pose.x_m)

    if abs(angle_deg) <= IR_DEADBAND_DEG:
        return _drive_towards_map_point(
            robot,
            drive_cache,
            target_x_m=bounded_x_m,
            target_y_m=DEFENSE_LINE_Y_M,
            speed_percent=IR_TRACK_MIN_SPEED_PERCENT,
            x_tolerance_m=VISION_X_TOLERANCE_M,
            y_tolerance_m=LINE_Y_TOLERANCE_M,
            pose=pose,
        )

    # Positive IR angle means the source is to robot-left, so move map-left
    # (smaller ROS map x) while holding yaw.
    step_m = _scale_ir_step(abs(angle_deg))
    direction = -1.0 if angle_deg > 0.0 else 1.0
    target_x_m = clamp_to_defense_line(bounded_x_m + direction * step_m)
    speed_percent = _scale_ir_speed(abs(angle_deg))

    return _drive_towards_map_point(
        robot,
        drive_cache,
        target_x_m=target_x_m,
        target_y_m=DEFENSE_LINE_Y_M,
        speed_percent=speed_percent,
        x_tolerance_m=VISION_X_TOLERANCE_M,
        y_tolerance_m=LINE_Y_TOLERANCE_M,
        pose=pose,
    )


def handle_neutral_defense(
    robot: Any,
    drive_cache: HeadLockDriveCache,
) -> bool:
    return _drive_towards_map_point(
        robot,
        drive_cache,
        target_x_m=DEFENSE_LINE_CENTER_X_M,
        target_y_m=DEFENSE_LINE_Y_M,
        speed_percent=RETURN_SPEED_PERCENT,
        x_tolerance_m=NEUTRAL_TOLERANCE_M,
        y_tolerance_m=LINE_Y_TOLERANCE_M,
    )


def safe_stop(robot: Any, drive_cache: HeadLockDriveCache) -> None:
    for action in (
        lambda: drive_cache.stop(robot),
        lambda: robot.suck_off(),
    ):
        try:
            action()
        except Exception as error:
            robot.get_logger().warn(f"Defense shutdown cleanup skipped: {error}")


def clamp_to_defense_line(x_m: float) -> float:
    return _clamp(x_m, _line_x_min_m(), _line_x_max_m())


def _drive_towards_map_point(
    robot: Any,
    drive_cache: HeadLockDriveCache,
    *,
    target_x_m: float,
    target_y_m: float,
    speed_percent: int,
    x_tolerance_m: float,
    y_tolerance_m: float,
    pose: Optional[Any] = None,
) -> bool:
    current_pose = pose if pose is not None else robot.get_pose(timeout_sec=POSE_TIMEOUT_SEC)
    dx_m = target_x_m - current_pose.x_m
    dy_m = target_y_m - current_pose.y_m

    if abs(dx_m) <= x_tolerance_m and abs(dy_m) <= y_tolerance_m:
        drive_cache.stop(robot)
        return True

    map_angle_deg = math.degrees(math.atan2(dy_m, dx_m))
    robot_relative_angle_deg = _normalize_angle_deg(map_angle_deg - current_pose.yaw_deg)
    drive_cache.drive(
        robot,
        speed_percent=speed_percent,
        move_angle_deg=robot_relative_angle_deg,
    )
    return False


def _read_infrared_channel(robot: Any) -> Optional[int]:
    try:
        channel = robot.infrared_channel()
    except Exception as error:
        robot.get_logger().warn(f"Infrared read skipped: {error}")
        return None

    if channel not in IR_CHANNEL_TO_BEARING_DEG:
        return None
    return channel


def _scale_ir_step(abs_angle_deg: float) -> float:
    ratio = _clamp(abs_angle_deg / 90.0, 0.0, 1.0)
    return IR_STEP_MIN_M + (IR_STEP_MAX_M - IR_STEP_MIN_M) * ratio


def _scale_ir_speed(abs_angle_deg: float) -> int:
    ratio = _clamp(abs_angle_deg / 90.0, 0.0, 1.0)
    speed = IR_TRACK_MIN_SPEED_PERCENT + (
        IR_TRACK_MAX_SPEED_PERCENT - IR_TRACK_MIN_SPEED_PERCENT
    ) * ratio
    return int(round(speed))


def _line_x_min_m() -> float:
    return DEFENSE_LINE_CENTER_X_M - DEFENSE_LINE_HALF_LENGTH_M


def _line_x_max_m() -> float:
    return DEFENSE_LINE_CENTER_X_M + DEFENSE_LINE_HALF_LENGTH_M


def _normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
