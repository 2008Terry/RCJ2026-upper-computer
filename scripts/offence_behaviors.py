from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional

from infrared_smoothing import InfraredAngleFilter


# Field assumptions for the default competition setup:
# - own half is map -y
# - attack direction is map +y, STM32 yaw 0 deg
ATTACK_YAW_DEG = 0.0
DEFENSE_POINT_X_M = 0.0
DEFENSE_POINT_Y_M = -0.40

FIELD_X_MIN_M = -0.72
FIELD_X_MAX_M = 0.72
FIELD_Y_MIN_M = -1.00
FIELD_Y_MAX_M = 1.00

# Use the latest published vision state instead of waiting for a new frame each
# loop. With timeout 0, robot.find_ball() keeps using the last positive vision
# detection until the detector publishes an explicit negative detection.
VISION_TIMEOUT_SEC = 0.0
MIN_BALL_CONFIDENCE = 0.3
POSE_TIMEOUT_SEC = 0.2
LOOP_SLEEP_SEC = 0.04
LOG_STATUS_PERIOD_SEC = 0.5
INFRARED_WARN_PERIOD_SEC = 2.0
DRIVE_RESEND_SEC = 0.16
ANGLE_RESEND_DELTA_DEG = 6.0

DEFENSE_TOLERANCE_M = 0.08
DEFENSE_SPEED_PERCENT = 18
NO_VISION_REVERSE_SEC = 0.45
NO_VISION_REVERSE_SPEED_PERCENT = 18
CHASE_SPEED_PERCENT = 24
IR_CHASE_SPEED_PERCENT = 18
KICK_DRIVE_SPEED_PERCENT = 20

CHASE_BEHIND_OFFSET_M = 0.24
CHASE_BEHIND_TOLERANCE_M = 0.08
KICK_DISTANCE_M = 0.30
KICK_ANGLE_DEG = 30.0
KICK_SUCK_SPEED_PERCENT = 15

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
IR_HISTORY_SIZE = 8
IR_EWMA_ALPHA = 0.35
IR_CLEAR_AFTER_MISSES = 3
_last_infrared_warn_time = 0.0


@dataclass(frozen=True)
class BallCue:
    ball: Optional[Any]
    infrared_channel: Optional[int]
    infrared_angle_deg: Optional[float]
    infrared_behind: bool = False

    @property
    def source(self) -> str:
        if self.ball is not None:
            return "vision"
        if self.infrared_behind:
            return "infrared_behind"
        return "infrared"


class OffenceRuntime:
    def __init__(self, robot: Any) -> None:
        self.drive = DriveCache()
        self.suck = SuckCache()
        self.infrared = InfraredAngleFilter(
            channel_to_angle_deg=IR_CHANNEL_TO_MOVE_ANGLE_DEG,
            history_size=IR_HISTORY_SIZE,
            ewma_alpha=IR_EWMA_ALPHA,
            clear_after_misses=IR_CLEAR_AFTER_MISSES,
        )
        self.no_vision_return = NoVisionReturn()
        self._robot = robot
        self._state: Optional[str] = None
        self._last_status_time = 0.0

    def report_state(self, state: str, detail: str = "") -> None:
        if state == self._state:
            return
        self._state = state
        suffix = f": {detail}" if detail else ""
        self._robot.get_logger().info(f"Offence state -> {state}{suffix}")

    def report_decision(self, state: str, detail: str) -> None:
        now = time.monotonic()
        if now - self._last_status_time < LOG_STATUS_PERIOD_SEC:
            return
        self._last_status_time = now
        self._robot.get_logger().info(f"Offence decision [{state}]: {detail}")


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


class NoVisionReturn:
    def __init__(self) -> None:
        self._reverse_until: Optional[float] = None

    def reset(self) -> None:
        self._reverse_until = None

    def should_reverse(self) -> bool:
        if NO_VISION_REVERSE_SEC <= 0.0:
            return False

        now = time.monotonic()
        if self._reverse_until is None:
            self._reverse_until = now + NO_VISION_REVERSE_SEC
        return now < self._reverse_until


def sense_ball(
    robot: Any,
    infrared_filter: InfraredAngleFilter,
    *,
    use_infrared: bool = True,
) -> Optional[BallCue]:
    infrared_cue = None
    if use_infrared:
        infrared_cue = _sense_infrared(robot, infrared_filter)
    else:
        infrared_filter.clear()

    ball = robot.find_ball(
        timeout_sec=VISION_TIMEOUT_SEC,
        min_confidence=MIN_BALL_CONFIDENCE,
    )
    if ball is not None:
        return BallCue(
            ball=ball,
            infrared_channel=None,
            infrared_angle_deg=None,
            infrared_behind=False,
        )

    return infrared_cue


def _sense_infrared(
    robot: Any,
    infrared_filter: InfraredAngleFilter,
) -> Optional[BallCue]:
    channel = _read_infrared_channel(robot)
    if channel == -1:
        infrared_filter.clear()
        return BallCue(
            ball=None,
            infrared_channel=channel,
            infrared_angle_deg=None,
            infrared_behind=True,
        )
    if channel is not None:
        angle = infrared_filter.update(channel)
        return BallCue(
            ball=None,
            infrared_channel=channel,
            infrared_angle_deg=angle,
            infrared_behind=False,
        )
    infrared_filter.mark_missed()
    return None


def handle_defense(robot: Any, drive_cache: DriveCache) -> None:
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


def handle_no_vision_defense(
    robot: Any,
    drive_cache: DriveCache,
    no_vision_return: NoVisionReturn,
) -> None:
    if no_vision_return.should_reverse():
        drive_cache.drive(
            robot,
            speed_percent=NO_VISION_REVERSE_SPEED_PERCENT,
            move_angle_deg=180.0,
        )
        return

    handle_defense(robot, drive_cache)


def handle_find_ball(robot: Any, drive_cache: DriveCache, cue: BallCue) -> None:
    if cue.ball is None:
        if cue.infrared_behind:
            drive_cache.drive(
                robot,
                speed_percent=IR_CHASE_SPEED_PERCENT,
                move_angle_deg=180.0,
            )
            return
        if cue.infrared_angle_deg is None:
            drive_cache.stop(robot)
            return
        drive_cache.drive(
            robot,
            speed_percent=IR_CHASE_SPEED_PERCENT,
            move_angle_deg=cue.infrared_angle_deg,
        )
        return

    ball = cue.ball
    if ball.base_x_m < 0.0:
        drive_cache.drive(
            robot,
            speed_percent=IR_CHASE_SPEED_PERCENT,
            move_angle_deg=ball.angle_deg,
        )
        return

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


def handle_kick(
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


def should_kick(ball: Any) -> bool:
    return (
        0.0 < ball.base_x_m <= KICK_DISTANCE_M
        and abs(ball.angle_deg) <= KICK_ANGLE_DEG
    )


def describe_defense_action(*, use_infrared: bool = True) -> str:
    cue_text = "vision/infrared" if use_infrared else "vision"
    return (
        f"no {cue_text} cue -> reverse then return to defense point "
        f"reverse={NO_VISION_REVERSE_SEC:.2f}s "
        f"reverse_speed={NO_VISION_REVERSE_SPEED_PERCENT}% "
        f"target=({DEFENSE_POINT_X_M:.2f}, {DEFENSE_POINT_Y_M:.2f}) "
        f"speed={DEFENSE_SPEED_PERCENT}% tol={DEFENSE_TOLERANCE_M:.2f}m"
    )


def describe_find_ball_action(
    cue: BallCue,
    *,
    use_infrared: bool = True,
) -> str:
    if cue.ball is None:
        if cue.infrared_behind:
            return (
                "infrared ch=-1 behind -> reverse search "
                f"angle=180.0deg speed={IR_CHASE_SPEED_PERCENT}%"
            )
        if cue.infrared_angle_deg is None:
            cue_text = "vision/infrared" if use_infrared else "vision"
            return f"forced find-ball but no {cue_text} cue -> stop drive"
        return (
            f"infrared ch={cue.infrared_channel} "
            f"angle={cue.infrared_angle_deg:.1f}deg -> chase by IR "
            f"speed={IR_CHASE_SPEED_PERCENT}%"
        )

    ball = cue.ball
    if ball.base_x_m < 0.0:
        return (
            f"vision {_format_ball(ball)} -> ball behind robot, rear-side chase "
            f"angle={ball.angle_deg:.1f}deg speed={IR_CHASE_SPEED_PERCENT}%"
        )

    target_x_m = _clamp(ball.absolute_x_m, FIELD_X_MIN_M, FIELD_X_MAX_M)
    target_y_m = _clamp(
        ball.absolute_y_m - CHASE_BEHIND_OFFSET_M,
        FIELD_Y_MIN_M,
        FIELD_Y_MAX_M,
    )
    return (
        f"vision {_format_ball(ball)} -> chase behind ball "
        f"target=({target_x_m:.2f}, {target_y_m:.2f}) "
        f"speed={CHASE_SPEED_PERCENT}% tol={CHASE_BEHIND_TOLERANCE_M:.2f}m"
    )


def describe_kick_action(ball: Optional[Any] = None) -> str:
    ball_detail = ""
    if ball is not None:
        ball_detail = f"{_format_ball(ball)} -> "
    return (
        f"{ball_detail}kick window met "
        f"(base_x<= {KICK_DISTANCE_M:.2f}m, abs(angle)<= {KICK_ANGLE_DEG:.1f}deg) "
        f"-> suck={KICK_SUCK_SPEED_PERCENT}% drive={KICK_DRIVE_SPEED_PERCENT}%"
    )


def safe_stop(robot: Any, drive_cache: DriveCache, suck_cache: SuckCache) -> None:
    for action in (
        lambda: drive_cache.stop(robot),
        lambda: suck_cache.set(robot, 0),
    ):
        try:
            action()
        except Exception as error:
            robot.get_logger().warn(f"Shutdown cleanup skipped: {error}")


def _read_infrared_channel(robot: Any) -> Optional[int]:
    try:
        channel = robot.infrared_channel()
    except Exception as error:
        _warn_infrared_read_skipped(robot, error)
        return None

    if channel not in IR_CHANNEL_TO_MOVE_ANGLE_DEG:
        if channel == -1:
            return channel
        return None
    return channel


def _warn_infrared_read_skipped(robot: Any, error: Exception) -> None:
    global _last_infrared_warn_time

    now = time.monotonic()
    if now - _last_infrared_warn_time < INFRARED_WARN_PERIOD_SEC:
        return
    _last_infrared_warn_time = now
    robot.get_logger().warn(f"Infrared read skipped: {error}")


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


def _normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _format_ball(ball: Any) -> str:
    return (
        "ball "
        f"abs=({ball.absolute_x_m:.2f}, {ball.absolute_y_m:.2f}) "
        f"base_x={ball.base_x_m:.2f}m angle={ball.angle_deg:.1f}deg "
        f"conf={ball.confidence:.2f}"
    )
