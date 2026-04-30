from __future__ import annotations

from typing import Any, Optional

import rclpy

from duel_behaviors import (
    ATTACK_YAW_DEG,
    LOOP_SLEEP_SEC,
    DuelRuntime,
    handle_defense,
    handle_find_ball,
    handle_kick,
    safe_stop,
    sense_ball,
    should_kick,
)


STATE_DEFENSE = "DEFENSE"
STATE_FIND_BALL = "FIND_BALL"
STATE_KICK_BALL = "KICK_BALL"

# Set this to one state name when debugging a single behavior:
# FORCE_STATE = STATE_DEFENSE
# FORCE_STATE = STATE_FIND_BALL
# FORCE_STATE = STATE_KICK_BALL
FORCE_STATE: Optional[str] = STATE_DEFENSE


def run_task(robot: Any) -> None:
    """Run the duel state machine until ROS shuts down or Ctrl-C."""

    runtime = DuelRuntime(robot)

    robot.motion_enable()
    robot.infrared_modulated_mode()
    robot.suck_off()
    robot.turn(angle_deg=ATTACK_YAW_DEG)

    _validate_force_state()

    try:
        while rclpy.ok():
            if FORCE_STATE is not None:
                _run_forced_state(robot, runtime, FORCE_STATE)
                robot.timer(duration_sec=LOOP_SLEEP_SEC)
                continue

            cue = sense_ball(robot)

            if cue is None:
                runtime.report_state(STATE_DEFENSE)
                runtime.suck.set(robot, 0)
                handle_defense(robot, runtime.drive)
            elif cue.ball is not None and should_kick(cue.ball):
                runtime.report_state(STATE_KICK_BALL, "ball close and centered")
                handle_kick(robot, runtime.drive, runtime.suck)
            else:
                runtime.report_state(STATE_FIND_BALL, cue.source)
                runtime.suck.set(robot, 0)
                handle_find_ball(robot, runtime.drive, cue)

            robot.timer(duration_sec=LOOP_SLEEP_SEC)
    finally:
        safe_stop(robot, runtime.drive, runtime.suck)


def _validate_force_state() -> None:
    if FORCE_STATE is None:
        return
    if FORCE_STATE not in (STATE_DEFENSE, STATE_FIND_BALL, STATE_KICK_BALL):
        raise ValueError(
            "FORCE_STATE must be None, STATE_DEFENSE, STATE_FIND_BALL, "
            "or STATE_KICK_BALL."
        )


def _run_forced_state(robot: Any, runtime: DuelRuntime, state: str) -> None:
    if state == STATE_DEFENSE:
        runtime.report_state(STATE_DEFENSE, "forced")
        runtime.suck.set(robot, 0)
        handle_defense(robot, runtime.drive)
        return

    if state == STATE_FIND_BALL:
        cue = sense_ball(robot)
        detail = "forced"
        if cue is not None:
            detail = f"forced: {cue.source}"
        runtime.report_state(STATE_FIND_BALL, detail)
        runtime.suck.set(robot, 0)
        if cue is None:
            runtime.drive.stop(robot)
            return
        handle_find_ball(robot, runtime.drive, cue)
        return

    if state == STATE_KICK_BALL:
        runtime.report_state(STATE_KICK_BALL, "forced")
        handle_kick(robot, runtime.drive, runtime.suck)
        return

    raise AssertionError(f"Unexpected forced state: {state}")
