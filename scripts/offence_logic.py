from __future__ import annotations

from typing import Any, Optional

import rclpy

from keyboard_stop import KeyboardStop
from offence_behaviors import (
    ATTACK_YAW_DEG,
    LOOP_SLEEP_SEC,
    OffenceRuntime,
    describe_defense_action,
    describe_find_ball_action,
    describe_kick_action,
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
FORCE_STATE: Optional[str] = None

# Set this to False when offence should find balls with vision only.
USE_INFRARED_TO_FIND_BALL = False



def run_task(robot: Any) -> None:
    """Run the offence state machine until ROS shuts down or Ctrl-C."""

    runtime = OffenceRuntime(robot)
    use_infrared_to_find_ball = USE_INFRARED_TO_FIND_BALL

    robot.motion_enable()
    # robot.infrared_modulated_mode()
    robot.suck_off()
    robot.turn(angle_deg=ATTACK_YAW_DEG)

    _validate_force_state()
    force_state = FORCE_STATE if FORCE_STATE is not None else "auto"
    robot.get_logger().info(
        f"Offence task started: attack_yaw={ATTACK_YAW_DEG:.1f}deg "
        f"mode={force_state} find_ir={'on' if use_infrared_to_find_ball else 'off'} "
        f"loop={LOOP_SLEEP_SEC:.2f}s"
    )

    try:
        with KeyboardStop(robot.get_logger(), "Offence") as keyboard_stop:
            while rclpy.ok():
                if keyboard_stop.should_stop():
                    break

                if FORCE_STATE is not None:
                    _run_forced_state(
                        robot,
                        runtime,
                        FORCE_STATE,
                        use_infrared=use_infrared_to_find_ball,
                    )
                    robot.timer(duration_sec=LOOP_SLEEP_SEC)
                    continue

                cue = sense_ball(
                    robot,
                    runtime.infrared,
                    use_infrared=use_infrared_to_find_ball,
                )

                if cue is None:
                    runtime.report_state(STATE_DEFENSE)
                    runtime.report_decision(
                        STATE_DEFENSE,
                        describe_defense_action(
                            use_infrared=use_infrared_to_find_ball,
                        ),
                    )
                    runtime.suck.set(robot, 0)
                    handle_defense(robot, runtime.drive)
                elif cue.ball is not None and should_kick(cue.ball):
                    runtime.report_state(STATE_KICK_BALL, "ball close and centered")
                    runtime.report_decision(
                        STATE_KICK_BALL,
                        describe_kick_action(cue.ball),
                    )
                    handle_kick(robot, runtime.drive, runtime.suck)
                else:
                    runtime.report_state(STATE_FIND_BALL, cue.source)
                    runtime.report_decision(
                        STATE_FIND_BALL,
                        describe_find_ball_action(
                            cue,
                            use_infrared=use_infrared_to_find_ball,
                        ),
                    )
                    runtime.suck.set(robot, 0)
                    handle_find_ball(robot, runtime.drive, cue)

                robot.timer(duration_sec=LOOP_SLEEP_SEC)
    finally:
        robot.get_logger().warn("Offence task exiting -> safe_stop")
        safe_stop(robot, runtime.drive, runtime.suck)


def _validate_force_state() -> None:
    if FORCE_STATE is None:
        return
    if FORCE_STATE not in (STATE_DEFENSE, STATE_FIND_BALL, STATE_KICK_BALL):
        raise ValueError(
            "FORCE_STATE must be None, STATE_DEFENSE, STATE_FIND_BALL, "
            "or STATE_KICK_BALL."
        )


def _run_forced_state(
    robot: Any,
    runtime: OffenceRuntime,
    state: str,
    *,
    use_infrared: bool,
) -> None:
    if state == STATE_DEFENSE:
        runtime.report_state(STATE_DEFENSE, "forced")
        runtime.report_decision(
            STATE_DEFENSE,
            f"forced state -> {describe_defense_action(use_infrared=use_infrared)}",
        )
        runtime.suck.set(robot, 0)
        handle_defense(robot, runtime.drive)
        return

    if state == STATE_FIND_BALL:
        cue = sense_ball(
            robot,
            runtime.infrared,
            use_infrared=use_infrared,
        )
        detail = "forced"
        if cue is not None:
            detail = f"forced: {cue.source}"
        runtime.report_state(STATE_FIND_BALL, detail)
        if cue is None:
            cue_text = "vision/infrared" if use_infrared else "vision"
            runtime.report_decision(
                STATE_FIND_BALL,
                f"forced state -> no {cue_text} cue -> stop drive",
            )
        else:
            runtime.report_decision(
                STATE_FIND_BALL,
                "forced state -> "
                f"{describe_find_ball_action(cue, use_infrared=use_infrared)}",
            )
        runtime.suck.set(robot, 0)
        if cue is None:
            runtime.drive.stop(robot)
            return
        handle_find_ball(robot, runtime.drive, cue)
        return

    if state == STATE_KICK_BALL:
        runtime.report_state(STATE_KICK_BALL, "forced")
        runtime.report_decision(
            STATE_KICK_BALL,
            f"forced state -> {describe_kick_action()}",
        )
        handle_kick(robot, runtime.drive, runtime.suck)
        return

    raise AssertionError(f"Unexpected forced state: {state}")
