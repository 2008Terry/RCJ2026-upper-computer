from __future__ import annotations

from typing import Any

import rclpy

from defense_behaviors import (
    LOOP_SLEEP_SEC,
    DefenseRuntime,
    handle_infrared_defense,
    handle_neutral_defense,
    handle_vision_defense,
    prepare_defense,
    safe_stop,
    sense_defense_cue,
)
from keyboard_stop import KeyboardStop


STATE_VISION_TRACK = "VISION_TRACK"
STATE_INFRARED_TRACK = "INFRARED_TRACK"
STATE_RETURN_NEUTRAL = "RETURN_NEUTRAL"
STATE_HOLD_NEUTRAL = "HOLD_NEUTRAL"


def run_task(robot: Any) -> None:
    """Run the locked-head goalkeeper defense state machine."""

    runtime = DefenseRuntime(robot)
    prepare_defense(robot)

    try:
        with KeyboardStop(robot.get_logger(), "Defense") as keyboard_stop:
            while rclpy.ok():
                if keyboard_stop.should_stop():
                    break

                cue = sense_defense_cue(robot, runtime.infrared)

                if cue.ball is not None:
                    runtime.report_state(
                        STATE_VISION_TRACK,
                        f"ball_x={cue.ball.absolute_x_m:.3f}",
                    )
                    handle_vision_defense(robot, runtime.drive, cue.ball)
                elif cue.infrared_angle_deg is not None:
                    runtime.report_state(
                        STATE_INFRARED_TRACK,
                        (
                            f"channel={cue.infrared_channel}, "
                            f"angle={cue.infrared_angle_deg:.1f}"
                        ),
                    )
                    handle_infrared_defense(
                        robot,
                        runtime.drive,
                        cue.infrared_angle_deg,
                    )
                else:
                    reached = handle_neutral_defense(robot, runtime.drive)
                    if reached:
                        runtime.report_state(STATE_HOLD_NEUTRAL)
                    else:
                        runtime.report_state(STATE_RETURN_NEUTRAL)

                robot.timer(duration_sec=LOOP_SLEEP_SEC)
    finally:
        safe_stop(robot, runtime.drive)
