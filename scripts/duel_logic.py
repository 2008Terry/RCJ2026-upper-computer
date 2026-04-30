from __future__ import annotations

from typing import Any

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


def run_task(robot: Any) -> None:
    """Run the duel state machine until ROS shuts down or Ctrl-C."""

    runtime = DuelRuntime(robot)

    robot.motion_enable()
    robot.infrared_modulated_mode()
    robot.suck_off()
    robot.turn(angle_deg=ATTACK_YAW_DEG)

    try:
        while rclpy.ok():
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
