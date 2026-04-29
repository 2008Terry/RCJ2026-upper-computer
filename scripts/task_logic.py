from __future__ import annotations


def run_task(robot) -> None:
    """Edit this function to write the competition task sequence."""

    robot.reset_yaw()
    robot.motion_enable()

    # Example task sequence. Coordinates are absolute map-frame meters.
    robot.suck_on(100)
    robot.goto(-0.40, -0.60)
    robot.turn(90)
    robot.suck_off()
