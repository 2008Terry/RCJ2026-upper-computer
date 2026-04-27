from __future__ import annotations

from typing import Iterable, Tuple


TaskPoint = Tuple[float, float]


TASK_POINTS: Iterable[TaskPoint] = (
    # Fill in absolute map-frame targets, in meters:
    # (1.20, 0.80),
    # (0.60, 1.40),
)


def run_task(nav) -> None:
    points = list(TASK_POINTS)
    if not points:
        nav.get_logger().warn(
            "TASK_POINTS is empty; task_runner finished without sending motion. "
            "Edit scripts/task_logic.py to add map-frame targets."
        )
        return

    for index, (x_m, y_m) in enumerate(points, start=1):
        nav.get_logger().info(
            "Task point %d/%d: goto(%.3f, %.3f)",
            index,
            len(points),
            x_m,
            y_m,
        )
        nav.goto(x_m, y_m)
