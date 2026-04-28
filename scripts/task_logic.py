from __future__ import annotations

from collections.abc import Mapping

from task_sequence import TASKS


def run_task(nav) -> None:
    tasks = list(TASKS)
    if not tasks:
        nav.get_logger().warn(
            "TASKS is empty; task_runner finished without sending motion. "
            "Edit scripts/task_sequence.py to add task commands."
        )
        return

    for index, task in enumerate(tasks, start=1):
        if not isinstance(task, Mapping):
            raise TypeError(f"Task {index} must be a mapping/dict.")

        command = str(task.get("command", "")).strip().lower()
        if command == "goto":
            _run_goto(nav, index, len(tasks), task)
            continue

        raise ValueError(f"Unsupported task command at index {index}: {command!r}")


def _run_goto(nav, index: int, total: int, task: Mapping) -> None:
    if "x_m" not in task or "y_m" not in task:
        raise ValueError(
            f"goto task {index} must use x_m/y_m in meters. "
            "AMCL pose is meter-based; x_cm/y_cm are not supported."
        )
    x_m = float(task["x_m"])
    y_m = float(task["y_m"])
    options = {
        name: task[name]
        for name in (
            "goal_tolerance_m",
            "tolerance_m",
            "max_step_m",
            "settle_sec",
            "max_iterations",
            "goto_timeout_sec",
            "pose_wait_timeout_sec",
            "action_server_wait_sec",
        )
        if name in task
    }
    nav.get_logger().info(
        f"Task {index}/{total}: goto({x_m:.3f}, {y_m:.3f}), "
        f"options={options}"
    )
    nav.goto(x_m, y_m, **options)
