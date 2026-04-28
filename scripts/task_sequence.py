from __future__ import annotations


# Edit only this list for normal task programming.
# Coordinates are absolute map-frame meters.
TASKS = [
    # {"command": "goto", "x_m": -0.40, "y_m": -0.60},
    {
        "command": "goto",
        "x_m": -0.40,
        "y_m": -0.60,
        "goal_tolerance_m": 0.02,
        "max_step_m": 0.8,
        "settle_sec": 0.7,
        "max_iterations": 25,
        "goto_timeout_sec": 40.0,
        "pose_wait_timeout_sec": 5.0,
        "action_server_wait_sec": 2.0,
    },
    {
        "command": "goto",
        "x_m": 0.40,
        "y_m": -0.60,
        "goal_tolerance_m": 0.02,
        "max_step_m": 0.8,
        "settle_sec": 0.7,
        "max_iterations": 25,
        "goto_timeout_sec": 40.0,
        "pose_wait_timeout_sec": 5.0,
        "action_server_wait_sec": 2.0,
    },
    {
        "command": "goto",
        "x_m": -0.40,
        "y_m": 0.60,
        "goal_tolerance_m": 0.02,
        "max_step_m": 0.8,
        "settle_sec": 0.7,
        "max_iterations": 25,
        "goto_timeout_sec": 40.0,
        "pose_wait_timeout_sec": 5.0,
        "action_server_wait_sec": 2.0,
    },
    {
        "command": "goto",
        "x_m": 0.40,
        "y_m": 0.60,
        "goal_tolerance_m": 0.02,
        "max_step_m": 0.8,
        "settle_sec": 0.7,
        "max_iterations": 25,
        "goto_timeout_sec": 40.0,
        "pose_wait_timeout_sec": 5.0,
        "action_server_wait_sec": 2.0,
    },
]
