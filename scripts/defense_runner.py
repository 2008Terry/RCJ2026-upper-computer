#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy

from competition_start_gate import wait_for_ready_and_start
from competition_robot import CompetitionRobot
from defense_logic import run_task


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    robot = CompetitionRobot()
    exit_code = 0

    try:
        wait_for_ready_and_start(robot, "Defense")
        run_task(robot)
    except KeyboardInterrupt:
        robot.get_logger().warn("Defense task interrupted by user.")
        exit_code = 130
    except Exception as error:
        robot.get_logger().error(f"Defense task failed: {error}")
        exit_code = 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
