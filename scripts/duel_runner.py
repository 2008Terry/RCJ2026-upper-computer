#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy

from competition_robot import CompetitionRobot
from duel_logic import run_task


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    robot = CompetitionRobot()
    exit_code = 0

    try:
        run_task(robot)
    except KeyboardInterrupt:
        robot.get_logger().warn("Duel task interrupted by user.")
        exit_code = 130
    except Exception as error:
        robot.get_logger().error(f"Duel task failed: {error}")
        exit_code = 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
