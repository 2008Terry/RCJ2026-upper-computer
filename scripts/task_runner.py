#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy

from goto_point import GotoNavigator
from task_logic import run_task


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    nav = GotoNavigator()
    exit_code = 0

    try:
        run_task(nav)
    except KeyboardInterrupt:
        nav.get_logger().warn("Task interrupted by user.")
        exit_code = 130
    except Exception as error:
        nav.get_logger().error(f"Task failed: {error}")
        exit_code = 1
    finally:
        nav.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
