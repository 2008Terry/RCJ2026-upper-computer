#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _ensure_import_path() -> None:
    script_dir = Path(__file__).resolve().parent
    source_helper_dir = script_dir.parent / "src"
    if source_helper_dir.exists():
        sys.path.insert(0, str(source_helper_dir))


_ensure_import_path()

from stm32_command_sender import Stm32CommandSender


class _DryRunSerial:
    def __init__(self, *args, **kwargs) -> None:
        self.writes = []

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send or preview STM32 ASCII motion commands."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument(
        "--x-coord",
        type=float,
        default=100.0,
        help="cmd_dis world x in cm; positive is map-up.",
    )
    parser.add_argument(
        "--y-coord",
        type=float,
        default=-100.0,
        help="cmd_dis world y in cm; positive is map-left.",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        default=90.0,
        help="cmd_turn STM32 yaw in degrees; 0 is map-up, 90 is map-left.",
    )
    parser.add_argument(
        "--speed-profile",
        type=int,
        default=1,
        choices=(0, 1, 2),
        help="cmd_dis speed profile: 0 fast, 1 normal, 2 smooth.",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="Send commands to the serial port instead of dry-run preview.",
    )
    args = parser.parse_args()

    serial_factory = None if args.send else _DryRunSerial
    sender = Stm32CommandSender(
        port=args.port,
        baudrate=args.baudrate,
        timeout_sec=args.timeout_sec,
        serial_factory=serial_factory,
    )

    try:
        dis_packet = sender.cmd_dis(args.x_coord, args.y_coord, args.speed_profile)
        turn_packet = sender.cmd_turn(args.degrees)
    finally:
        sender.close()

    print(dis_packet.decode("ascii").rstrip("\r\n"))
    print(turn_packet.decode("ascii").rstrip("\r\n"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
