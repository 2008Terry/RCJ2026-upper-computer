#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pyserial is required to run this script. Install python3-serial first."
    ) from exc


# HARDCODED_COMMANDS = [
#     "cmd_dis 100 -100 *7841",
#     "cmd_turn 90 *1935",
#     "cmd_dis 50 0 *798F",
# ]

HARDCODED_COMMANDS = [
     "cmd_dis 100 -100 *7841",
]

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a fixed sequence of hardcoded STM32 move commands."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument(
        "--delay-sec",
        type=float,
        default=0.5,
        help="Delay between each hardcoded command.",
    )
    args = parser.parse_args()

    with serial.Serial(
        args.port,
        baudrate=args.baudrate,
        timeout=args.timeout_sec,
    ) as serial_port:
        for command in HARDCODED_COMMANDS:
            packet = f"{command}\r\n".encode("ascii")
            serial_port.write(packet)
            serial_port.flush()
            print(command)
            time.sleep(args.delay_sec)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
