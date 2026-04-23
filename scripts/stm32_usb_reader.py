#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial import SerialException
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pyserial is required to run this script. Install python3-serial first."
    ) from exc


def format_raw_bytes(data: bytes) -> str:
    parts: list[str] = []
    for byte in data:
        if byte == 0x0D:
            parts.append("\\r")
        elif byte == 0x0A:
            parts.append("\\n")
        elif byte == 0x09:
            parts.append("\\t")
        elif 0x20 <= byte <= 0x7E:
            parts.append(chr(byte))
        else:
            parts.append(f"\\x{byte:02X}")
    return "".join(parts)


def format_hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously read and print any bytes received from a USB serial port."
    )
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-sec", type=float, default=0.1)
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument(
        "--show-hex",
        action="store_true",
        help="Also print the received bytes as hexadecimal.",
    )
    args = parser.parse_args()

    if args.chunk_size <= 0:
        raise SystemExit("--chunk-size must be greater than 0.")

    try:
        with serial.Serial(
            args.port,
            baudrate=args.baudrate,
            timeout=args.timeout_sec,
        ) as serial_port:
            print(
                f"Listening on {args.port} at {args.baudrate} baud. Press Ctrl+C to stop."
            )
            sys.stdout.flush()

            while True:
                bytes_to_read = max(1, min(args.chunk_size, serial_port.in_waiting or 1))
                data = serial_port.read(bytes_to_read)
                if not data:
                    continue

                timestamp = time.strftime("%H:%M:%S")
                escaped = format_raw_bytes(data)
                if args.show_hex:
                    print(f"[{timestamp}] {escaped}    hex={format_hex_bytes(data)}")
                else:
                    print(f"[{timestamp}] {escaped}")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except SerialException as exc:
        raise SystemExit(f"Failed to open or read serial port '{args.port}': {exc}") from exc


if __name__ == "__main__":
    raise SystemExit(main())
