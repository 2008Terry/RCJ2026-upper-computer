#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Callable, Optional

try:
    import serial
except ModuleNotFoundError:  # pragma: no cover - depends on runtime environment
    serial = None


class Stm32CommandSender:
    """Send ASCII motion commands with CRC16-CCITT-FALSE to the STM32."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_sec: float = 1.0,
        serial_factory: Optional[Callable[..., object]] = None,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_sec = float(timeout_sec)
        self._serial_factory = serial_factory
        self._serial = None

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def cmd_dis(
        self, x_coord: float, y_coord: float, speed_profile: int = 1
    ) -> bytes:
        speed_profile_int = self._validate_binary_or_profile(
            "speed_profile", speed_profile, allowed=(0, 1, 2)
        )
        command_text = self._format_command(
            "cmd_dis", x_coord, y_coord, speed_profile_int
        )
        packet = self._build_packet(command_text)
        self._write_packet(packet)
        return packet

    def cmd_turn(self, degrees: float) -> bytes:
        command_text = self._format_command("cmd_turn", degrees)
        packet = self._build_packet(command_text)
        self._write_packet(packet)
        return packet

    def cmd_xqcx(self) -> bytes:
        packet = self._build_packet("cmd_xqcx")
        self._write_packet(packet)
        return packet

    def cmd_dct(self, enabled: int | bool) -> bytes:
        enabled_int = self._validate_binary_or_profile(
            "enabled", int(enabled), allowed=(0, 1)
        )
        command_text = self._format_command("cmd_dct", enabled_int)
        packet = self._build_packet(command_text)
        self._write_packet(packet)
        return packet

    def _get_serial(self):
        if self._serial is None:
            serial_factory = self._serial_factory
            if serial_factory is None:
                if serial is None:
                    raise ModuleNotFoundError(
                        "pyserial is required for live STM32 serial communication."
                    )
                serial_factory = serial.Serial

            self._serial = serial_factory(
                self.port,
                baudrate=self.baudrate,
                timeout=self.timeout_sec,
            )
        return self._serial

    def _write_packet(self, packet: bytes) -> None:
        serial_port = self._get_serial()
        serial_port.write(packet)
        serial_port.flush()

    def _format_command(self, name: str, *values: float) -> str:
        return f"{name} {' '.join(self._format_value(value) for value in values)}"

    def _format_value(self, value: float) -> str:
        value_float = float(value)
        if math.isclose(value_float, round(value_float), abs_tol=1e-9):
            return str(int(round(value_float)))
        return f"{value_float:.6f}".rstrip("0").rstrip(".")

    def _validate_binary_or_profile(
        self, name: str, value: int, *, allowed: tuple[int, ...]
    ) -> int:
        value_int = int(value)
        if value_int not in allowed:
            allowed_text = ", ".join(str(item) for item in allowed)
            raise ValueError(f"{name} must be one of: {allowed_text}.")
        return value_int

    def _build_packet(self, command_text: str) -> bytes:
        command_bytes = command_text.encode("ascii")
        crc = self._crc16_ccitt_false(command_bytes)
        return command_bytes + f" *{crc:04X}\r\n".encode("ascii")

    def _crc16_ccitt_false(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc & 0xFFFF
