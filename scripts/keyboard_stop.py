from __future__ import annotations

import os
import select
import signal
import sys
import termios
import threading
import tty
from types import TracebackType
from typing import Any, Optional, Type


STOP_KEYS = ("q", "Q", "\x1b")


class KeyboardStop:
    """Non-blocking single-key stop detector for terminal-launched tasks."""

    def __init__(self, logger: Any, task_name: str) -> None:
        self._logger = logger
        self._task_name = task_name
        self._fd: Optional[int] = None
        self._old_settings: Optional[list[Any]] = None

    def __enter__(self) -> "KeyboardStop":
        if sys.stdin is None or not sys.stdin.isatty():
            return self

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._logger.info(f"{self._task_name}: press q or Esc to safe-stop.")
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc: Optional[BaseException],
        tb: Optional[TracebackType],
    ) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def should_stop(self) -> bool:
        if self._fd is None:
            return False

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return False

        key = os.read(self._fd, 1).decode(errors="ignore")
        if key in STOP_KEYS:
            self._logger.warn(f"{self._task_name}: keyboard stop requested.")
            return True
        return False


class KeyboardInterruptStop:
    """Background key listener that interrupts arbitrary task runner code."""

    def __init__(self, logger: Any, task_name: str) -> None:
        self._logger = logger
        self._task_name = task_name
        self._fd: Optional[int] = None
        self._old_settings: Optional[list[Any]] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def __enter__(self) -> "KeyboardInterruptStop":
        if sys.stdin is None or not sys.stdin.isatty():
            return self

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._running = True
        self._thread = threading.Thread(
            target=self._listen,
            name=f"{self._task_name.lower()}_keyboard_interrupt",
            daemon=True,
        )
        self._thread.start()
        self._logger.info(f"{self._task_name}: press q or Esc to interrupt.")
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc: Optional[BaseException],
        tb: Optional[TracebackType],
    ) -> None:
        self._running = False
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def _listen(self) -> None:
        while self._running and self._fd is not None:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            key = os.read(self._fd, 1).decode(errors="ignore")
            if key in STOP_KEYS:
                self._logger.warn(f"{self._task_name}: keyboard interrupt requested.")
                os.kill(os.getpid(), signal.SIGINT)
                return
