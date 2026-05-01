#!/usr/bin/env python3
"""Smoke test for the repo-local YOLO virtual environment."""

from __future__ import annotations

import importlib
import sys


def import_version(module_name: str, attr: str = "__version__") -> str:
    module = importlib.import_module(module_name)
    return str(getattr(module, attr, "present"))


def main() -> int:
    print(f"python: {sys.version.split()[0]} ({sys.executable})")

    import torch
    import torchvision
    import cv2
    import rclpy  # noqa: F401
    from ultralytics import YOLO  # noqa: F401

    print(f"torch: {torch.__version__}")
    print(f"torchvision: {torchvision.__version__}")
    print(f"ultralytics: {import_version('ultralytics')}")
    print(f"opencv: {cv2.__version__} ({getattr(cv2, '__file__', 'unknown')})")
    print("rclpy: present")
    print(f"cuda_available: {torch.cuda.is_available()}")

    try:
        import cv_bridge  # noqa: F401
    except ImportError as exc:
        print(f"cv_bridge: missing ({exc})")
        return 1

    print("cv_bridge: present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
