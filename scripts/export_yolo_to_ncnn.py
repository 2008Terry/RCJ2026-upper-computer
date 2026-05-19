#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export an existing Ultralytics YOLO .pt model to NCNN format."
    )
    parser.add_argument(
        "--model",
        required=True,
        help="Path to the input YOLO .pt model.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="Export image size passed to Ultralytics export(). Default: 320.",
    )
    return parser.parse_args()


def resolve_model_path(raw_path: str) -> Path:
    model_path = Path(raw_path).expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"YOLO model path does not exist: {model_path}")
    if not model_path.is_file():
        raise ValueError(f"YOLO model path is not a file: {model_path}")
    if model_path.suffix.lower() != ".pt":
        raise ValueError(
            f"YOLO model path must point to a .pt file, got: {model_path}"
        )
    return model_path


def normalize_export_path(export_result: object, model_path: Path) -> Path:
    if isinstance(export_result, Path):
        return export_result.resolve()
    if isinstance(export_result, str) and export_result.strip():
        return Path(export_result).expanduser().resolve()
    return model_path.with_name(f"{model_path.stem}_ncnn_model").resolve()


def main() -> int:
    args = parse_args()

    try:
        model_path = resolve_model_path(args.model)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print(
            "Error: failed to import ultralytics. "
            "Source install/setup.bash or install the YOLO environment first. "
            f"Details: {exc}",
            file=sys.stderr,
        )
        return 1

    try:
        model = YOLO(str(model_path))
        export_result = model.export(format="ncnn", imgsz=args.imgsz)
        export_path = normalize_export_path(export_result, model_path)
    except Exception as exc:  # noqa: BLE001
        print(f"Error: NCNN export failed for {model_path}: {exc}", file=sys.stderr)
        return 1

    print(str(export_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
