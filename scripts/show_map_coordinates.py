#!/usr/bin/env python3
"""Show rcj_map.png and print clicked coordinates."""

from __future__ import annotations

import argparse
import ast
import os
from pathlib import Path

import cv2


WINDOW_NAME = "RCJ Map Coordinates"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Display maps/rcj_map.png. Hover to see coordinates; left click prints "
            "the current coordinate to the terminal."
        )
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="Map image path. Defaults to maps/rcj_map.png from this package.",
    )
    parser.add_argument(
        "--yaml",
        type=Path,
        help="Map YAML path. Defaults to rcj_map.yaml next to the image.",
    )
    parser.add_argument(
        "--window-name",
        default=WINDOW_NAME,
        help="OpenCV window name.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Display scale. Coordinates are still reported in original image pixels.",
    )
    return parser.parse_args()


def package_share_dir() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None

    try:
        return Path(get_package_share_directory("rcj_localization"))
    except Exception:
        return None


def default_image_path() -> Path:
    candidates = [
        Path.cwd() / "maps" / "rcj_map.png",
        Path(__file__).resolve().parents[1] / "maps" / "rcj_map.png",
    ]

    share_dir = package_share_dir()
    if share_dir is not None:
        candidates.append(share_dir / "maps" / "rcj_map.png")

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return candidates[0]


def resolve_path(path: Path | None, default: Path) -> Path:
    if path is None:
        return default.resolve()
    return path.expanduser().resolve()


def load_map_metadata(yaml_path: Path) -> tuple[float, tuple[float, float, float]] | None:
    if not yaml_path.exists():
        return None

    resolution = None
    origin = None
    for raw_line in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue

        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()

        if key == "resolution":
            resolution = float(value)
        elif key == "origin":
            parsed_origin = ast.literal_eval(value)
            if len(parsed_origin) >= 3:
                origin = (
                    float(parsed_origin[0]),
                    float(parsed_origin[1]),
                    float(parsed_origin[2]),
                )

    if resolution is None or origin is None:
        return None
    return resolution, origin


def image_to_map(
    x_px: int,
    y_px: int,
    image_height: int,
    resolution: float,
    origin: tuple[float, float, float],
) -> tuple[float, float]:
    map_x = origin[0] + (x_px + 0.5) * resolution
    map_y = origin[1] + (image_height - y_px - 0.5) * resolution
    return map_x, map_y


def format_coordinate(
    x_px: int,
    y_px: int,
    image_height: int,
    metadata: tuple[float, tuple[float, float, float]] | None,
) -> str:
    text = f"pixel=({x_px}, {y_px})"
    if metadata is None:
        return text

    resolution, origin = metadata
    map_x, map_y = image_to_map(x_px, y_px, image_height, resolution, origin)
    return f"{text} map=({map_x:.4f}, {map_y:.4f}) m"


class CoordinateViewer:
    def __init__(
        self,
        image,
        window_name: str,
        metadata: tuple[float, tuple[float, float, float]] | None,
        scale: float,
    ) -> None:
        self.image = image
        self.window_name = window_name
        self.metadata = metadata
        self.scale = scale
        self.mouse_x = 0
        self.mouse_y = 0
        self.has_mouse = False

    def display_to_image_coordinate(self, x: int, y: int) -> tuple[int, int]:
        image_x = int(round(x / self.scale))
        image_y = int(round(y / self.scale))
        height, width = self.image.shape[:2]
        image_x = min(max(image_x, 0), width - 1)
        image_y = min(max(image_y, 0), height - 1)
        return image_x, image_y

    def on_mouse(self, event, x, y, _flags, _param) -> None:
        image_x, image_y = self.display_to_image_coordinate(x, y)
        self.mouse_x = image_x
        self.mouse_y = image_y
        self.has_mouse = True

        if event == cv2.EVENT_LBUTTONDOWN:
            print(
                format_coordinate(
                    image_x,
                    image_y,
                    self.image.shape[0],
                    self.metadata,
                ),
                flush=True,
            )

    def frame(self):
        canvas = self.image.copy()
        if self.has_mouse:
            coordinate = format_coordinate(
                self.mouse_x,
                self.mouse_y,
                self.image.shape[0],
                self.metadata,
            )
            cv2.drawMarker(
                canvas,
                (self.mouse_x, self.mouse_y),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                18,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                coordinate,
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                coordinate,
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        if self.scale == 1.0:
            return canvas

        return cv2.resize(
            canvas,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_NEAREST,
        )


def ensure_display_available() -> None:
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return
    raise RuntimeError(
        "show_map_coordinates.py requires DISPLAY or WAYLAND_DISPLAY for the OpenCV window."
    )


def main() -> int:
    args = parse_args()
    if args.scale <= 0.0:
        raise ValueError("--scale must be greater than 0.")

    image_path = resolve_path(args.image, default_image_path())
    yaml_path = resolve_path(args.yaml, image_path.with_suffix(".yaml"))

    ensure_display_available()

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to load image: {image_path}")

    metadata = load_map_metadata(yaml_path)
    viewer = CoordinateViewer(image, args.window_name, metadata, args.scale)

    cv2.namedWindow(args.window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(args.window_name, viewer.on_mouse)

    print(f"Loaded image: {image_path}")
    if metadata is None:
        print(f"Map YAML not loaded: {yaml_path}")
        print("Click output will include pixel coordinates only.")
    else:
        resolution, origin = metadata
        print(f"Loaded map YAML: {yaml_path}")
        print(f"resolution={resolution:g} m/pixel origin={origin}")
        print("Click output includes original image pixels and map-frame meters.")
    print("Controls: left click prints coordinates, q/Esc exits.")

    try:
        while True:
            cv2.imshow(args.window_name, viewer.frame())
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
