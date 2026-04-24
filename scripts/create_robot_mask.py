#!/usr/bin/env python3

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np


class RobotMaskCreator:
    def __init__(self, image, window_name):
        self.original_image = image
        self.window_name = window_name
        self.points = []
        self.display_image = image.copy()

    def on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((int(x), int(y)))
            self.redraw()
        elif event == cv2.EVENT_RBUTTONDOWN and self.points:
            self.points.pop()
            self.redraw()

    def redraw(self):
        canvas = self.original_image.copy()

        if len(self.points) >= 3:
            overlay = canvas.copy()
            polygon = np.array(self.points, dtype=np.int32)
            cv2.fillPoly(overlay, [polygon], (0, 0, 255))
            canvas = cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0.0)

        if len(self.points) >= 2:
            cv2.polylines(
                canvas,
                [np.array(self.points, dtype=np.int32)],
                len(self.points) >= 3,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

        for index, point in enumerate(self.points, start=1):
            cv2.circle(canvas, point, 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                canvas,
                str(index),
                (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        instructions = [
            "Left click: add point",
            "Right click: undo last point",
            "c: clear all points",
            "Enter: save mask and points",
            "Esc: exit without saving",
        ]
        for index, text in enumerate(instructions):
            cv2.putText(
                canvas,
                text,
                (16, 30 + index * 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            canvas,
            f"Points: {len(self.points)}",
            (16, 30 + len(instructions) * 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        self.display_image = canvas

    def show(self):
        cv2.imshow(self.window_name, self.display_image)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a robot mask by clicking polygon points on a still image."
    )
    parser.add_argument("--image", required=True, help="Input image path.")
    parser.add_argument(
        "--output-mask",
        help="Output mask path. Defaults to mask.png next to the input image.",
    )
    parser.add_argument(
        "--output-points",
        help=(
            "Output polygon points JSON path. Defaults to "
            "robot_polygon_points.json next to the input image."
        ),
    )
    parser.add_argument(
        "--window-name",
        default="Create Robot Mask",
        help="OpenCV window name.",
    )
    return parser.parse_args()


def resolve_output_path(value, image_path, default_name):
    if value:
        return Path(value).expanduser().resolve()
    return (image_path.parent / default_name).resolve()


def load_image(image_path):
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to load image: {image_path}")
    return image


def build_mask(image_shape, points):
    mask = np.full(image_shape[:2], 255, dtype=np.uint8)
    polygon = np.array(points, dtype=np.int32)
    cv2.fillPoly(mask, [polygon], 0)
    return mask


def save_outputs(mask_path, points_path, image_shape, points):
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    points_path.parent.mkdir(parents=True, exist_ok=True)

    mask = build_mask(image_shape, points)
    if not cv2.imwrite(str(mask_path), mask):
        raise RuntimeError(f"Failed to write mask image: {mask_path}")

    points_data = [[int(x), int(y)] for x, y in points]
    with points_path.open("w", encoding="utf-8") as handle:
        json.dump(points_data, handle, indent=2)
        handle.write("\n")

    return points_data


def ensure_display_available():
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return
    raise RuntimeError(
        "create_robot_mask.py requires DISPLAY or WAYLAND_DISPLAY for the OpenCV window."
    )


def main():
    args = parse_args()
    image_path = Path(args.image).expanduser().resolve()
    mask_path = resolve_output_path(args.output_mask, image_path, "mask.png")
    points_path = resolve_output_path(
        args.output_points, image_path, "robot_polygon_points.json"
    )

    ensure_display_available()
    image = load_image(image_path)

    creator = RobotMaskCreator(image, args.window_name)
    creator.redraw()

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(args.window_name, creator.on_mouse)

    print(f"Loaded image: {image_path}")
    print(f"Mask output: {mask_path}")
    print(f"Points output: {points_path}")
    print("Controls:")
    print("  Left click  - add point")
    print("  Right click - undo last point")
    print("  c           - clear all points")
    print("  Enter       - save outputs")
    print("  Esc         - exit without saving")

    try:
        while True:
            creator.show()
            key = cv2.waitKey(20) & 0xFF

            if key == 27:
                print("Exited without saving.")
                break

            if key in (10, 13):
                if len(creator.points) < 3:
                    print("Need at least 3 points before saving.")
                    continue

                points_data = save_outputs(
                    mask_path,
                    points_path,
                    creator.original_image.shape,
                    creator.points,
                )
                print(f"Mask saved as {mask_path}")
                print(f"Points saved as {points_path}")
                print(f"robot_polygon_points = {points_data}")
                return 0

            if key in (ord("c"), ord("C")):
                creator.points.clear()
                creator.redraw()
    finally:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
