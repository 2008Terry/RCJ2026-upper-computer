#!/usr/bin/env python3

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test a saved robot mask against an image."
    )
    parser.add_argument("--image", required=True, help="Input image path.")
    parser.add_argument("--mask", required=True, help="Input mask path.")
    parser.add_argument(
        "--window-name",
        default="Robot Mask Test",
        help="OpenCV window title prefix.",
    )
    parser.add_argument(
        "--save-overlay",
        help="Optional path to save the red overlay preview image.",
    )
    return parser.parse_args()


def ensure_display_available():
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return
    raise RuntimeError(
        "test_robot_mask.py requires DISPLAY or WAYLAND_DISPLAY for the OpenCV windows."
    )


def load_image(path):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to load image: {path}")
    return image


def load_mask(path):
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError(f"Failed to load mask: {path}")
    return mask


def build_overlay(image, mask):
    overlay = image.copy()
    overlay[mask == 0] = (0, 0, 255)
    blended = cv2.addWeighted(image, 0.7, overlay, 0.3, 0.0)
    outline_mask = np.zeros_like(mask)
    robot_region = np.where(mask == 0, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(
        robot_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(outline_mask, contours, -1, 255, 2)
    blended[outline_mask > 0] = (0, 255, 255)
    return blended


def print_stats(image_path, mask_path, image, mask):
    zero_pixels = int(np.count_nonzero(mask == 0))
    nonzero_pixels = int(np.count_nonzero(mask != 0))
    unique_values = np.unique(mask)

    print(f"Image path: {image_path}")
    print(f"Mask path : {mask_path}")
    print(f"Image size: {image.shape[1]}x{image.shape[0]}")
    print(f"Mask size : {mask.shape[1]}x{mask.shape[0]}")
    print(f"Mask dtype: {mask.dtype}")
    print(f"Mask min/max: {int(mask.min())} / {int(mask.max())}")
    print(f"Mask unique values: {unique_values.tolist()}")
    print(f"Masked pixels (robot region): {zero_pixels}")
    print(f"Unmasked pixels: {nonzero_pixels}")


def validate_mask(image, mask):
    if image.shape[:2] != mask.shape[:2]:
        raise RuntimeError(
            "Image and mask dimensions do not match: "
            f"image={image.shape[1]}x{image.shape[0]}, "
            f"mask={mask.shape[1]}x{mask.shape[0]}"
        )

    unique_values = set(int(value) for value in np.unique(mask))
    unexpected_values = sorted(value for value in unique_values if value not in (0, 255))
    if unexpected_values:
        print(
            "Warning: mask contains values other than 0 and 255: "
            f"{unexpected_values}"
        )

    if np.count_nonzero(mask == 0) == 0:
        print("Warning: mask does not contain any 0-valued robot region.")

    if np.count_nonzero(mask == 255) == 0:
        print("Warning: mask does not contain any 255-valued background.")


def maybe_save_overlay(path, overlay):
    if not path:
        return
    output_path = Path(path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), overlay):
        raise RuntimeError(f"Failed to write overlay image: {output_path}")
    print(f"Overlay saved as {output_path}")


def main():
    args = parse_args()
    image_path = Path(args.image).expanduser().resolve()
    mask_path = Path(args.mask).expanduser().resolve()

    ensure_display_available()
    image = load_image(image_path)
    mask = load_mask(mask_path)
    validate_mask(image, mask)
    print_stats(image_path, mask_path, image, mask)

    overlay = build_overlay(image, mask)
    maybe_save_overlay(args.save_overlay, overlay)

    print("Press any key in an OpenCV window to close.")
    cv2.imshow(f"{args.window_name} - Image", image)
    cv2.imshow(f"{args.window_name} - Mask", mask)
    cv2.imshow(f"{args.window_name} - Overlay", overlay)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
