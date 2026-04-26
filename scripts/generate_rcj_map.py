#!/usr/bin/env python3
"""Generate the RCJ soccer field map and a dimension reference image."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ARENA_WIDTH_CM = 182.0
ARENA_HEIGHT_CM = 243.0
FIELD_OUTER_WIDTH_CM = 158.0
FIELD_OUTER_HEIGHT_CM = 219.0
LINE_WIDTH_CM = 2.0
CENTER_CIRCLE_OUTER_DIAMETER_CM = 60.0
GOAL_AREA_OUTER_WIDTH_CM = 80.0
GOAL_AREA_DEPTH_CM = 25.0
GOAL_AREA_OUTER_RADIUS_CM = 15.0

BLACK = 0
WHITE = 255
REFERENCE_SCALE_PX_PER_CM = 4
REFERENCE_MARGIN_PX = 92
DIMENSION_COLOR = (145, 145, 145)
FIELD_COLOR = (25, 25, 25)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate maps/rcj_map.png, maps/rcj_map.yaml, and a reference image."
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.005,
        help="Meters per pixel for the generated occupancy map.",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=Path("maps/rcj_map.png"),
        help="Output occupancy PNG path.",
    )
    parser.add_argument(
        "--yaml",
        type=Path,
        default=Path("maps/rcj_map.yaml"),
        help="Output Nav2 map YAML path.",
    )
    parser.add_argument(
        "--reference",
        type=Path,
        default=Path("maps/rcj_map_reference.png"),
        help="Output annotated reference PNG path.",
    )
    return parser.parse_args()


def field_edges() -> tuple[float, float, float, float]:
    margin_x = (ARENA_WIDTH_CM - FIELD_OUTER_WIDTH_CM) / 2.0
    margin_y = (ARENA_HEIGHT_CM - FIELD_OUTER_HEIGHT_CM) / 2.0
    return (
        margin_x,
        margin_y,
        ARENA_WIDTH_CM - margin_x,
        ARENA_HEIGHT_CM - margin_y,
    )


class BinaryFieldMap:
    def __init__(self, resolution_m: float) -> None:
        self.resolution_m = resolution_m
        self.px_per_cm = 0.01 / resolution_m
        self.width_px = round(ARENA_WIDTH_CM * self.px_per_cm)
        self.height_px = round(ARENA_HEIGHT_CM * self.px_per_cm)
        self.image = Image.new("L", (self.width_px, self.height_px), WHITE)
        self.draw = ImageDraw.Draw(self.image)

    def x_px(self, x_cm: float) -> int:
        return round(x_cm * self.px_per_cm)

    def y_px(self, y_cm: float) -> int:
        return self.height_px - round(y_cm * self.px_per_cm)

    def box(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> list[int]:
        return [
            self.x_px(left_cm),
            self.y_px(top_cm),
            self.x_px(right_cm) - 1,
            self.y_px(bottom_cm) - 1,
        ]

    def rectangle_ring(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> None:
        self.draw.rectangle(self.box(left_cm, bottom_cm, right_cm, top_cm), fill=BLACK)
        inset = LINE_WIDTH_CM
        self.draw.rectangle(
            self.box(
                left_cm + inset,
                bottom_cm + inset,
                right_cm - inset,
                top_cm - inset,
            ),
            fill=WHITE,
        )

    def filled_rect(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> None:
        self.draw.rectangle(self.box(left_cm, bottom_cm, right_cm, top_cm), fill=BLACK)

    def pieslice_ring(
        self,
        center_x_cm: float,
        center_y_cm: float,
        outer_radius_cm: float,
        inner_radius_cm: float,
        start_deg: float,
        end_deg: float,
    ) -> None:
        self.draw.pieslice(
            self.box(
                center_x_cm - outer_radius_cm,
                center_y_cm - outer_radius_cm,
                center_x_cm + outer_radius_cm,
                center_y_cm + outer_radius_cm,
            ),
            start_deg,
            end_deg,
            fill=BLACK,
        )
        self.draw.pieslice(
            self.box(
                center_x_cm - inner_radius_cm,
                center_y_cm - inner_radius_cm,
                center_x_cm + inner_radius_cm,
                center_y_cm + inner_radius_cm,
            ),
            start_deg,
            end_deg,
            fill=WHITE,
        )

    def ellipse_ring(
        self, center_x_cm: float, center_y_cm: float, outer_diameter_cm: float
    ) -> None:
        outer_radius = outer_diameter_cm / 2.0
        inner_radius = outer_radius - LINE_WIDTH_CM
        self.draw.ellipse(
            self.box(
                center_x_cm - outer_radius,
                center_y_cm - outer_radius,
                center_x_cm + outer_radius,
                center_y_cm + outer_radius,
            ),
            fill=BLACK,
        )
        self.draw.ellipse(
            self.box(
                center_x_cm - inner_radius,
                center_y_cm - inner_radius,
                center_x_cm + inner_radius,
                center_y_cm + inner_radius,
            ),
            fill=WHITE,
        )

    def bottom_goal_area(self, field_bottom_cm: float, center_x_cm: float) -> None:
        outer_left = center_x_cm - (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_right = center_x_cm + (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_top = field_bottom_cm + GOAL_AREA_DEPTH_CM
        radius = GOAL_AREA_OUTER_RADIUS_CM
        inner_radius = radius - LINE_WIDTH_CM

        self.filled_rect(
            outer_left,
            field_bottom_cm,
            outer_left + LINE_WIDTH_CM,
            outer_top - radius,
        )
        self.filled_rect(
            outer_right - LINE_WIDTH_CM,
            field_bottom_cm,
            outer_right,
            outer_top - radius,
        )
        self.filled_rect(
            outer_left + radius,
            outer_top - LINE_WIDTH_CM,
            outer_right - radius,
            outer_top,
        )
        self.pieslice_ring(
            outer_left + radius,
            outer_top - radius,
            radius,
            inner_radius,
            180.0,
            270.0,
        )
        self.pieslice_ring(
            outer_right - radius,
            outer_top - radius,
            radius,
            inner_radius,
            270.0,
            360.0,
        )

    def top_goal_area(self, field_top_cm: float, center_x_cm: float) -> None:
        outer_left = center_x_cm - (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_right = center_x_cm + (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_bottom = field_top_cm - GOAL_AREA_DEPTH_CM
        radius = GOAL_AREA_OUTER_RADIUS_CM
        inner_radius = radius - LINE_WIDTH_CM

        self.filled_rect(
            outer_left,
            outer_bottom + radius,
            outer_left + LINE_WIDTH_CM,
            field_top_cm,
        )
        self.filled_rect(
            outer_right - LINE_WIDTH_CM,
            outer_bottom + radius,
            outer_right,
            field_top_cm,
        )
        self.filled_rect(
            outer_left + radius,
            outer_bottom,
            outer_right - radius,
            outer_bottom + LINE_WIDTH_CM,
        )
        self.pieslice_ring(
            outer_left + radius,
            outer_bottom + radius,
            radius,
            inner_radius,
            90.0,
            180.0,
        )
        self.pieslice_ring(
            outer_right - radius,
            outer_bottom + radius,
            radius,
            inner_radius,
            0.0,
            90.0,
        )

    def draw_field(self) -> None:
        field_left, field_bottom, field_right, field_top = field_edges()
        center_x = ARENA_WIDTH_CM / 2.0
        center_y = ARENA_HEIGHT_CM / 2.0

        self.rectangle_ring(field_left, field_bottom, field_right, field_top)
        self.ellipse_ring(center_x, center_y, CENTER_CIRCLE_OUTER_DIAMETER_CM)
        self.bottom_goal_area(field_bottom, center_x)
        self.top_goal_area(field_top, center_x)

    def save(self, png_path: Path, yaml_path: Path) -> None:
        png_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        self.image.save(png_path)

        origin_x = -(ARENA_WIDTH_CM * 0.01) / 2.0
        origin_y = -(ARENA_HEIGHT_CM * 0.01) / 2.0
        yaml_path.write_text(
            "\n".join(
                [
                    f"image: {png_path.name}",
                    f"resolution: {self.resolution_m:.6g}",
                    f"origin: [{origin_x:.6g}, {origin_y:.6g}, 0.0]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.196",
                    "",
                ]
            ),
            encoding="utf-8",
        )


class ReferenceDrawing:
    def __init__(self) -> None:
        width = round(ARENA_WIDTH_CM * REFERENCE_SCALE_PX_PER_CM)
        height = round(ARENA_HEIGHT_CM * REFERENCE_SCALE_PX_PER_CM)
        self.width_px = width + (2 * REFERENCE_MARGIN_PX)
        self.height_px = height + (2 * REFERENCE_MARGIN_PX)
        self.image = Image.new("RGB", (self.width_px, self.height_px), "white")
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()

    def point(self, x_cm: float, y_cm: float) -> tuple[int, int]:
        return (
            round(REFERENCE_MARGIN_PX + x_cm * REFERENCE_SCALE_PX_PER_CM),
            round(
                REFERENCE_MARGIN_PX
                + (ARENA_HEIGHT_CM - y_cm) * REFERENCE_SCALE_PX_PER_CM
            ),
        )

    def box(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> list[int]:
        x0, y0 = self.point(left_cm, top_cm)
        x1, y1 = self.point(right_cm, bottom_cm)
        return [x0, y0, x1, y1]

    def text_center(self, xy: tuple[int, int], text: str) -> None:
        bbox = self.draw.textbbox((0, 0), text, font=self.font)
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        x = xy[0] - (width // 2)
        y = xy[1] - (height // 2)
        pad = 2
        self.draw.rectangle(
            [x - pad, y - pad, x + width + pad, y + height + pad],
            fill="white",
        )
        self.draw.text((x, y), text, fill=DIMENSION_COLOR, font=self.font)

    def dimension_arrow(self, p1: tuple[int, int], p2: tuple[int, int]) -> None:
        self.draw.line([p1, p2], fill=DIMENSION_COLOR, width=1)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length == 0:
            return
        ux = dx / length
        uy = dy / length
        px = -uy
        py = ux
        arrow = 7
        for base, sign in ((p1, 1), (p2, -1)):
            tip_x = base[0]
            tip_y = base[1]
            back_x = tip_x + sign * ux * arrow
            back_y = tip_y + sign * uy * arrow
            self.draw.line(
                [(tip_x, tip_y), (back_x + px * 3, back_y + py * 3)],
                fill=DIMENSION_COLOR,
                width=1,
            )
            self.draw.line(
                [(tip_x, tip_y), (back_x - px * 3, back_y - py * 3)],
                fill=DIMENSION_COLOR,
                width=1,
            )

    def hdim(
        self, x1_cm: float, x2_cm: float, y_dim_cm: float, y_anchor_cm: float, text: str
    ) -> None:
        p1 = self.point(x1_cm, y_dim_cm)
        p2 = self.point(x2_cm, y_dim_cm)
        a1 = self.point(x1_cm, y_anchor_cm)
        a2 = self.point(x2_cm, y_anchor_cm)
        self.draw.line([a1, p1], fill=DIMENSION_COLOR, width=1)
        self.draw.line([a2, p2], fill=DIMENSION_COLOR, width=1)
        self.dimension_arrow(p1, p2)
        mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2 - 10)
        self.text_center(mid, text)

    def vdim(
        self,
        x_dim_cm: float,
        y1_cm: float,
        y2_cm: float,
        x_anchor_cm: float,
        text: str,
        label_dx_px: int = 36,
    ) -> None:
        p1 = self.point(x_dim_cm, y1_cm)
        p2 = self.point(x_dim_cm, y2_cm)
        a1 = self.point(x_anchor_cm, y1_cm)
        a2 = self.point(x_anchor_cm, y2_cm)
        self.draw.line([a1, p1], fill=DIMENSION_COLOR, width=1)
        self.draw.line([a2, p2], fill=DIMENSION_COLOR, width=1)
        self.dimension_arrow(p1, p2)
        mid = (p1[0] + label_dx_px, (p1[1] + p2[1]) // 2)
        self.text_center(mid, text)

    def rdim(
        self,
        center_x_cm: float,
        center_y_cm: float,
        angle_deg: float,
        radius_cm: float,
        text: str,
        label_dx_px: int,
        label_dy_px: int,
    ) -> None:
        edge_x_cm = center_x_cm + radius_cm * math.cos(math.radians(angle_deg))
        edge_y_cm = center_y_cm + radius_cm * math.sin(math.radians(angle_deg))
        p1 = self.point(center_x_cm, center_y_cm)
        p2 = self.point(edge_x_cm, edge_y_cm)
        self.dimension_arrow(p1, p2)
        self.text_center(
            (
                (p1[0] + p2[0]) // 2 + label_dx_px,
                (p1[1] + p2[1]) // 2 + label_dy_px,
            ),
            text,
        )

    def rectangle_ring(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> None:
        self.draw.rectangle(self.box(left_cm, bottom_cm, right_cm, top_cm), fill=FIELD_COLOR)
        inset = LINE_WIDTH_CM
        self.draw.rectangle(
            self.box(
                left_cm + inset,
                bottom_cm + inset,
                right_cm - inset,
                top_cm - inset,
            ),
            fill="white",
        )

    def filled_rect(
        self, left_cm: float, bottom_cm: float, right_cm: float, top_cm: float
    ) -> None:
        self.draw.rectangle(
            self.box(left_cm, bottom_cm, right_cm, top_cm),
            fill=FIELD_COLOR,
        )

    def pieslice_ring(
        self,
        center_x_cm: float,
        center_y_cm: float,
        outer_radius_cm: float,
        inner_radius_cm: float,
        start_deg: float,
        end_deg: float,
    ) -> None:
        self.draw.pieslice(
            self.box(
                center_x_cm - outer_radius_cm,
                center_y_cm - outer_radius_cm,
                center_x_cm + outer_radius_cm,
                center_y_cm + outer_radius_cm,
            ),
            start_deg,
            end_deg,
            fill=FIELD_COLOR,
        )
        self.draw.pieslice(
            self.box(
                center_x_cm - inner_radius_cm,
                center_y_cm - inner_radius_cm,
                center_x_cm + inner_radius_cm,
                center_y_cm + inner_radius_cm,
            ),
            start_deg,
            end_deg,
            fill="white",
        )

    def ellipse_ring(
        self, center_x_cm: float, center_y_cm: float, outer_diameter_cm: float
    ) -> None:
        outer_radius = outer_diameter_cm / 2.0
        inner_radius = outer_radius - LINE_WIDTH_CM
        self.draw.ellipse(
            self.box(
                center_x_cm - outer_radius,
                center_y_cm - outer_radius,
                center_x_cm + outer_radius,
                center_y_cm + outer_radius,
            ),
            fill=FIELD_COLOR,
        )
        self.draw.ellipse(
            self.box(
                center_x_cm - inner_radius,
                center_y_cm - inner_radius,
                center_x_cm + inner_radius,
                center_y_cm + inner_radius,
            ),
            fill="white",
        )

    def bottom_goal_area(self, field_bottom_cm: float, center_x_cm: float) -> None:
        outer_left = center_x_cm - (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_right = center_x_cm + (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_top = field_bottom_cm + GOAL_AREA_DEPTH_CM
        radius = GOAL_AREA_OUTER_RADIUS_CM
        inner_radius = radius - LINE_WIDTH_CM

        self.filled_rect(
            outer_left,
            field_bottom_cm,
            outer_left + LINE_WIDTH_CM,
            outer_top - radius,
        )
        self.filled_rect(
            outer_right - LINE_WIDTH_CM,
            field_bottom_cm,
            outer_right,
            outer_top - radius,
        )
        self.filled_rect(
            outer_left + radius,
            outer_top - LINE_WIDTH_CM,
            outer_right - radius,
            outer_top,
        )
        self.pieslice_ring(
            outer_left + radius,
            outer_top - radius,
            radius,
            inner_radius,
            180.0,
            270.0,
        )
        self.pieslice_ring(
            outer_right - radius,
            outer_top - radius,
            radius,
            inner_radius,
            270.0,
            360.0,
        )

    def top_goal_area(self, field_top_cm: float, center_x_cm: float) -> None:
        outer_left = center_x_cm - (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_right = center_x_cm + (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        outer_bottom = field_top_cm - GOAL_AREA_DEPTH_CM
        radius = GOAL_AREA_OUTER_RADIUS_CM
        inner_radius = radius - LINE_WIDTH_CM

        self.filled_rect(
            outer_left,
            outer_bottom + radius,
            outer_left + LINE_WIDTH_CM,
            field_top_cm,
        )
        self.filled_rect(
            outer_right - LINE_WIDTH_CM,
            outer_bottom + radius,
            outer_right,
            field_top_cm,
        )
        self.filled_rect(
            outer_left + radius,
            outer_bottom,
            outer_right - radius,
            outer_bottom + LINE_WIDTH_CM,
        )
        self.pieslice_ring(
            outer_left + radius,
            outer_bottom + radius,
            radius,
            inner_radius,
            90.0,
            180.0,
        )
        self.pieslice_ring(
            outer_right - radius,
            outer_bottom + radius,
            radius,
            inner_radius,
            0.0,
            90.0,
        )

    def draw_field(self) -> None:
        field_left, field_bottom, field_right, field_top = field_edges()
        center_x = ARENA_WIDTH_CM / 2.0
        center_y = ARENA_HEIGHT_CM / 2.0

        self.draw.rectangle(
            self.box(0.0, 0.0, ARENA_WIDTH_CM, ARENA_HEIGHT_CM),
            outline=(210, 210, 210),
            width=1,
        )
        self.rectangle_ring(field_left, field_bottom, field_right, field_top)
        self.ellipse_ring(center_x, center_y, CENTER_CIRCLE_OUTER_DIAMETER_CM)
        self.bottom_goal_area(field_bottom, center_x)
        self.top_goal_area(field_top, center_x)

    def draw_dimensions(self) -> None:
        field_left, field_bottom, field_right, field_top = field_edges()
        center_x = ARENA_WIDTH_CM / 2.0
        center_y = ARENA_HEIGHT_CM / 2.0
        goal_left = center_x - (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        goal_right = center_x + (GOAL_AREA_OUTER_WIDTH_CM / 2.0)
        radius = GOAL_AREA_OUTER_RADIUS_CM

        self.hdim(0.0, ARENA_WIDTH_CM, ARENA_HEIGHT_CM + 8.0, ARENA_HEIGHT_CM, "arena 182")
        self.vdim(-8.0, 0.0, ARENA_HEIGHT_CM, 0.0, "arena 243")

        self.hdim(field_left, field_right, field_bottom - 5.0, field_bottom, "field outer 158")
        self.hdim(
            field_left + LINE_WIDTH_CM,
            field_right - LINE_WIDTH_CM,
            field_bottom + 9.0,
            field_bottom + LINE_WIDTH_CM,
            "field inner 154",
        )
        self.vdim(
            field_right + 8.0,
            field_bottom,
            field_top,
            field_right,
            "field outer 219",
            label_dx_px=48,
        )
        self.vdim(
            field_right - 8.0,
            field_bottom + LINE_WIDTH_CM,
            field_top - LINE_WIDTH_CM,
            field_right - LINE_WIDTH_CM,
            "field inner 215",
            label_dx_px=-48,
        )

        self.hdim(goal_left, goal_right, field_bottom + 31.0, field_bottom + 25.0, "goal outer 80")
        self.hdim(
            goal_left + LINE_WIDTH_CM,
            goal_right - LINE_WIDTH_CM,
            field_bottom + 27.0,
            field_bottom + 23.0,
            "goal inner 76",
        )
        self.vdim(goal_right + 8.0, field_bottom, field_bottom + GOAL_AREA_DEPTH_CM, goal_right, "depth 25")
        self.rdim(
            goal_right - radius,
            field_bottom + GOAL_AREA_DEPTH_CM - radius,
            45.0,
            radius,
            "R15",
            label_dx_px=18,
            label_dy_px=-12,
        )

        self.hdim(goal_left, goal_right, field_top - 31.0, field_top - 25.0, "goal outer 80")
        self.hdim(
            goal_left + LINE_WIDTH_CM,
            goal_right - LINE_WIDTH_CM,
            field_top - 27.0,
            field_top - 23.0,
            "goal inner 76",
        )
        self.vdim(goal_right + 8.0, field_top - GOAL_AREA_DEPTH_CM, field_top, goal_right, "depth 25")
        self.rdim(
            goal_right - radius,
            field_top - GOAL_AREA_DEPTH_CM + radius,
            -45.0,
            radius,
            "R15",
            label_dx_px=18,
            label_dy_px=12,
        )

        self.hdim(
            center_x - (CENTER_CIRCLE_OUTER_DIAMETER_CM / 2.0),
            center_x + (CENTER_CIRCLE_OUTER_DIAMETER_CM / 2.0),
            center_y + 37.0,
            center_y + (CENTER_CIRCLE_OUTER_DIAMETER_CM / 2.0),
            "circle outer 60",
        )
        self.hdim(
            center_x - ((CENTER_CIRCLE_OUTER_DIAMETER_CM - (2.0 * LINE_WIDTH_CM)) / 2.0),
            center_x + ((CENTER_CIRCLE_OUTER_DIAMETER_CM - (2.0 * LINE_WIDTH_CM)) / 2.0),
            center_y - 37.0,
            center_y - ((CENTER_CIRCLE_OUTER_DIAMETER_CM - (2.0 * LINE_WIDTH_CM)) / 2.0),
            "circle inner 56",
        )
        self.hdim(
            field_left,
            field_left + LINE_WIDTH_CM,
            center_y + 71.0,
            center_y + 71.0,
            "2",
        )
        self.text_center(self.point(field_left - 5.0, center_y + 75.0), "line width 2")

    def save(self, reference_path: Path) -> None:
        reference_path.parent.mkdir(parents=True, exist_ok=True)
        self.image.save(reference_path)


def main() -> None:
    args = parse_args()
    field_map = BinaryFieldMap(args.resolution)
    field_map.draw_field()
    field_map.save(args.png, args.yaml)

    reference = ReferenceDrawing()
    reference.draw_field()
    reference.draw_dimensions()
    reference.save(args.reference)

    print(
        f"Generated {args.png} ({field_map.width_px}x{field_map.height_px}) "
        f"at {args.resolution:g} m/px"
    )
    print(f"Generated {args.reference}")


if __name__ == "__main__":
    main()
