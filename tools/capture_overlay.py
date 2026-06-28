#!/usr/bin/env python3
"""Capture a frame and overlay the colour-sampling points, for scan tuning.

Run on the Pi to visualise exactly where ``vision.get_facelets`` samples each
of the 9 stickers (and the white-balance patch). Hold a cube in the grippers,
capture, copy the annotated image to your laptop, and adjust the grid until the
boxes sit on the sticker centres.

The 3x3 grid columns/rows and the WB patch can be overridden on the command line
so you can iterate without editing config.py:

    .venv/bin/python tools/capture_overlay.py \
        --rotation 0 --cols 230,450,730 --rows 230,500,730 --wb 890,980 \
        --out /tmp/overlay.jpg

Use --from-file to annotate an existing capture instead of taking a new one.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PIL import Image, ImageDraw  # noqa: E402

from rubik_robot import config  # noqa: E402


def _triple(s: str) -> list[int]:
    parts = [int(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated ints")
    return parts


def _pair(s: str) -> tuple[int, int]:
    parts = [int(x) for x in s.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected two comma-separated ints")
    return parts[0], parts[1]


def annotate(image: Image.Image, cols, rows, wb) -> Image.Image:
    draw = ImageDraw.Draw(image)
    # 9 sticker sample boxes (10x10, matching pixel_average) with row/col labels.
    for ri, y in enumerate(rows):
        for ci, x in enumerate(cols):
            draw.rectangle([x - 5, y - 5, x + 5, y + 5], outline=(255, 0, 0), width=3)
            draw.line([x - 14, y, x + 14, y], fill=(255, 0, 0), width=1)
            draw.line([x, y - 14, x, y + 14], fill=(255, 0, 0), width=1)
            draw.text((x + 10, y + 10), f"{ri},{ci}", fill=(255, 255, 0))
    # White-balance patch.
    wx, wy = wb
    draw.rectangle([wx - 5, wy - 5, wx + 5, wy + 5], outline=(0, 128, 255), width=3)
    draw.text((wx + 10, wy + 10), "WB", fill=(0, 128, 255))
    return image


def main() -> None:
    default_cols = [config.LFT_COL_PX, config.MID_COL_PX, config.RGT_COL_PX]
    default_rows = [config.TOP_ROW_PX, config.MID_ROW_PX, config.BOT_ROW_PX]

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cols", type=_triple, default=default_cols, help="3 sample column x-coords")
    p.add_argument("--rows", type=_triple, default=default_rows, help="3 sample row y-coords")
    p.add_argument("--wb", type=_pair, default=(config.WB_COL_PX, config.WB_ROW_PX), help="white-balance x,y")
    p.add_argument("--rotation", type=int, default=config.CAMERA_ROTATION, help="camera rotation, deg CW")
    p.add_argument("--from-file", default=None, help="annotate this image instead of capturing")
    p.add_argument("--out", default="/tmp/overlay.jpg", help="output path")
    args = p.parse_args()

    if args.from_file:
        from rubik_robot.hardware.camera import rotate_image

        image = rotate_image(Image.open(args.from_file).convert("RGB"), args.rotation)
    else:
        from rubik_robot.hardware.camera import PiCamera2

        cam = PiCamera2(rotation=args.rotation)
        try:
            image = cam.capture()
        finally:
            cam.close()

    annotate(image, args.cols, args.rows, args.wb).save(args.out)
    print(f"cols={args.cols} rows={args.rows} wb={args.wb} rotation={args.rotation}")
    print(f"wrote {args.out} ({image.size[0]}x{image.size[1]})")


if __name__ == "__main__":
    main()
