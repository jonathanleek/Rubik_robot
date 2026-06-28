"""Cube colour detection.

Refactored port of ``pix_average`` / ``get_sticker`` from the original
firmware. Given the six face images captured during a scan, it produces a
54-character facelet string in kociemba order (U, R, F, D, L, B).

The classifier is relative: each sticker is matched (by Euclidean RGB distance)
to the six centre stickers sampled from the same capture session, after a
per-image manual white-balance correction. Because it is purely relative, it is
robust to the camera reporting channels in RGB vs BGR order, as long as every
image is captured the same way.

Only depends on Pillow, so it is testable off-device.
"""

from __future__ import annotations

from .config import PIXEL_LOCATIONS, WB_COL_PX, WB_ROW_PX

# Image index -> face letter. The scan sequence (see ``scan.py``) is arranged so
# that captured image N shows the face listed here.
FACE_ORDER = ["U", "R", "F", "D", "L", "B"]  # indices 0..5
_CENTER = PIXEL_LOCATIONS[1][1]  # (x, y) of the centre sticker


def pixel_average(image, x: int, y: int) -> tuple[float, float, float]:
    """Average RGB over a 10x10 patch centred on (x, y)."""
    r = g = b = 0
    for i in range(10):
        for j in range(10):
            r1, g1, b1 = image.getpixel((x - 5 + i, y - 5 + j))
            r += r1
            g += g1
            b += b1
    return r / 100.0, g / 100.0, b / 100.0


def _white_balanced_center(image) -> tuple[float, float, float]:
    """Centre sticker colour, normalised against the white-balance patch."""
    cr, cg, cb = pixel_average(image, _CENTER[0], _CENTER[1])
    wr, wg, wb = pixel_average(image, WB_COL_PX, WB_ROW_PX)
    return cr / wr * 255, cg / wg * 255, cb / wb * 255


def _sq_dist(a, b) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def get_facelets(images) -> str:
    """Classify all 54 stickers from six captured face images.

    ``images`` is indexable 0..5 (list or dict) of Pillow images; index N must
    correspond to ``FACE_ORDER[N]``. Returns the 54-char facelet string.
    """
    rgb = [images[i].convert("RGB") for i in range(6)]

    # Reference colour for each face, taken from its centre sticker.
    base = {FACE_ORDER[i]: _white_balanced_center(rgb[i]) for i in range(6)}

    stickers = [""] * 54
    for img_index in range(6):
        image = rgb[img_index]
        wr, wg, wb = pixel_average(image, WB_COL_PX, WB_ROW_PX)
        for col in range(3):
            for row in range(3):
                x, y = PIXEL_LOCATIONS[row][col]
                r, g, b = pixel_average(image, x, y)
                sample = (r / wr * 255, g / wg * 255, b / wb * 255)
                color = min(base, key=lambda f: _sq_dist(sample, base[f]))
                stickers[img_index * 9 + 3 * row + col] = color

    _reorder_capture_rotation(stickers)
    return "".join(stickers)


def _reorder_capture_rotation(stickers: list[str]) -> None:
    """Undo the in-plane rotation of the U and D faces caused by the scan moves.

    Identical index shuffle to the original's "Korrektur oben/unten".
    """
    # Top (U) face, indices 0..8.
    d1, d2 = stickers[0], stickers[1]
    stickers[0] = stickers[6]
    stickers[1] = stickers[3]
    stickers[6] = stickers[8]
    stickers[3] = stickers[7]
    stickers[8] = stickers[2]
    stickers[7] = stickers[5]
    stickers[2] = d1
    stickers[5] = d2

    # Bottom (D) face, indices 27..35.
    d1, d2 = stickers[27], stickers[28]
    stickers[27] = stickers[33]
    stickers[28] = stickers[30]
    stickers[33] = stickers[35]
    stickers[30] = stickers[34]
    stickers[35] = stickers[29]
    stickers[34] = stickers[32]
    stickers[29] = d1
    stickers[32] = d2
