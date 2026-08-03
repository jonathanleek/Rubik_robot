"""
Cube face color detection.

Analyzes photographs of the cube faces to determine the color of each
sticker. Uses a simple nearest-neighbor approach: the center sticker of
each face is used as the reference color, and all other stickers are
classified by finding the closest reference color in RGB space.

White balance correction is applied by sampling a known neutral area
in each image and normalizing the RGB values.

The output is a 54-character string in Kociemba URFDLB notation,
where each character represents the color of one sticker on the cube.
The string is ordered: U face (9 chars), R face (9), F face (9),
D face (9), L face (9), B face (9).
"""

import math
from PIL import Image

from rubik_robot.config import HOME


def pix_average(im, x, y):
    """Average the RGB values in a 10x10 pixel area centered at (x, y).

    Averaging reduces noise from individual pixels and gives a more
    reliable color reading for each sticker.

    Args:
        im: PIL Image in RGB mode.
        x: Center x coordinate of the sampling area.
        y: Center y coordinate of the sampling area.

    Returns:
        Tuple of (r, g, b) average values as floats.
    """
    r, g, b = 0, 0, 0
    for i in range(10):
        for j in range(10):
            r1, g1, b1 = im.getpixel((x - 5 + i, y - 5 + j))
            r += r1
            g += g1
            b += b1
    return r / 100, g / 100, b / 100


def get_sticker(pixel_locs):
    """Analyze all six face images and return the cube state string.

    Process:
    1. Read the center sticker of each face as the reference color
    2. Apply white-balance correction to each reference color
    3. For each of the 54 stickers, find the closest reference color
       using Euclidean distance in RGB space
    4. Apply sticker order corrections for faces that were photographed
       in a rotated orientation

    The face images must already exist at ~/Cube/face0.jpg through
    ~/Cube/face5.jpg (captured by the cube_reader module).

    Face image mapping:
        face0.jpg = U (up) face
        face1.jpg = R (right) face
        face2.jpg = F (front) face
        face3.jpg = D (down) face
        face4.jpg = L (left) face
        face5.jpg = B (back) face

    Args:
        pixel_locs: PixelLocations instance with sampling coordinates
            and white-balance reference position.

    Returns:
        54-character string representing the cube state in URFDLB notation.
    """
    pxl_grid = pixel_locs.get_grid()
    wb_col = pixel_locs.wb_col
    wb_row = pixel_locs.wb_row

    col_sticker = [""] * 54

    # --- Step 1: Read center sticker of each face as reference colors ---

    # Face ordering: [R, -, F, D, L, B] for files 1,_,2,3,4,5
    # (face0 = U, read last since it's the starting position)
    face_files = {
        "R": "face1.jpg",
        "B": "face5.jpg",
        "U": "face0.jpg",
        "D": "face3.jpg",
        "F": "face2.jpg",
        "L": "face4.jpg",
    }

    # Center sticker coordinates (middle of the 3x3 grid)
    center_x = pxl_grid[1][1][0]
    center_y = pxl_grid[1][1][1]

    base_colors = {}  # {face_letter: (r, g, b)} after white balance

    for face, filename in face_files.items():
        im = Image.open(HOME + "Cube/" + filename)
        im = im.convert("RGB")

        # Get raw center color
        r, g, b = pix_average(im, center_x, center_y)

        # White-balance correction: normalize to what white would be
        wb_r, wb_g, wb_b = pix_average(im, wb_col, wb_row)
        base_colors[face] = (
            r / wb_r * 255,
            g / wb_g * 255,
            b / wb_b * 255,
        )

    # --- Step 2: Classify each sticker by nearest reference color ---

    face_order = ["U", "R", "F", "D", "L", "B"]

    for img_idx in range(6):
        img_path = HOME + "Cube/face" + str(img_idx) + ".jpg"
        im = Image.open(img_path)
        im = im.convert("RGB")

        # White-balance for this image
        wb_r, wb_g, wb_b = pix_average(im, wb_col, wb_row)

        for y_idx in range(3):
            for x_idx in range(3):
                # Get white-balance-corrected pixel color
                r, g, b = pix_average(
                    im, pxl_grid[y_idx][x_idx][0], pxl_grid[y_idx][x_idx][1]
                )
                r = r / wb_r * 255
                g = g / wb_g * 255
                b = b / wb_b * 255

                # Find closest reference color (Euclidean distance in RGB)
                min_dist = float("inf")
                best_color = "U"

                for face_letter, (br, bg, bb) in base_colors.items():
                    dist = (
                        math.pow(r - br, 2)
                        + math.pow(g - bg, 2)
                        + math.pow(b - bb, 2)
                    )
                    if dist < min_dist:
                        min_dist = dist
                        best_color = face_letter

                col_sticker[img_idx * 9 + 3 * y_idx + x_idx] = best_color

    # --- Step 3: Apply sticker order corrections ---
    # The U and D faces are photographed in a rotated orientation,
    # so their sticker order needs to be corrected.

    # Correction for U face (top) - rotate sticker positions
    _rotate_face(col_sticker, 0)

    # Correction for D face (bottom) - rotate sticker positions
    _rotate_face(col_sticker, 27)

    # Build the result string
    result = ""
    for sticker in col_sticker:
        result += sticker

    return result


def _rotate_face(stickers, offset):
    """Rotate the sticker order for a face that was photographed rotated.

    The camera photographs the U and D faces in a different orientation
    than the standard Kociemba notation expects. This function applies
    the necessary permutation to correct the sticker order.

    The permutation is: 0->6, 1->3, 2->0, 3->7, 5->1, 6->8, 7->5, 8->2

    Args:
        stickers: List of 54 sticker color characters (modified in place).
        offset: Starting index of the face to rotate (0 for U, 27 for D).
    """
    d1 = stickers[offset + 0]
    d2 = stickers[offset + 1]
    stickers[offset + 0] = stickers[offset + 6]
    stickers[offset + 1] = stickers[offset + 3]
    stickers[offset + 6] = stickers[offset + 8]
    stickers[offset + 3] = stickers[offset + 7]
    stickers[offset + 8] = stickers[offset + 2]
    stickers[offset + 7] = stickers[offset + 5]
    stickers[offset + 2] = d1
    stickers[offset + 5] = d2
