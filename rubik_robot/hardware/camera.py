"""Raspberry Pi camera driver.

Uses ``picamera2`` (the supported library on Raspberry Pi OS Bookworm; the
original's ``picamera`` is legacy and does not work there). Captures are
returned as in-memory Pillow images rather than written to disk.
"""

from __future__ import annotations

import time
from typing import Protocol

from ..config import CAMERA_ROTATION, IMG_HEIGHT, IMG_WIDTH


def rotate_image(image, degrees: int):
    """Rotate a Pillow image clockwise by 0/90/180/270 degrees (lossless)."""
    from PIL import Image

    mapping = {
        0: None,
        90: Image.Transpose.ROTATE_270,   # PIL rotates CCW; 270 CCW == 90 CW
        180: Image.Transpose.ROTATE_180,
        270: Image.Transpose.ROTATE_90,
    }
    if degrees % 90 != 0:
        return image.rotate(-degrees, expand=True)  # arbitrary angle, CW
    op = mapping[degrees % 360]
    return image if op is None else image.transpose(op)


class Camera(Protocol):
    def capture(self): ...  # returns a Pillow Image

    def close(self) -> None: ...


class PiCamera2:
    """Real camera driver backed by picamera2."""

    def __init__(
        self,
        width: int = IMG_WIDTH,
        height: int = IMG_HEIGHT,
        warmup: float = 2.0,
        rotation: int = CAMERA_ROTATION,
    ):
        from picamera2 import Picamera2  # type: ignore

        self._rotation = rotation
        self._cam = Picamera2()
        config = self._cam.create_still_configuration(main={"size": (width, height)})
        self._cam.configure(config)
        self._cam.start()
        time.sleep(warmup)  # let auto-exposure / white-balance settle

    def capture(self):
        from PIL import Image

        array = self._cam.capture_array()  # numpy HxWx3 (or 4) array
        image = Image.fromarray(array).convert("RGB")
        return rotate_image(image, self._rotation)

    def close(self) -> None:
        try:
            self._cam.stop()
        finally:
            self._cam.close()


class MockCamera:
    """Returns a solid mid-grey image so the API runs without a camera.

    Colour detection on these images is meaningless, but every endpoint and the
    full scan motion sequence can be exercised end-to-end.
    """

    def __init__(self, width: int = IMG_WIDTH, height: int = IMG_HEIGHT, **_kwargs):
        self._size = (width, height)

    def capture(self):
        from PIL import Image

        return Image.new("RGB", self._size, (128, 128, 128))

    def close(self) -> None:  # pragma: no cover
        pass
