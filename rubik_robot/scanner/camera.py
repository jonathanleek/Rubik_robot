"""
PiCamera wrapper for cube face photography.

Manages the Raspberry Pi camera lifecycle (initialization, capture,
shutdown). The camera is initialized once at startup and kept alive
for the duration of the program to avoid the 2-second warm-up delay
on each scan.

The camera captures 1080x1080 images with automatic exposure. Images
are saved to ~/Cube/ and later analyzed by the color detection module.
"""

import os
import time
from picamera import PiCamera

from rubik_robot.config import HOME, IMG_WIDTH, IMG_HEIGHT


class Camera:
    """Wrapper around PiCamera with managed lifecycle.

    The camera is initialized with a resolution of 1080x1080 and
    automatic exposure mode. A 2-second warm-up period is required
    after initialization for the auto-exposure to stabilize.

    Usage:
        camera = Camera()
        camera.setup()          # Initialize and warm up
        camera.capture("path")  # Take a photo
        camera.close()          # Shut down
    """

    def __init__(self):
        self._camera = None

    def setup(self):
        """Initialize the camera and wait for auto-exposure to stabilize.

        Also creates the ~/Cube/ directory for storing face images
        if it does not already exist.
        """
        # Ensure the image output directory exists
        cube_dir = os.path.join(HOME, "Cube")
        if not os.path.exists(cube_dir):
            os.makedirs(cube_dir)
            os.chmod(cube_dir, 0o777)

        # Initialize camera
        self._camera = PiCamera()
        self._camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
        self._camera.exposure_mode = "auto"
        self._camera.start_preview()

        # Wait for auto-exposure to stabilize
        time.sleep(2)

    def capture(self, filename):
        """Capture an image and save it to the specified path.

        Args:
            filename: Full file path for the saved image (JPEG format).
        """
        self._camera.capture(filename)

    def close(self):
        """Shut down the camera and release resources."""
        if self._camera:
            self._camera.close()
            self._camera = None
