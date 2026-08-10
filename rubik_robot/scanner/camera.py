"""
Picamera2 wrapper for cube face photography.

Manages the Raspberry Pi camera lifecycle (initialization, capture,
shutdown) using the modern picamera2 stack (libcamera). The camera is
initialized once at startup and kept alive for the duration of the
program to avoid repeated warm-up delays on each scan.

The camera captures 1080x1080 images with automatic exposure. Images
are saved to ~/Cube/ and later analyzed by the color detection module.

Note: picamera2 is installed via the system package manager
(``sudo apt install -y python3-picamera2``), not pip, because it depends
on the system libcamera stack. This wrapper keeps the same public
interface as the previous PiCamera version (setup/capture/close) so the
rest of the code is unaffected.
"""

import os
import time
from picamera2 import Picamera2

from rubik_robot.config import HOME, IMG_WIDTH, IMG_HEIGHT


class Camera:
    """Wrapper around Picamera2 with managed lifecycle.

    The camera is configured for still capture at IMG_WIDTH x IMG_HEIGHT
    with automatic exposure and white balance (picamera2's defaults). A
    short warm-up period after start() lets auto-exposure settle, matching
    the behavior of the original PiCamera implementation.

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

        # Initialize the camera for still capture. picamera2 enables
        # automatic exposure and white balance by default.
        self._camera = Picamera2()
        still_config = self._camera.create_still_configuration(
            main={"size": (IMG_WIDTH, IMG_HEIGHT)}
        )
        self._camera.configure(still_config)
        self._camera.start()

        # Wait for auto-exposure / auto-white-balance to stabilize
        time.sleep(2)

    def capture(self, filename):
        """Capture an image and save it to the specified path.

        The image format is inferred from the file extension (e.g. ``.jpg``
        produces a JPEG), matching the previous PiCamera behavior.

        Args:
            filename: Full file path for the saved image (JPEG format).
        """
        self._camera.capture_file(filename)

    def close(self):
        """Shut down the camera and release resources."""
        if self._camera:
            self._camera.stop()
            self._camera.close()
            self._camera = None
