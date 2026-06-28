"""Entry point: ``python -m rubik_robot``.

Builds the FastAPI app around a real or mock ``Robot`` and serves it with
uvicorn. Use ``--mock`` (or ``RUBIK_MOCK=1``) to run with no hardware attached,
which is handy for developing and testing the client against the real API
surface.
"""

from __future__ import annotations

import argparse
import os

import uvicorn

from .api import create_app
from .config import Calibration
from .robot import Robot


def build_robot(servos: str = "real", camera: str = "real", rotation: int | None = None) -> Robot:
    """Build a Robot with independently selectable servo/camera backends.

    ``servos`` / ``camera`` are each "real" or "mock". This lets us run the
    camera before the PCA9685 is wired (``--servos mock --camera real``).
    """
    if servos == "mock":
        from .hardware.servos import MockServoDriver

        driver = MockServoDriver()
    else:
        from .config import PCA9685_ADDRESS
        from .hardware.servos import PCA9685ServoDriver

        driver = PCA9685ServoDriver(address=PCA9685_ADDRESS)

    if camera == "mock":
        from .hardware.camera import MockCamera

        cam = MockCamera()
    else:
        from .config import CAMERA_ROTATION
        from .hardware.camera import PiCamera2

        cam = PiCamera2(rotation=CAMERA_ROTATION if rotation is None else rotation)

    return Robot(driver, cam)


def main() -> None:
    parser = argparse.ArgumentParser(description="Rubik robot hardware API server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--mock",
        action="store_true",
        default=os.environ.get("RUBIK_MOCK", "") == "1",
        help="shortcut for --servos mock --camera mock (no hardware required)",
    )
    parser.add_argument(
        "--servos",
        choices=("real", "mock"),
        default=os.environ.get("RUBIK_SERVOS", "real"),
        help="servo backend (default real; use mock before the PCA9685 is wired)",
    )
    parser.add_argument(
        "--camera",
        choices=("real", "mock"),
        default=os.environ.get("RUBIK_CAMERA", "real"),
        help="camera backend (default real)",
    )
    parser.add_argument(
        "--rotation",
        type=int,
        default=None,
        help="override camera rotation in degrees clockwise (0/90/180/270)",
    )
    parser.add_argument(
        "--home-on-start",
        action="store_true",
        help="move the robot to the home position before serving",
    )
    args = parser.parse_args()

    servos = "mock" if args.mock else args.servos
    camera = "mock" if args.mock else args.camera

    # Surface calibration problems early.
    Calibration.from_disk()

    robot = build_robot(servos=servos, camera=camera, rotation=args.rotation)
    if args.home_on_start and servos == "real":
        robot.home()

    app = create_app(robot)
    try:
        uvicorn.run(app, host=args.host, port=args.port)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
