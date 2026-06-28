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


def build_robot(mock: bool) -> Robot:
    if mock:
        from .hardware.camera import MockCamera
        from .hardware.servos import MockServoDriver

        return Robot(MockServoDriver(), MockCamera())

    from .config import PCA9685_ADDRESS
    from .hardware.camera import PiCamera2
    from .hardware.servos import PCA9685ServoDriver

    driver = PCA9685ServoDriver(address=PCA9685_ADDRESS)
    camera = PiCamera2()
    return Robot(driver, camera)


def main() -> None:
    parser = argparse.ArgumentParser(description="Rubik robot hardware API server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--mock",
        action="store_true",
        default=os.environ.get("RUBIK_MOCK", "") == "1",
        help="run with mock hardware (no servos/camera required)",
    )
    parser.add_argument(
        "--home-on-start",
        action="store_true",
        help="move the robot to the home position before serving",
    )
    args = parser.parse_args()

    # Surface calibration problems early.
    Calibration.from_disk()

    robot = build_robot(args.mock)
    if args.home_on_start and not args.mock:
        robot.home()

    app = create_app(robot)
    try:
        uvicorn.run(app, host=args.host, port=args.port)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
