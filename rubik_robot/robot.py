"""High-level robot controller: servo motion, scanning, and move execution.

This replaces the global-variable soup of the original firmware with a single
stateful object. All hardware access goes through ``_lock`` so the FastAPI
layer can field concurrent requests while the robot still performs one physical
operation at a time.

Servo motion (``_left_wrist`` / ``regrip`` / ``home`` / ``run_action`` ...) is a
direct port of the original ``setDirection_*`` / ``single_action`` functions.
"""

from __future__ import annotations

import threading
import time

from . import config
from .config import Calibration
from .hardware.camera import Camera
from .hardware.servos import ServoDriver
from .moves import MoveTranslator, Orientation
from .vision import get_facelets


class Robot:
    def __init__(
        self,
        driver: ServoDriver,
        camera: Camera,
        calibration: Calibration | None = None,
        c180: int = config.C180,
    ):
        self.driver = driver
        self.camera = camera
        self.cal = calibration or Calibration.from_disk()
        self.c180 = c180
        self.translator = MoveTranslator(c180=c180)

        self._lock = threading.RLock()
        self.l_pos = 90  # current left wrist angle: 0 / 90 / 180
        self.r_pos = 90  # current right wrist angle
        self.remaining_moves = 0  # decremented by the 't' action

    # ------------------------------------------------------------------ #
    # Low-level servo positioning (ported from setDirection_*).
    # ------------------------------------------------------------------ #
    def _left_wrist(self, degrees: float, factor: float) -> None:
        duty = (
            config.SERVO_PWM_LEFT_WRIST / config.TURN_RANGE_DEG * (degrees + config.OVERSHOOT)
            + config.SERVO_OFFSET_LEFT_WRIST
        )
        self.driver.set_pulse_ms(config.CH_LEFT_WRIST, duty)
        time.sleep(self.cal.sleep * factor)

    def _left_grip(self, degrees: float) -> None:
        duty = (
            config.SERVO_PWM_LEFT_GRIP / config.TURN_RANGE_DEG * degrees
            + config.SERVO_OFFSET_LEFT_GRIP
        )
        self.driver.set_pulse_ms(config.CH_LEFT_GRIP, duty)
        time.sleep(config.SLEEP_GRIP)

    def _right_wrist(self, degrees: float, factor: float) -> None:
        duty = (
            config.SERVO_PWM_RIGHT_WRIST / config.TURN_RANGE_DEG * (degrees + config.OVERSHOOT)
            + config.SERVO_OFFSET_RIGHT_WRIST
        )
        self.driver.set_pulse_ms(config.CH_RIGHT_WRIST, duty)
        time.sleep(self.cal.sleep * factor)

    def _right_grip(self, degrees: float) -> None:
        duty = (
            config.SERVO_PWM_RIGHT_GRIP / config.TURN_RANGE_DEG * degrees
            + config.SERVO_OFFSET_RIGHT_GRIP
        )
        self.driver.set_pulse_ms(config.CH_RIGHT_GRIP, duty)
        time.sleep(config.SLEEP_GRIP)

    def regrip(self) -> None:
        """Briefly open both grippers to the load position and re-clamp.

        Reseats the cube before a layer turn. Ported from ``regrip``.
        """
        with self._lock:
            cal = self.cal
            self._left_grip_raw(cal.load + cal.left_grip_tune)
            self._right_grip_raw(cal.load + cal.right_grip_tune)
            time.sleep(cal.sleep / 2)
            self._left_grip_raw(cal.left_grip_tune)
            self._right_grip_raw(cal.right_grip_tune)
            time.sleep(cal.sleep / 2)

    def _left_grip_raw(self, degrees: float) -> None:
        """Set the left gripper without the SLEEP_GRIP settle (regrip helper)."""
        duty = (
            config.SERVO_PWM_LEFT_GRIP / config.TURN_RANGE_DEG * degrees
            + config.SERVO_OFFSET_LEFT_GRIP
        )
        self.driver.set_pulse_ms(config.CH_LEFT_GRIP, duty)

    def _right_grip_raw(self, degrees: float) -> None:
        duty = (
            config.SERVO_PWM_RIGHT_GRIP / config.TURN_RANGE_DEG * degrees
            + config.SERVO_OFFSET_RIGHT_GRIP
        )
        self.driver.set_pulse_ms(config.CH_RIGHT_GRIP, duty)

    # ------------------------------------------------------------------ #
    # Single-character action dispatch (ported from single_action).
    # ------------------------------------------------------------------ #
    def run_action(self, action: str) -> None:
        cal = self.cal
        if action == "A":  # left gripper close
            self._left_grip(cal.left_grip_tune)
        elif action == "a":  # left gripper open
            self._left_grip(config.GRIPPER_MAX + cal.left_grip_tune)
        elif action == "B":  # right gripper close
            self._right_grip(cal.right_grip_tune)
        elif action == "b":  # right gripper open
            self._right_grip(config.GRIPPER_MAX + cal.right_grip_tune)
        elif action == "M":  # left wrist -> 0
            if self.l_pos != 0:
                factor = 1 if self.l_pos == 90 else config.SLEEP_LONG_FACTOR
                self._left_wrist(0 + cal.left_wrist_tune, factor)
                self.l_pos = 0
        elif action == "N":  # left wrist -> 90
            if self.l_pos != 90:
                self._left_wrist(90 + cal.left_wrist_tune, 1)
                self.l_pos = 90
        elif action == "O":  # left wrist -> 180
            if self.l_pos != 180:
                factor = 1 if self.l_pos == 90 else config.SLEEP_LONG_FACTOR
                self._left_wrist(180 + cal.left_wrist_tune, factor)
                self.l_pos = 180
        elif action == "X":  # right wrist -> 0
            if self.r_pos != 0:
                factor = 1 if self.r_pos == 90 else config.SLEEP_LONG_FACTOR
                self._right_wrist(0 + cal.right_wrist_tune, factor)
                self.r_pos = 0
        elif action == "Y":  # right wrist -> 90
            if self.r_pos != 90:
                self._right_wrist(90 + cal.right_wrist_tune, 1)
                self.r_pos = 90
        elif action == "Z":  # right wrist -> 180
            if self.r_pos != 180:
                factor = 1 if self.r_pos == 90 else config.SLEEP_LONG_FACTOR
                self._right_wrist(180 + cal.right_wrist_tune, factor)
                self.r_pos = 180
        elif action == "R":  # re-grip
            if cal.regrip == 1:
                self.regrip()
        elif action == "t":  # progress counter
            self.remaining_moves = max(0, self.remaining_moves - 1)
        else:
            raise ValueError(f"unknown action char: {action!r}")

    def run_actions(self, actions: str) -> None:
        for action in actions:
            self.run_action(action)

    # ------------------------------------------------------------------ #
    # Public, high-level operations.
    # ------------------------------------------------------------------ #
    def home(self) -> None:
        """Return wrists to 90 deg and grippers to the load position."""
        with self._lock:
            cal = self.cal
            self._left_wrist(90 + cal.left_wrist_tune, 1)
            self._right_wrist(90 + cal.right_wrist_tune, 1)
            time.sleep(cal.sleep)
            self.regrip()
            self._left_grip(cal.load + cal.left_grip_tune)
            self._right_grip(cal.load + cal.right_grip_tune)
            time.sleep(cal.sleep)
            self.l_pos = 90
            self.r_pos = 90

    def grip(self) -> None:
        """Close both grippers onto the cube (left/right grip tune positions)."""
        with self._lock:
            self._left_grip(self.cal.left_grip_tune)
            self._right_grip(self.cal.right_grip_tune)

    def move(self, cube_move: str) -> str:
        """Execute one logical cube move (e.g. ``"U'"``). Returns the actions run."""
        with self._lock:
            actions = self.translator.translate(cube_move)
            self.run_actions(actions)
            return actions

    def move_sequence(self, moves: list[str], reset_orientation: bool = False) -> str:
        """Execute a batch of moves with cross-move optimisation.

        Set ``reset_orientation`` when starting from a known home pose (e.g. a
        scramble) rather than continuing after a scan.
        """
        with self._lock:
            if reset_orientation:
                self.translator.reset_orientation()
            self.remaining_moves = len(moves)
            actions = self.translator.translate_batch(moves, optimise_result=True)
            self.run_actions(actions)
            return actions

    def reset_orientation(self) -> None:
        with self._lock:
            self.translator.reset_orientation()

    def scan(self) -> str:
        """Run the physical scan sequence and return the 54-char facelet string.

        Leaves the cube orientation in the post-scan frame so that a kociemba
        solution (computed by the client) can be sent straight to
        :meth:`move_sequence` / :meth:`move`.
        """
        with self._lock:
            images = self._capture_faces()
            facelets = get_facelets(images)
            # The original applied one ``correct_left`` to the solver output
            # before executing it; replicate by leaving the orientation in that
            # frame after a scan.
            self.translator.orientation = Orientation()
            self.translator.orientation.apply("left")
            return facelets

    def _capture_faces(self) -> list:
        """Port of ``get_cube``: rotate the cube and capture all six faces.

        Returns a list indexed 0..5 = U, R, F, D, L, B (kociemba face order),
        matching ``vision.FACE_ORDER``.
        """
        faces: dict[int, object] = {}

        def roll_right_hand():
            # The repeated 6-action motif that re-presents a new face. (b X B a Y A)
            for a in "bXBaYA":
                self.run_action(a)

        self.regrip()
        faces[1] = self.camera.capture()  # R

        roll_right_hand()
        self.regrip()
        faces[2] = self.camera.capture()  # F

        roll_right_hand()
        self.regrip()
        faces[4] = self.camera.capture()  # L

        roll_right_hand()
        self.regrip()
        faces[5] = self.camera.capture()  # B

        for a in "bMBaNA":  # LTm-style move to reach the next face
            self.run_action(a)
        self.regrip()
        roll_right_hand()
        self.regrip()
        faces[3] = self.camera.capture()  # D

        if self.c180 == 1:
            for a in "bXBaZAbYB":
                self.run_action(a)
        else:
            roll_right_hand()
            roll_right_hand()
        self.regrip()
        faces[0] = self.camera.capture()  # U

        return [faces[i] for i in range(6)]

    # ------------------------------------------------------------------ #
    # Calibration.
    # ------------------------------------------------------------------ #
    def update_calibration(self, **changes) -> Calibration:
        with self._lock:
            return self.cal.update(**changes)

    def close(self) -> None:
        with self._lock:
            self.driver.close()
            self.camera.close()
