"""Configuration, calibration constants, and persisted tuning values.

This module is pure-Python (no hardware imports) so it can be imported and
unit-tested on any machine.

Hardware background
-------------------
All four servos are driven by a single PCA9685 16-channel PWM board over I2C.
The pulse widths and the ``PWM_FREQUENCY`` value below are inherited verbatim
from the original (working) ``rubik_robot_en_PCA9685.py`` so the mechanical
calibration is preserved exactly. Re-tune at runtime via the ``/tune`` endpoint
rather than editing these defaults.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path

# --------------------------------------------------------------------------- #
# Servo / PCA9685 hardware constants
# --------------------------------------------------------------------------- #

#: PCA9685 I2C address.
PCA9685_ADDRESS = 0x40

#: PWM frequency in Hz. Carried over from the original firmware (300 Hz).
PWM_FREQUENCY = 300

#: The original firmware fed an "on count" of ``duty_ms/1000 * freq * PWM_RES``
#: into a 12-bit (0..4096) PCA9685 register. The 200-count overshoot beyond
#: 4096 is an intentional part of the original servo calibration and is kept so
#: behaviour is identical. See ``hardware/servos.py`` for how this maps onto the
#: modern Adafruit 16-bit ``duty_cycle`` API.
PWM_RES = 4096 + 200
PWM_REG_FULL_SCALE = 4096

#: PCA9685 channel assignments for each servo.
CH_LEFT_WRIST = 0
CH_LEFT_GRIP = 1
CH_RIGHT_WRIST = 2
CH_RIGHT_GRIP = 3

#: 1 = grippers can do a 180 deg wrist turn (requires >= 270 deg servos);
#: 0 = classic 90 deg moves only. Affects the macro move tables in ``moves.py``.
C180 = 1

#: Gripper open/closed travel limits (degrees, before per-servo tuning).
GRIPPER_MAX = 50
GRIPPER_MIN = 0

#: Wrist servo travel limits used for setup sanity only.
TURN_MAX = 270
TURN_MIN = 0

#: Settle delays.
SLEEP_GRIP = 0.3
#: Multiplier applied to the settle delay for a long (0 <-> 180 deg) wrist sweep.
SLEEP_LONG_FACTOR = 2

#: Extra degrees commanded past the target so the servo reaches it under load.
OVERSHOOT = 0

# Per-servo pulse mapping. ``PWM`` is the pulse swing in ms across the full
# travel; ``OFFSET`` is the pulse (ms) at the zero position.
TURN_RANGE_DEG = 180 + C180 * 90
SERVO_PWM_LEFT_GRIP = 2
SERVO_OFFSET_LEFT_GRIP = 0
SERVO_PWM_LEFT_WRIST = 2
SERVO_OFFSET_LEFT_WRIST = 0
SERVO_PWM_RIGHT_GRIP = 2
SERVO_OFFSET_RIGHT_GRIP = 0
SERVO_PWM_RIGHT_WRIST = 2
SERVO_OFFSET_RIGHT_WRIST = 0

# --------------------------------------------------------------------------- #
# Camera / vision constants
# --------------------------------------------------------------------------- #

IMG_WIDTH = 1080
IMG_HEIGHT = 1080

#: Degrees clockwise to rotate each captured frame so a gripped cube appears as
#: an upright 3x3 grid. Set empirically during scan tuning (0/90/180/270).
#: Override at runtime with the RUBIK_CAMERA_ROTATION env var or --rotation flag.
CAMERA_ROTATION = int(os.environ.get("RUBIK_CAMERA_ROTATION", "0"))

#: Pixel rows/columns at which the 9 stickers of a face are sampled.
TOP_ROW_PX = 230
MID_ROW_PX = 500
BOT_ROW_PX = 730
LFT_COL_PX = 230
MID_COL_PX = 450
RGT_COL_PX = 730

#: A neutral patch used for manual white balance.
WB_ROW_PX = 980
WB_COL_PX = 890

#: Sample locations as [row][col] -> (x, y), matching the original layout.
PIXEL_LOCATIONS = [
    [(LFT_COL_PX, TOP_ROW_PX), (MID_COL_PX, TOP_ROW_PX), (RGT_COL_PX, TOP_ROW_PX)],
    [(LFT_COL_PX, MID_ROW_PX), (MID_COL_PX, MID_ROW_PX), (RGT_COL_PX, MID_ROW_PX)],
    [(LFT_COL_PX, BOT_ROW_PX), (MID_COL_PX, BOT_ROW_PX), (RGT_COL_PX, BOT_ROW_PX)],
]

#: A solved cube in kociemba facelet order (U, R, F, D, L, B).
TARGET_STANDARD = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"

# --------------------------------------------------------------------------- #
# Persisted, user-tunable calibration
# --------------------------------------------------------------------------- #


def data_dir() -> Path:
    """Directory where runtime state (calibration) is stored.

    Override with the ``RUBIK_DATA_DIR`` environment variable.
    """
    return Path(os.environ.get("RUBIK_DATA_DIR", Path.home() / ".rubik_robot"))


@dataclass
class Calibration:
    """Runtime-tunable servo offsets and timing.

    These are the values the original firmware adjusted via the physical
    buttons and stored in ``tune_values.txt``. They are now editable over the
    API and persisted as JSON.
    """

    left_grip_tune: int = 0
    left_wrist_tune: int = 0
    right_grip_tune: int = 0
    right_wrist_tune: int = 0
    #: Gripper position at which the cube is released ("load" position).
    load: int = 30
    #: Servo settle delay in seconds.
    sleep: float = 0.5
    #: 1 = re-grip before turning a layer (recommended), 0 = skip.
    regrip: int = 1

    @classmethod
    def path(cls) -> Path:
        return data_dir() / "calibration.json"

    @classmethod
    def from_disk(cls) -> "Calibration":
        p = cls.path()
        if p.exists():
            data = json.loads(p.read_text())
            known = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
            return cls(**known)
        cal = cls()
        cal.save()
        return cal

    def save(self) -> None:
        p = self.path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(asdict(self), indent=2))

    def update(self, **changes) -> "Calibration":
        for key, value in changes.items():
            if key not in self.__dataclass_fields__:
                raise KeyError(f"unknown calibration field: {key}")
            setattr(self, key, value)
        self.save()
        return self
