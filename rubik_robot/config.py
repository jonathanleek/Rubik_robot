"""
Configuration for the Rubik Robot.

This module defines all hardware and software constants for both GPIO and
PCA9685 driver variants. Each variant has its own HardwareConfig with
driver-specific defaults for servo tuning, PWM parameters, and pixel
detection coordinates.

To add a new hardware variant, subclass HardwareConfig and override the
relevant constants.
"""

import os
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Home directory -- all runtime files (images, calibration) live here
# ---------------------------------------------------------------------------
HOME = "/home/pi/"

# ---------------------------------------------------------------------------
# Image capture settings (shared by both variants)
# ---------------------------------------------------------------------------
IMG_WIDTH = 1080
IMG_HEIGHT = 1080

# ---------------------------------------------------------------------------
# Cube state representing a solved cube (Kociemba URFDLB notation)
# ---------------------------------------------------------------------------
TARGET_STANDARD = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"

# ---------------------------------------------------------------------------
# Maximum number of scramble moves
# ---------------------------------------------------------------------------
SCRAMBLE_MAX = 20


@dataclass
class PixelLocations:
    """Pixel coordinates used to sample cube face colors from camera images.

    Each face of the cube is divided into a 3x3 grid. These coordinates
    define where to sample the color of each sticker in the captured image.
    The white-balance reference area is sampled separately.
    """

    # Row pixel positions (top, middle, bottom of the 3x3 grid)
    top_row: int = 250
    mid_row: int = 500
    bot_row: int = 750

    # Column pixel positions (left, middle, right of the 3x3 grid)
    lft_col: int = 200
    mid_col: int = 450
    rgt_col: int = 700

    # White-balance reference area (a neutral-colored region in the image)
    wb_row: int = 980
    wb_col: int = 890

    def get_grid(self):
        """Return a 3x3 list of (x, y) tuples for sampling sticker colors.

        The grid is indexed as grid[row][col], where each entry is the
        (x, y) pixel coordinate to sample.
        """
        rows = [self.top_row, self.mid_row, self.bot_row]
        cols = [self.lft_col, self.mid_col, self.rgt_col]
        return [[(c, r) for c in cols] for r in rows]


@dataclass
class ServoConfig:
    """PWM configuration for a single servo.

    Attributes:
        turn_max: Maximum rotation angle of the servo in degrees.
        pwm_range: The range of the PWM signal. For GPIO this is a
            percentage of the PWM period; for PCA9685 it is in milliseconds.
        offset: The baseline PWM value corresponding to 0 degrees.
    """
    turn_max: int = 180
    pwm_range: float = 10.0
    offset: float = 2.0


@dataclass
class HardwareConfig:
    """Complete hardware configuration for one driver variant.

    This dataclass holds every tunable constant that differs between the
    GPIO and PCA9685 hardware variants. The defaults here match the GPIO
    variant; the PCA9685 variant overrides them via pca9685_config().

    Attributes:
        c180: If True, servos support 180-degree wrist rotation (requires
            270-degree servos). If False, wrist rotation is limited to 90
            degrees. This fundamentally changes the macro move sequences.
        gripper_max: Maximum gripper opening angle in degrees.
        gripper_min: Minimum gripper angle (fully closed).
        turn_max: Maximum wrist rotation angle (used as a setup limit).
        turn_min: Minimum wrist rotation angle.
        sleep_grip: Delay (seconds) after a gripper move to let it settle.
        sleep_long_factor: Multiplier for delay on long (180-degree) wrist moves.
        overshoot: Extra degrees added to wrist turns to overcome friction.
            GPIO variant uses 5 to compensate for weaker PWM signal;
            PCA9685 uses 0 because the dedicated driver is more precise.
        pwm_freq: PWM signal frequency in Hz.
        grip_adds_tune_on_open: Whether to add the grip tune offset when
            opening the gripper. The PCA9685 variant does this; GPIO does not.
        regrip_before_r_moves: Whether to prepend a regrip action before
            R and D face moves. The GPIO variant does this; PCA9685 does not.
        left_grip: ServoConfig for the left gripper servo.
        left_turn: ServoConfig for the left wrist (turn) servo.
        right_grip: ServoConfig for the right gripper servo.
        right_turn: ServoConfig for the right wrist (turn) servo.
        pixels: PixelLocations for cube face color detection.
    """

    # --- Mechanical parameters ---
    c180: bool = False
    gripper_max: int = 65
    gripper_min: int = 0
    turn_max: int = 270
    turn_min: int = 0
    sleep_grip: float = 0.3
    sleep_long_factor: float = 2.0
    overshoot: int = 5

    # --- PWM parameters ---
    pwm_freq: int = 50

    # --- Behavioral flags ---
    # PCA9685 adds tune offset when opening gripper; GPIO does not
    grip_adds_tune_on_open: bool = False
    # GPIO prepends regrip before R/D moves; PCA9685 does not
    regrip_before_r_moves: bool = True

    # --- Per-servo PWM configuration ---
    left_grip: ServoConfig = field(default_factory=lambda: ServoConfig(180, 10.0, 2.0))
    left_turn: ServoConfig = field(default_factory=lambda: ServoConfig(180, 10.0, 2.0))
    right_grip: ServoConfig = field(default_factory=lambda: ServoConfig(180, 10.0, 2.0))
    right_turn: ServoConfig = field(default_factory=lambda: ServoConfig(180, 10.0, 2.0))

    # --- Camera pixel locations ---
    pixels: PixelLocations = field(default_factory=PixelLocations)


def gpio_config():
    """Return the default HardwareConfig for the GPIO PWM variant.

    This variant uses the Raspberry Pi's built-in GPIO pins to generate
    PWM signals directly. It uses 90-degree wrist rotation and adds a
    small overshoot to compensate for less precise PWM timing.
    """
    return HardwareConfig(
        c180=False,
        gripper_max=65,
        overshoot=5,
        pwm_freq=50,
        grip_adds_tune_on_open=False,
        regrip_before_r_moves=True,
        left_grip=ServoConfig(180, 10.0, 2.0),
        left_turn=ServoConfig(180, 10.0, 2.0),
        right_grip=ServoConfig(180, 10.0, 2.0),
        right_turn=ServoConfig(180, 10.0, 2.0),
        pixels=PixelLocations(
            top_row=250, mid_row=500, bot_row=750,
            lft_col=200, mid_col=450, rgt_col=700,
            wb_row=980, wb_col=890,
        ),
    )


def pca9685_config():
    """Return the default HardwareConfig for the PCA9685 PWM driver variant.

    This variant uses an Adafruit PCA9685 I2C PWM driver for more stable
    and jitter-free servo control. It supports 180-degree wrist rotation
    (requiring 270-degree servos) and does not need overshoot compensation.

    The PWM range and offset values are in milliseconds (the PCA9685 driver
    converts these to register ticks internally).
    """
    return HardwareConfig(
        c180=True,
        gripper_max=50,
        overshoot=0,
        pwm_freq=300,
        grip_adds_tune_on_open=True,
        regrip_before_r_moves=False,
        # offset is the minimum pulse width in ms (DS3218: ~0.5 ms = 0 deg,
        # ~2.5 ms = 270 deg). offset was previously 0.0, which produced
        # sub-0.5 ms pulses for low angles (e.g. the 0-50 deg gripper range),
        # too short for the servo to position -- so grippers stayed pinned.
        left_grip=ServoConfig(270, 2.0, 0.5),
        left_turn=ServoConfig(270, 2.0, 0.5),
        right_grip=ServoConfig(270, 2.0, 0.5),
        right_turn=ServoConfig(270, 2.0, 0.5),
        pixels=PixelLocations(
            top_row=230, mid_row=500, bot_row=730,
            lft_col=230, mid_col=450, rgt_col=730,
            wb_row=980, wb_col=890,
        ),
    )


@dataclass
class CalibrationValues:
    """Runtime calibration offsets, loaded from and saved to tune_values.txt.

    These values are adjusted during the servo calibration process (either
    via physical buttons or the /calibration API endpoint) and persist
    across restarts.

    Attributes:
        left_grip_tune: Offset in degrees for the left gripper servo.
        left_wrist_tune: Offset in degrees for the left wrist servo.
        right_grip_tune: Offset in degrees for the right gripper servo.
        right_wrist_tune: Offset in degrees for the right wrist servo.
        load: The gripper angle at which the cube is released (load position).
        sleep: Delay in seconds after each servo move to allow settling.
        regrip_enabled: Whether to perform a regrip before layer moves.
            Can be disabled if the robot's grip is tight enough.
    """
    left_grip_tune: int = 0
    left_wrist_tune: int = 0
    right_grip_tune: int = 0
    right_wrist_tune: int = 0
    load: int = 30
    sleep: float = 0.5
    regrip_enabled: bool = True

    def save(self, path=None):
        """Save calibration values to a text file.

        Each value is written on its own line in a fixed order. The file
        is overwritten completely on each save.

        Args:
            path: File path to write to. Defaults to ~/tune_values.txt.
        """
        if path is None:
            path = os.path.join(HOME, "tune_values.txt")
        with open(path, "w") as f:
            f.write(str(self.left_grip_tune) + "\n")
            f.write(str(self.left_wrist_tune) + "\n")
            f.write(str(self.right_grip_tune) + "\n")
            f.write(str(self.right_wrist_tune) + "\n")
            f.write(str(self.load) + "\n")
            f.write(str(self.sleep) + "\n")
            f.write(str(1 if self.regrip_enabled else 0) + "\n")

    @classmethod
    def load_from_file(cls, path=None):
        """Load calibration values from a text file.

        If the file does not exist, returns default values and creates
        the file with those defaults.

        Args:
            path: File path to read from. Defaults to ~/tune_values.txt.

        Returns:
            A CalibrationValues instance with the loaded (or default) values.
        """
        if path is None:
            path = os.path.join(HOME, "tune_values.txt")
        if os.path.exists(path):
            with open(path, "r") as f:
                lines = f.readlines()
            cal = cls(
                left_grip_tune=int(lines[0].strip()),
                left_wrist_tune=int(lines[1].strip()),
                right_grip_tune=int(lines[2].strip()),
                right_wrist_tune=int(lines[3].strip()),
                load=int(lines[4].strip()),
                sleep=float(lines[5].strip()),
                regrip_enabled=int(lines[6].strip()) == 1,
            )
            return cal
        else:
            # First run: create defaults and save them
            cal = cls()
            cal.save(path)
            return cal
