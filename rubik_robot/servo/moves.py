"""
Low-level servo movement functions.

This module translates logical robot actions (grip, turn, regrip) into
hardware commands via the ServoDriver interface. It also defines the
macro move functions that convert high-level cube operations into
sequences of single-character action codes.

Action code reference (used in action strings):
    A  - Left gripper close
    a  - Left gripper open
    B  - Right gripper close
    b  - Right gripper open
    M  - Left wrist to 0 degrees
    N  - Left wrist to 90 degrees
    O  - Left wrist to 180 degrees
    X  - Right wrist to 0 degrees
    Y  - Right wrist to 90 degrees
    Z  - Right wrist to 180 degrees
    R  - Regrip (release and re-grab the cube)
    t  - Decrement move counter

These action codes are concatenated into strings that represent complete
movements. For example, "bXBaYA" means: open right grip, right wrist
to 0, close right grip, open left grip, left wrist to 90, close left grip.
"""

import time


class ServoState:
    """Tracks the current position of both wrist servos.

    The robot needs to know current positions to determine whether a
    move is a 90-degree or 180-degree rotation (which affects timing).

    Attributes:
        l_pos: Current left wrist position in degrees (0, 90, or 180).
        r_pos: Current right wrist position in degrees (0, 90, or 180).
        moves: Move counter, decremented by the 't' action code.
    """

    def __init__(self):
        self.l_pos = 90
        self.r_pos = 90
        self.moves = 0


def _compute_duty(servo_config, direction, overshoot=0):
    """Compute the PWM duty value for a given servo angle.

    The duty value is calculated as a linear interpolation between
    the servo's offset (0 degrees) and offset + pwm_range (max degrees).

    Args:
        servo_config: ServoConfig with turn_max, pwm_range, and offset.
        direction: Target angle in degrees.
        overshoot: Extra degrees to add (compensates for mechanical friction).

    Returns:
        The duty value to send to the hardware driver.
    """
    return (servo_config.pwm_range / servo_config.turn_max
            * (direction + overshoot)
            + servo_config.offset)


# ---------------------------------------------------------------------------
# Directional servo commands
# ---------------------------------------------------------------------------

def set_left_turn(driver, config, cal, direction, sleep_factor):
    """Move the left wrist servo to the specified angle.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with wrist tune offset.
        direction: Target angle in degrees (including tune offset).
        sleep_factor: Multiplier for the settling delay (1 for 90-degree
            moves, sleep_long_factor for 180-degree moves).
    """
    duty = _compute_duty(config.left_turn, direction + config.overshoot)
    driver.set_left_turn(duty)
    time.sleep(cal.sleep * sleep_factor)


def set_left_grip(driver, config, cal, direction):
    """Move the left gripper servo to the specified angle.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues (not used for grip duty, but sleep_grip
            comes from config).
        direction: Target angle in degrees.
    """
    duty = _compute_duty(config.left_grip, direction)
    driver.set_left_grip(duty)
    time.sleep(config.sleep_grip)


def set_right_turn(driver, config, cal, direction, sleep_factor):
    """Move the right wrist servo to the specified angle.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with wrist tune offset.
        direction: Target angle in degrees (including tune offset).
        sleep_factor: Multiplier for the settling delay.
    """
    duty = _compute_duty(config.right_turn, direction + config.overshoot)
    driver.set_right_turn(duty)
    time.sleep(cal.sleep * sleep_factor)


def set_right_grip(driver, config, cal, direction):
    """Move the right gripper servo to the specified angle.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues (not used for grip duty).
        direction: Target angle in degrees.
    """
    duty = _compute_duty(config.right_grip, direction)
    driver.set_right_grip(duty)
    time.sleep(config.sleep_grip)


# ---------------------------------------------------------------------------
# Compound servo commands
# ---------------------------------------------------------------------------

def regrip(driver, config, cal):
    """Release and re-grab the cube to ensure a solid grip.

    The sequence is:
    1. Both grippers go to LOAD position (partially open)
    2. Wait half the settling time
    3. Both grippers close fully
    4. Wait half the settling time

    This ensures the cube is centered and firmly held.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with grip tune offsets and load position.
    """
    # Open to load position
    duty = _compute_duty(config.left_grip, cal.load + cal.left_grip_tune)
    driver.set_left_grip(duty)
    duty = _compute_duty(config.right_grip, cal.load + cal.right_grip_tune)
    driver.set_right_grip(duty)
    time.sleep(cal.sleep / 2)

    # Close fully
    duty = _compute_duty(config.left_grip, cal.left_grip_tune)
    driver.set_left_grip(duty)
    duty = _compute_duty(config.right_grip, cal.right_grip_tune)
    driver.set_right_grip(duty)
    time.sleep(cal.sleep / 2)


def home_servos(driver, config, cal, state):
    """Return all servos to their home (neutral) position.

    Home position is: both wrists at 90 degrees, both grippers at
    the load (release) position.

    Args:
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with tune offsets.
        state: ServoState to update with new positions.
    """
    set_left_turn(driver, config, cal, 90 + cal.left_wrist_tune, 1)
    set_right_turn(driver, config, cal, 90 + cal.right_wrist_tune, 1)
    time.sleep(cal.sleep)
    regrip(driver, config, cal)
    set_left_grip(driver, config, cal, cal.load + cal.left_grip_tune)
    set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)
    time.sleep(cal.sleep)
    state.l_pos = 90
    state.r_pos = 90


# ---------------------------------------------------------------------------
# Single action dispatcher
# ---------------------------------------------------------------------------

def single_action(action, driver, config, cal, state):
    """Execute a single action code on the robot hardware.

    This is the core dispatcher that translates one-character action
    codes into servo movements. Action strings (e.g., "bXBaYA") are
    executed by calling this function for each character.

    Args:
        action: Single character action code (see module docstring).
        driver: ServoDriver instance.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with tune offsets.
        state: ServoState tracking current positions.
    """
    if action == "A":
        # Left gripper close
        set_left_grip(driver, config, cal, cal.left_grip_tune)

    elif action == "a":
        # Left gripper open
        if config.grip_adds_tune_on_open:
            set_left_grip(driver, config, cal,
                          config.gripper_max + cal.left_grip_tune)
        else:
            set_left_grip(driver, config, cal, config.gripper_max)

    elif action == "B":
        # Right gripper close
        set_right_grip(driver, config, cal, cal.right_grip_tune)

    elif action == "b":
        # Right gripper open
        if config.grip_adds_tune_on_open:
            set_right_grip(driver, config, cal,
                           config.gripper_max + cal.right_grip_tune)
        else:
            set_right_grip(driver, config, cal, config.gripper_max)

    elif action == "M":
        # Left wrist to 0 degrees
        if state.l_pos != 0:
            sleep = 1 if state.l_pos == 90 else config.sleep_long_factor
            set_left_turn(driver, config, cal,
                          0 + cal.left_wrist_tune, sleep)
            state.l_pos = 0

    elif action == "N":
        # Left wrist to 90 degrees
        if state.l_pos != 90:
            set_left_turn(driver, config, cal,
                          90 + cal.left_wrist_tune, 1)
            state.l_pos = 90

    elif action == "O":
        # Left wrist to 180 degrees
        if state.l_pos != 180:
            sleep = 1 if state.l_pos == 90 else config.sleep_long_factor
            set_left_turn(driver, config, cal,
                          180 + cal.left_wrist_tune, sleep)
            state.l_pos = 180

    elif action == "X":
        # Right wrist to 0 degrees
        if state.r_pos != 0:
            sleep = 1 if state.r_pos == 90 else config.sleep_long_factor
            set_right_turn(driver, config, cal,
                           0 + cal.right_wrist_tune, sleep)
            state.r_pos = 0

    elif action == "Y":
        # Right wrist to 90 degrees
        if state.r_pos != 90:
            set_right_turn(driver, config, cal,
                           90 + cal.right_wrist_tune, 1)
            state.r_pos = 90

    elif action == "Z":
        # Right wrist to 180 degrees
        if state.r_pos != 180:
            sleep = 1 if state.r_pos == 90 else config.sleep_long_factor
            set_right_turn(driver, config, cal,
                           180 + cal.right_wrist_tune, sleep)
            state.r_pos = 180

    elif action == "R":
        # Regrip (if enabled)
        if cal.regrip_enabled:
            regrip(driver, config, cal)

    elif action == "t":
        # Decrement move counter
        state.moves -= 1


# ---------------------------------------------------------------------------
# Macro move functions
#
# These return action code strings for common cube operations. The
# sequences differ depending on whether the robot has 180-degree wrist
# capability (c180=True) or only 90-degree (c180=False).
#
# Naming convention:
#   L/R = Left/Right arm
#   M   = Move (rotate the cube layer connected to this arm's grip)
#   T   = Turn (rotate the whole cube by turning the other arm's layer)
#   p   = Plus (clockwise 90 degrees)
#   m   = Minus (counter-clockwise 90 degrees)
#   pp  = Plus-plus (180 degrees)
# ---------------------------------------------------------------------------

def LMp(c180):
    """Left arm move, clockwise 90 degrees."""
    return "OtaNA" if c180 else "aMANt"

def LMm(c180):
    """Left arm move, counter-clockwise 90 degrees."""
    return "MtaNA"

def LMpp(c180):
    """Left arm move, 180 degrees."""
    return "aMAOtaNA" if c180 else "aMANaMANt"

def LTp(c180):
    """Left arm cube turn, clockwise 90 degrees."""
    return "bOBaNA" if c180 else "aMAbNB"

def LTm(c180):
    """Left arm cube turn, counter-clockwise 90 degrees."""
    return "bMBaNA"

def LTpp(c180):
    """Left arm cube turn, 180 degrees."""
    return "aMAbOBaNA" if c180 else "aMAbNBaMAbNB"

def RMp(c180):
    """Right arm move, clockwise 90 degrees."""
    return "ZtbYB" if c180 else "bXBYt"

def RMm(c180):
    """Right arm move, counter-clockwise 90 degrees."""
    return "XtbYB"

def RMpp(c180):
    """Right arm move, 180 degrees."""
    return "bXBZtbYB" if c180 else "bXBYbXBYt"

def RTp(c180):
    """Right arm cube turn, clockwise 90 degrees."""
    return "aZAbYB" if c180 else "bXBaYA"

def RTm(c180):
    """Right arm cube turn, counter-clockwise 90 degrees."""
    return "aXAbYB"

def RTpp(c180):
    """Right arm cube turn, 180 degrees."""
    return "bXBaZAbYB" if c180 else "bXBaYAbXBaYA"
