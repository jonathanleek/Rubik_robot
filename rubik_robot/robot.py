"""
Robot controller - the central orchestrator.

This module ties together all hardware and logic layers into a single
RobotController class. It manages:
    - Hardware initialization and shutdown
    - Thread-safe access to servos and camera via a threading lock
    - High-level operations: scan, move, scramble, calibrate
    - Robot state tracking (idle, scanning, moving, etc.)

All API endpoints delegate to RobotController methods. The threading
lock ensures that only one operation can access the hardware at a time.
If a request arrives while the robot is busy, the caller receives a
"busy" status rather than blocking.
"""

import random
import threading
import time
import atexit

from rubik_robot.config import (
    CalibrationValues, HardwareConfig,
    gpio_config, pca9685_config,
    SCRAMBLE_MAX,
)
from rubik_robot.hardware.gpio_driver import GPIODriver
from rubik_robot.hardware.pca9685_driver import PCA9685Driver
from rubik_robot.hardware.display import OLEDDisplay, NullDisplay
from rubik_robot.servo.moves import (
    ServoState, single_action, regrip, home_servos,
    set_left_grip, set_right_grip,
    set_left_turn, set_right_turn,
)
from rubik_robot.servo.solver import (
    create_master_string, correct_left,
)
from rubik_robot.scanner.camera import Camera
from rubik_robot.scanner.color import get_sticker
from rubik_robot.scanner.cube_reader import get_cube


# Valid robot states
STATUS_IDLE = "idle"
STATUS_SCANNING = "scanning"
STATUS_MOVING = "moving"
STATUS_SCRAMBLING = "scrambling"
STATUS_CALIBRATING = "calibrating"
STATUS_INITIALIZING = "initializing"


class RobotController:
    """Central controller for the Rubik's Cube solving robot.

    This class is the single point of coordination for all robot
    operations. It holds references to the hardware driver, camera,
    display, calibration values, and servo state. All public methods
    are thread-safe via an internal lock.

    The controller supports two hardware variants:
    - "gpio": Basic Raspberry Pi GPIO PWM (no display, no buttons)
    - "pca9685": PCA9685 I2C PWM driver with OLED display and buttons

    Usage:
        robot = RobotController(driver_type="pca9685")
        robot.initialize()
        state = robot.scan()       # Returns cube state string
        robot.execute_moves(["R", "U'", "F2"])
        robot.shutdown()

    Attributes:
        driver: ServoDriver instance (GPIODriver or PCA9685Driver).
        config: HardwareConfig with hardware-specific constants.
        calibration: CalibrationValues loaded from tune_values.txt.
        display: OLEDDisplay or NullDisplay for status output.
        servo_state: ServoState tracking current wrist positions.
        status: Current robot state string (one of the STATUS_* constants).
    """

    def __init__(self, driver_type="pca9685"):
        """Create a new RobotController.

        Does not initialize hardware -- call initialize() separately.

        Args:
            driver_type: "gpio" for GPIO PWM or "pca9685" for PCA9685
                I2C PWM driver. Determines which driver, config, and
                peripherals (display, buttons) are used.
        """
        self._lock = threading.Lock()
        self._driver_type = driver_type
        self.status = STATUS_INITIALIZING

        # Set up config and driver based on variant
        if driver_type == "gpio":
            self.config = gpio_config()
            self.driver = GPIODriver(pwm_freq=self.config.pwm_freq)
            self.display = NullDisplay()
        else:
            self.config = pca9685_config()
            self.driver = PCA9685Driver(pwm_freq=self.config.pwm_freq)
            try:
                self.display = OLEDDisplay()
            except Exception:
                # If OLED initialization fails, fall back to null display
                self.display = NullDisplay()

        self.calibration = CalibrationValues()
        self.servo_state = ServoState()
        self.camera = Camera()
        self._calibration_manager = None

    def initialize(self):
        """Initialize all hardware: servos, camera, buttons (if applicable).

        Loads calibration values from disk, sets up servo PWM, initializes
        the camera, and (for PCA9685) sets up physical calibration buttons.

        Registers a shutdown handler via atexit to ensure hardware is
        cleaned up even if the program exits unexpectedly.
        """
        self.display.show("Init", "")

        # Load saved calibration values
        self.calibration = CalibrationValues.load_from_file()

        # Initialize servo hardware
        self.driver.setup()
        time.sleep(self.calibration.sleep)

        # Initialize camera
        self.camera.setup()

        # Set up physical buttons for PCA9685 variant
        if self._driver_type == "pca9685":
            from rubik_robot.servo.calibration import CalibrationManager
            self._calibration_manager = CalibrationManager(self)
            self._calibration_manager.setup_buttons()

        # Home all servos
        home_servos(self.driver, self.config, self.calibration, self.servo_state)

        # Register cleanup handler
        atexit.register(self.shutdown)

        self.status = STATUS_IDLE
        self.display.show("Ready", "")

    def shutdown(self):
        """Shut down all hardware and release resources.

        Homes the servos, closes the camera, and releases GPIO pins.
        Safe to call multiple times.
        """
        try:
            home_servos(self.driver, self.config, self.calibration, self.servo_state)
        except Exception:
            pass
        try:
            self.camera.close()
        except Exception:
            pass
        try:
            self.driver.cleanup()
        except Exception:
            pass

    def acquire_lock(self):
        """Try to acquire the hardware lock (non-blocking).

        Returns:
            True if the lock was acquired, False if the robot is busy.
        """
        return self._lock.acquire(blocking=False)

    def release_lock(self):
        """Release the hardware lock."""
        self._lock.release()

    # -------------------------------------------------------------------
    # High-level operations
    # -------------------------------------------------------------------

    def scan(self):
        """Scan the cube and return its current state.

        Photographs all six faces, analyzes the colors, and returns
        the 54-character state string in Kociemba URFDLB notation.

        Returns:
            dict with:
                - "state": 54-character cube state string
                - "time": scan duration in seconds

        Raises:
            RuntimeError: If the scanned state is invalid (wrong number
                of colors per face, etc.).
        """
        self.status = STATUS_SCANNING
        self.display.show("Scanning", "")
        start_time = time.time()

        # Close grippers to hold the cube
        set_left_grip(self.driver, self.config, self.calibration,
                      self.calibration.left_grip_tune)
        set_right_grip(self.driver, self.config, self.calibration,
                       self.calibration.right_grip_tune)

        # Photograph all six faces
        get_cube(self.driver, self.config, self.calibration,
                 self.servo_state, self.camera)

        self.display.show("Analyzing", "")

        # Analyze colors to determine cube state
        state_string = get_sticker(self.config.pixels)

        # Validate the state string
        if len(state_string) != 54:
            raise RuntimeError(
                f"Invalid scan: expected 54 stickers, got {len(state_string)}"
            )

        # Check that each face color appears exactly 9 times
        for face in "URFDLB":
            count = state_string.count(face)
            if count != 9:
                raise RuntimeError(
                    f"Invalid scan: face {face} has {count} stickers (expected 9)"
                )

        elapsed = round(time.time() - start_time, 2)

        # Return to home position
        home_servos(self.driver, self.config, self.calibration, self.servo_state)

        self.status = STATUS_IDLE
        self.display.show("Scanned", f"{elapsed}s")

        return {"state": state_string, "time": elapsed}

    def execute_moves(self, moves):
        """Execute a list of Rubik's notation moves on the physical cube.

        Translates the moves into servo action codes, optimizes the
        sequence, and executes it.

        Args:
            moves: List of move strings in Rubik's notation
                (e.g., ["R", "U'", "F2"]).

        Returns:
            dict with:
                - "moves_executed": number of moves performed
                - "time": execution duration in seconds
        """
        self.status = STATUS_MOVING
        self.display.show("Moving", f"{len(moves)} moves")
        start_time = time.time()

        # Make a copy since create_master_string mutates the array
        solve_array = list(moves)

        # Apply initial orientation correction
        correct_left(solve_array)

        # Translate moves into optimized action string
        self.servo_state.moves = len(solve_array)
        sequence = create_master_string(
            solve_array,
            self.config.c180,
            self.config.regrip_before_r_moves,
        )

        # Execute the action sequence
        regrip(self.driver, self.config, self.calibration)
        for action in sequence:
            single_action(action, self.driver, self.config,
                          self.calibration, self.servo_state)

        elapsed = round(time.time() - start_time, 2)

        # Return to home position
        home_servos(self.driver, self.config, self.calibration, self.servo_state)

        self.status = STATUS_IDLE
        self.display.show("Done", f"{len(moves)} moves")

        return {"moves_executed": len(moves), "time": elapsed}

    def home(self):
        """Return all servos to their home (neutral) position.

        Returns:
            dict with "status": "homed"
        """
        home_servos(self.driver, self.config, self.calibration, self.servo_state)
        self.display.show("Ready", "")
        return {"status": "homed"}

    def scramble(self, count):
        """Scramble the cube with a random sequence of moves.

        Generates a random sequence of non-repeating-layer moves and
        executes them on the physical cube.

        Args:
            count: Number of random moves to perform (1 to SCRAMBLE_MAX).

        Returns:
            dict with:
                - "moves": list of moves performed
                - "count": number of moves
                - "time": execution duration in seconds
        """
        count = max(1, min(count, SCRAMBLE_MAX))

        self.status = STATUS_SCRAMBLING
        self.display.show("Scramble", f"{count} moves")
        start_time = time.time()

        # Generate random non-repeating-layer move sequence
        pos_moves = [
            ["U", "U2", "U'"],
            ["F", "F2", "F'"],
            ["R", "R2", "R'"],
            ["L", "L2", "L'"],
            ["B", "B2", "B'"],
            ["D", "D2", "D'"],
        ]

        moves = []
        last_layer = -1
        for _ in range(count):
            layer = last_layer
            while layer == last_layer:
                layer = random.randint(0, 5)
            last_layer = layer
            moves.append(pos_moves[layer][random.randint(0, 2)])

        # Execute the scramble
        solve_array = list(moves)
        self.servo_state.moves = len(solve_array)
        sequence = create_master_string(
            solve_array,
            self.config.c180,
            self.config.regrip_before_r_moves,
        )

        regrip(self.driver, self.config, self.calibration)
        for action in sequence:
            single_action(action, self.driver, self.config,
                          self.calibration, self.servo_state)

        elapsed = round(time.time() - start_time, 2)

        home_servos(self.driver, self.config, self.calibration, self.servo_state)

        self.status = STATUS_IDLE
        self.display.show("Scrambled", f"{count} moves")

        return {"moves": moves, "count": count, "time": elapsed}

    def get_status(self):
        """Get the current robot status and calibration values.

        This method does NOT acquire the hardware lock, so it can be
        called even while the robot is busy performing another operation.

        Returns:
            dict with:
                - "status": current state string (idle, scanning, etc.)
                - "calibration": dict of current calibration values
                - "driver": driver type string (gpio or pca9685)
        """
        cal = self.calibration
        return {
            "status": self.status,
            "driver": self._driver_type,
            "calibration": {
                "left_grip_tune": cal.left_grip_tune,
                "left_wrist_tune": cal.left_wrist_tune,
                "right_grip_tune": cal.right_grip_tune,
                "right_wrist_tune": cal.right_wrist_tune,
                "load": cal.load,
                "sleep": cal.sleep,
                "regrip_enabled": cal.regrip_enabled,
            },
        }

    def get_calibration(self):
        """Get current calibration values.

        Returns:
            dict with all calibration values.
        """
        cal = self.calibration
        return {
            "left_grip_tune": cal.left_grip_tune,
            "left_wrist_tune": cal.left_wrist_tune,
            "right_grip_tune": cal.right_grip_tune,
            "right_wrist_tune": cal.right_wrist_tune,
            "load": cal.load,
            "sleep": cal.sleep,
            "regrip_enabled": cal.regrip_enabled,
        }

    def set_calibration(self, values):
        """Update calibration values and save to disk.

        Only the keys present in the values dict are updated; other
        calibration values are left unchanged.

        Args:
            values: dict with any of the CalibrationValues fields
                (e.g., {"left_grip_tune": 4, "sleep": 0.45}).

        Returns:
            dict with the updated calibration values.
        """
        cal = self.calibration

        if "left_grip_tune" in values:
            cal.left_grip_tune = int(values["left_grip_tune"])
        if "left_wrist_tune" in values:
            cal.left_wrist_tune = int(values["left_wrist_tune"])
        if "right_grip_tune" in values:
            cal.right_grip_tune = int(values["right_grip_tune"])
        if "right_wrist_tune" in values:
            cal.right_wrist_tune = int(values["right_wrist_tune"])
        if "load" in values:
            cal.load = int(values["load"])
        if "sleep" in values:
            cal.sleep = float(values["sleep"])
        if "regrip_enabled" in values:
            cal.regrip_enabled = bool(values["regrip_enabled"])

        cal.save()
        self.display.show("Cal saved", "")

        return self.get_calibration()

    def test_servo(self, servo_name, value):
        """Move a single servo to a specific position for testing.

        This is used during calibration to test individual servo
        positions without going through the full calibration sequence.

        Args:
            servo_name: One of "left_grip", "left_turn", "right_grip",
                "right_turn".
            value: Target angle in degrees.

        Returns:
            dict with the servo name and value that was set.

        Raises:
            ValueError: If servo_name is not recognized.
        """
        if servo_name == "left_grip":
            set_left_grip(self.driver, self.config, self.calibration, value)
        elif servo_name == "left_turn":
            set_left_turn(self.driver, self.config, self.calibration, value, 1)
        elif servo_name == "right_grip":
            set_right_grip(self.driver, self.config, self.calibration, value)
        elif servo_name == "right_turn":
            set_right_turn(self.driver, self.config, self.calibration, value, 1)
        else:
            raise ValueError(
                f"Unknown servo: {servo_name}. "
                f"Must be one of: left_grip, left_turn, right_grip, right_turn"
            )

        self.display.show(f"Test", f"{servo_name}={value}")

        return {"servo": servo_name, "value": value}
