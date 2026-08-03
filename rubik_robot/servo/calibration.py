"""
Servo calibration via physical buttons.

Manages the servo calibration process using three physical buttons
(plus, minus, enter) and an optional OLED display. Only used by the
PCA9685 variant; the GPIO variant performs calibration via the API.

Calibration steps (navigated with Enter button):
    1. Left gripper tune  - Adjust with +/- until gripper closes squarely
    2. Left wrist tune    - Adjust with +/- until wrist is centered at 90
    3. Right gripper tune - Same as step 1 for right side
    4. Right wrist tune   - Same as step 2 for right side
    5. Load position      - Adjust the release/load angle for both grippers
    6. Sleep delay        - Adjust servo settling delay (0.00 - 1.00 seconds)
    7. Regrip toggle      - Enable or disable regrip between moves

After completing all steps (pressing Enter on step 7), the calibration
values are saved to ~/tune_values.txt and the robot returns to idle.

The calibration state machine uses a threading lock to prevent API
requests from interfering with calibration. While calibrating, API
endpoints will return HTTP 409 (Conflict).
"""

from rubik_robot.hardware.buttons import (
    Button, BUTTON_PRESSED, BUTTON_DOUBLECLICKED,
    PLUS_BUTTON_PIN, MINUS_BUTTON_PIN, ENTER_BUTTON_PIN,
)
from rubik_robot.servo.moves import (
    set_left_turn, set_left_grip, set_right_turn, set_right_grip,
    home_servos,
)


# Calibration states
CAL_IDLE = 0
CAL_LEFT_GRIP = 10
CAL_LEFT_WRIST = 20
CAL_RIGHT_GRIP = 30
CAL_RIGHT_WRIST = 40
CAL_LOAD = 50
CAL_SLEEP = 60
CAL_REGRIP = 65

# Display labels for each calibration step
CAL_LABELS = {
    CAL_LEFT_GRIP: "L. Grip",
    CAL_LEFT_WRIST: "L. Turn",
    CAL_RIGHT_GRIP: "R. Grip",
    CAL_RIGHT_WRIST: "R. Turn",
    CAL_LOAD: "Load",
    CAL_SLEEP: "Delay",
    CAL_REGRIP: "Regrip",
}

# Sequence of calibration steps
CAL_SEQUENCE = [
    CAL_LEFT_GRIP, CAL_LEFT_WRIST, CAL_RIGHT_GRIP, CAL_RIGHT_WRIST,
    CAL_LOAD, CAL_SLEEP, CAL_REGRIP,
]


class CalibrationManager:
    """Manages physical button-driven servo calibration.

    This class sets up button callbacks, tracks the calibration state,
    and updates servo positions and calibration values in response to
    button presses.

    The calibration process is:
    1. Enter calibration mode (via API or physical button)
    2. Step through each servo parameter with Enter
    3. Adjust each parameter with Plus/Minus
    4. Press Enter on the last step to save and exit

    Attributes:
        robot: Reference to the parent RobotController for hardware access.
        state: Current calibration state (CAL_IDLE or one of the CAL_* steps).
    """

    def __init__(self, robot):
        """Initialize the calibration manager.

        Args:
            robot: The parent RobotController instance, providing access
                to the hardware driver, config, calibration values,
                display, and threading lock.
        """
        self.robot = robot
        self.state = CAL_IDLE

    def setup_buttons(self):
        """Set up the three physical buttons and register callbacks.

        Called once during robot initialization. Only applicable for
        the PCA9685 variant which has physical buttons.
        """
        self._plus = Button(PLUS_BUTTON_PIN)
        self._minus = Button(MINUS_BUTTON_PIN)
        self._enter = Button(ENTER_BUTTON_PIN)
        self._plus.add_extended_listener(self._on_plus)
        self._minus.add_extended_listener(self._on_minus)
        self._enter.add_extended_listener(self._on_enter)

    def start_calibration(self):
        """Enter calibration mode, starting with left gripper tuning.

        Positions the servos for left gripper adjustment.
        """
        self.state = CAL_LEFT_GRIP
        driver = self.robot.driver
        config = self.robot.config
        cal = self.robot.calibration

        # Position servos for left grip calibration
        set_left_turn(driver, config, cal, 90 + cal.left_wrist_tune, 0.5)
        set_left_grip(driver, config, cal, cal.left_grip_tune)
        set_right_turn(driver, config, cal, 90 + cal.right_wrist_tune, 0.5)
        set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)

        self._update_display()

    def _update_display(self):
        """Update the OLED display with the current calibration step and value."""
        label = CAL_LABELS.get(self.state, "")
        cal = self.robot.calibration

        if self.state == CAL_LEFT_GRIP:
            value = str(cal.left_grip_tune)
        elif self.state == CAL_LEFT_WRIST:
            value = str(cal.left_wrist_tune)
        elif self.state == CAL_RIGHT_GRIP:
            value = str(cal.right_grip_tune)
        elif self.state == CAL_RIGHT_WRIST:
            value = str(cal.right_wrist_tune)
        elif self.state == CAL_LOAD:
            value = str(cal.load)
        elif self.state == CAL_SLEEP:
            value = "%4.2f" % cal.sleep
        elif self.state == CAL_REGRIP:
            value = "ON" if cal.regrip_enabled else "OFF"
        else:
            value = ""

        self.robot.display.show(label, value)

    def _on_plus(self, button, event):
        """Handle plus button press: increment the current calibration value."""
        if event != BUTTON_PRESSED:
            return
        if self.state == CAL_IDLE:
            return

        driver = self.robot.driver
        config = self.robot.config
        cal = self.robot.calibration

        if self.state == CAL_LEFT_GRIP:
            cal.left_grip_tune += 2
            set_left_grip(driver, config, cal, cal.left_grip_tune)

        elif self.state == CAL_LEFT_WRIST:
            cal.left_wrist_tune += 2
            set_left_turn(driver, config, cal, 90 + cal.left_wrist_tune, 0.5)

        elif self.state == CAL_RIGHT_GRIP:
            cal.right_grip_tune += 2
            set_right_grip(driver, config, cal, cal.right_grip_tune)

        elif self.state == CAL_RIGHT_WRIST:
            cal.right_wrist_tune += 2
            set_right_turn(driver, config, cal, 90 + cal.right_wrist_tune, 0.5)

        elif self.state == CAL_LOAD:
            cal.load += 2
            set_left_grip(driver, config, cal, cal.load + cal.left_grip_tune)
            set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)

        elif self.state == CAL_SLEEP:
            cal.sleep = min(cal.sleep + 0.05, 1.0)

        elif self.state == CAL_REGRIP:
            cal.regrip_enabled = not cal.regrip_enabled

        self._update_display()

    def _on_minus(self, button, event):
        """Handle minus button press: decrement the current calibration value."""
        if event != BUTTON_PRESSED:
            return
        if self.state == CAL_IDLE:
            return

        driver = self.robot.driver
        config = self.robot.config
        cal = self.robot.calibration

        if self.state == CAL_LEFT_GRIP:
            cal.left_grip_tune = max(cal.left_grip_tune - 2, 0)
            set_left_grip(driver, config, cal, cal.left_grip_tune)

        elif self.state == CAL_LEFT_WRIST:
            cal.left_wrist_tune = max(cal.left_wrist_tune - 2, 0)
            set_left_turn(driver, config, cal, 90 + cal.left_wrist_tune, 0.5)

        elif self.state == CAL_RIGHT_GRIP:
            cal.right_grip_tune = max(cal.right_grip_tune - 2, 0)
            set_right_grip(driver, config, cal, cal.right_grip_tune)

        elif self.state == CAL_RIGHT_WRIST:
            cal.right_wrist_tune = max(cal.right_wrist_tune - 2, 0)
            set_right_turn(driver, config, cal, 90 + cal.right_wrist_tune, 0.5)

        elif self.state == CAL_LOAD:
            cal.load = max(cal.load - 2, 0)
            set_left_grip(driver, config, cal, cal.load + cal.left_grip_tune)
            set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)

        elif self.state == CAL_SLEEP:
            cal.sleep = max(cal.sleep - 0.05, 0.0)

        elif self.state == CAL_REGRIP:
            cal.regrip_enabled = not cal.regrip_enabled

        self._update_display()

    def _on_enter(self, button, event):
        """Handle enter button press: advance to the next calibration step."""
        if event == BUTTON_DOUBLECLICKED:
            # Double-click exits calibration without saving
            self.state = CAL_IDLE
            self.robot.display.show("Cancelled", "")
            return

        if event != BUTTON_PRESSED:
            return

        driver = self.robot.driver
        config = self.robot.config
        cal = self.robot.calibration

        if self.state == CAL_IDLE:
            # Start calibration
            self.start_calibration()
            return

        # Find the current step index and advance to the next
        try:
            idx = CAL_SEQUENCE.index(self.state)
        except ValueError:
            self.state = CAL_IDLE
            return

        if idx < len(CAL_SEQUENCE) - 1:
            # Advance to next step
            next_state = CAL_SEQUENCE[idx + 1]
            self.state = next_state

            # Set up servos for the new calibration step
            if next_state == CAL_LEFT_WRIST:
                set_left_grip(driver, config, cal, cal.load + cal.left_grip_tune)

            elif next_state == CAL_RIGHT_GRIP:
                set_right_grip(driver, config, cal, cal.right_grip_tune)

            elif next_state == CAL_RIGHT_WRIST:
                set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)

            elif next_state == CAL_LOAD:
                set_left_grip(driver, config, cal, cal.load + cal.left_grip_tune)
                set_right_grip(driver, config, cal, cal.load + cal.right_grip_tune)

            self._update_display()

        else:
            # Last step: save calibration and return to idle
            cal.save()
            self.state = CAL_IDLE
            self.robot.display.show("Saved", "")
            home_servos(driver, config, cal, self.robot.servo_state)
