"""
GPIO PWM servo driver.

Controls servos using the Raspberry Pi's built-in GPIO pins and
software PWM via the RPi.GPIO library. This is the simpler variant
that does not require additional hardware beyond the Pi itself.

Pin assignments (BOARD numbering):
    - Pin 33: Left wrist (turn) servo
    - Pin 31: Left gripper servo
    - Pin 37: Right wrist (turn) servo
    - Pin 35: Right gripper servo

PWM duty cycle is specified as a percentage of the period. At 50Hz
(20ms period), a 7% duty cycle gives ~1.4ms pulse width, which is
roughly the center position for most servos.
"""

import RPi.GPIO as GPIO

from rubik_robot.hardware.base import ServoDriver


# GPIO pin assignments (BOARD numbering scheme)
LEFT_WRIST_PIN = 33
LEFT_GRIP_PIN = 31
RIGHT_WRIST_PIN = 37
RIGHT_GRIP_PIN = 35


class GPIODriver(ServoDriver):
    """Servo driver using Raspberry Pi GPIO software PWM.

    This driver generates PWM signals directly on GPIO pins. It is
    simpler to set up (no extra hardware) but may exhibit slight
    jitter compared to the PCA9685 variant.

    Attributes:
        pwm_freq: PWM frequency in Hz (typically 50Hz for servos).
    """

    def __init__(self, pwm_freq=50):
        """Initialize the GPIO driver.

        Args:
            pwm_freq: PWM frequency in Hz. Standard servo frequency
                is 50Hz (20ms period).
        """
        self.pwm_freq = pwm_freq
        self._left_turn = None
        self._left_grip = None
        self._right_turn = None
        self._right_grip = None

    def setup(self):
        """Configure GPIO pins and start PWM signals.

        Sets up BOARD numbering mode, configures all four servo pins
        as outputs, and starts PWM at neutral positions (7% for wrists,
        5% for grippers).
        """
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(LEFT_WRIST_PIN, GPIO.OUT)
        GPIO.setup(LEFT_GRIP_PIN, GPIO.OUT)
        GPIO.setup(RIGHT_WRIST_PIN, GPIO.OUT)
        GPIO.setup(RIGHT_GRIP_PIN, GPIO.OUT)

        self._left_turn = GPIO.PWM(LEFT_WRIST_PIN, self.pwm_freq)
        self._left_grip = GPIO.PWM(LEFT_GRIP_PIN, self.pwm_freq)
        self._right_turn = GPIO.PWM(RIGHT_WRIST_PIN, self.pwm_freq)
        self._right_grip = GPIO.PWM(RIGHT_GRIP_PIN, self.pwm_freq)

        # Start at neutral positions
        self._left_turn.start(7)
        self._left_grip.start(5)
        self._right_turn.start(7)
        self._right_grip.start(5)

    def set_left_turn(self, duty_value):
        """Set left wrist servo duty cycle.

        Args:
            duty_value: Duty cycle as a percentage (0-100).
        """
        self._left_turn.ChangeDutyCycle(duty_value)

    def set_left_grip(self, duty_value):
        """Set left gripper servo duty cycle.

        Args:
            duty_value: Duty cycle as a percentage (0-100).
        """
        self._left_grip.ChangeDutyCycle(duty_value)

    def set_right_turn(self, duty_value):
        """Set right wrist servo duty cycle.

        Args:
            duty_value: Duty cycle as a percentage (0-100).
        """
        self._right_turn.ChangeDutyCycle(duty_value)

    def set_right_grip(self, duty_value):
        """Set right gripper servo duty cycle.

        Args:
            duty_value: Duty cycle as a percentage (0-100).
        """
        self._right_grip.ChangeDutyCycle(duty_value)

    def cleanup(self):
        """Stop all PWM signals and release GPIO resources."""
        GPIO.cleanup()
