"""
PCA9685 I2C PWM servo driver.

Controls servos using an Adafruit PCA9685 16-channel PWM driver board
connected via I2C. This provides hardware-generated PWM signals that
are more stable and jitter-free than GPIO software PWM.

Channel assignments on the PCA9685 board:
    - Channel 0: Left wrist (turn) servo
    - Channel 1: Left gripper servo
    - Channel 2: Right wrist (turn) servo
    - Channel 3: Right gripper servo

PWM values are specified in milliseconds and converted to PCA9685
register ticks internally using:
    ticks = int(duty_ms / 1000 * pwm_freq * pwm_resolution)
"""

import RPi.GPIO as GPIO
import Adafruit_PCA9685

from rubik_robot.hardware.base import ServoDriver


# PCA9685 channel assignments
LEFT_WRIST_CHANNEL = 0
LEFT_GRIP_CHANNEL = 1
RIGHT_WRIST_CHANNEL = 2
RIGHT_GRIP_CHANNEL = 3

# PCA9685 I2C address (default for Adafruit board)
PCA9685_ADDRESS = 0x40

# PCA9685 has 12-bit resolution (4096 steps) plus a small overshoot
# buffer to ensure the servo reaches full range
PCA9685_RESOLUTION = 4096 + 200


class PCA9685Driver(ServoDriver):
    """Servo driver using the PCA9685 I2C PWM controller.

    This driver communicates with the PCA9685 board over I2C to
    generate precise PWM signals. It produces much more stable servo
    movement compared to GPIO software PWM.

    Attributes:
        pwm_freq: PWM frequency in Hz (typically 300Hz for this setup).
    """

    def __init__(self, pwm_freq=300):
        """Initialize the PCA9685 driver.

        Args:
            pwm_freq: PWM frequency in Hz. The PCA9685 variant uses
                300Hz for faster servo response.
        """
        self.pwm_freq = pwm_freq
        self._pwm = None

    def setup(self):
        """Initialize the PCA9685 board and set the PWM frequency.

        Also sets up GPIO board numbering mode (needed for button pins
        even though servos use I2C).
        """
        GPIO.setmode(GPIO.BOARD)
        self._pwm = Adafruit_PCA9685.PCA9685(address=PCA9685_ADDRESS)
        self._pwm.set_pwm_freq(self.pwm_freq)

    def _ms_to_ticks(self, duty_ms):
        """Convert a pulse width in milliseconds to PCA9685 register ticks.

        Args:
            duty_ms: Pulse width in milliseconds.

        Returns:
            Integer tick count for the PCA9685 register.
        """
        return int(duty_ms / 1000 * self.pwm_freq * PCA9685_RESOLUTION)

    def set_left_turn(self, duty_value):
        """Set left wrist servo pulse width.

        Args:
            duty_value: Pulse width in milliseconds.
        """
        self._pwm.set_pwm(LEFT_WRIST_CHANNEL, 0, self._ms_to_ticks(duty_value))

    def set_left_grip(self, duty_value):
        """Set left gripper servo pulse width.

        Args:
            duty_value: Pulse width in milliseconds.
        """
        self._pwm.set_pwm(LEFT_GRIP_CHANNEL, 0, self._ms_to_ticks(duty_value))

    def set_right_turn(self, duty_value):
        """Set right wrist servo pulse width.

        Args:
            duty_value: Pulse width in milliseconds.
        """
        self._pwm.set_pwm(RIGHT_WRIST_CHANNEL, 0, self._ms_to_ticks(duty_value))

    def set_right_grip(self, duty_value):
        """Set right gripper servo pulse width.

        Args:
            duty_value: Pulse width in milliseconds.
        """
        self._pwm.set_pwm(RIGHT_GRIP_CHANNEL, 0, self._ms_to_ticks(duty_value))

    def cleanup(self):
        """Release GPIO resources.

        Note: The PCA9685 board does not require explicit cleanup,
        but GPIO pins used for buttons need to be freed.
        """
        GPIO.cleanup()
