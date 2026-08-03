"""
Abstract base class for servo hardware drivers.

Both the GPIO and PCA9685 variants implement this interface, allowing
all higher-level code (moves, scanning, calibration) to work with
either hardware backend without modification.

Each method corresponds to a single servo actuator on the robot:
- left_grip / right_grip: Open and close the grippers that hold the cube.
- left_turn / right_turn: Rotate the wrist to turn cube faces.

The duty_value parameter has different units depending on the driver:
- GPIO driver: duty cycle as a percentage of the PWM period (0-100).
- PCA9685 driver: pulse width in milliseconds (typically 0-2.5 ms),
  which the driver converts to PCA9685 register ticks internally.
"""

from abc import ABC, abstractmethod


class ServoDriver(ABC):
    """Abstract interface for controlling the robot's four servos.

    Implementations must handle PWM signal generation for their specific
    hardware. The driver is initialized via setup() and cleaned up via
    cleanup(). Between those calls, the set_* methods move individual servos.
    """

    @abstractmethod
    def setup(self):
        """Initialize the hardware and start PWM signals.

        This must be called before any set_* methods. It configures
        GPIO pins or I2C communication and starts the servos at a
        neutral position.
        """
        pass

    @abstractmethod
    def set_left_turn(self, duty_value):
        """Set the left wrist (turn) servo to the given duty value.

        Args:
            duty_value: The PWM duty value (units depend on driver).
        """
        pass

    @abstractmethod
    def set_left_grip(self, duty_value):
        """Set the left gripper servo to the given duty value.

        Args:
            duty_value: The PWM duty value (units depend on driver).
        """
        pass

    @abstractmethod
    def set_right_turn(self, duty_value):
        """Set the right wrist (turn) servo to the given duty value.

        Args:
            duty_value: The PWM duty value (units depend on driver).
        """
        pass

    @abstractmethod
    def set_right_grip(self, duty_value):
        """Set the right gripper servo to the given duty value.

        Args:
            duty_value: The PWM duty value (units depend on driver).
        """
        pass

    @abstractmethod
    def cleanup(self):
        """Release hardware resources (GPIO pins, I2C bus, etc.).

        Called on shutdown. After this, no further set_* calls should
        be made.
        """
        pass
