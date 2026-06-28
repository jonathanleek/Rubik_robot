"""PCA9685 servo driver.

The original firmware used the deprecated ``Adafruit_PCA9685`` library and
addressed the chip with the 12-bit ``set_pwm(channel, 0, on_count)`` call. This
port uses the maintained CircuitPython driver (``adafruit_pca9685``) which
exposes a 16-bit ``duty_cycle``. ``_ms_to_duty_cycle`` reproduces the original's
pulse calibration exactly (including its intentional 200-count overshoot) so the
mechanical behaviour is unchanged.
"""

from __future__ import annotations

from typing import Protocol

from ..config import PWM_FREQUENCY, PWM_REG_FULL_SCALE, PWM_RES


class ServoDriver(Protocol):
    """Minimal interface the robot needs from a PWM driver."""

    def set_pulse_ms(self, channel: int, duty_ms: float) -> None: ...

    def close(self) -> None: ...


def _ms_to_duty_cycle(duty_ms: float, freq: int) -> int:
    """Convert a pulse width in ms to a 16-bit PCA9685 duty cycle.

    Mirrors the original ``int(duty_ms/1000 * freq * PWM_RES)`` fed into a
    12-bit register. The fraction of the period is therefore
    ``duty_ms/1000 * freq * (PWM_RES / PWM_REG_FULL_SCALE)`` and we scale that to
    16 bits.
    """
    fraction = (duty_ms / 1000.0) * freq * (PWM_RES / PWM_REG_FULL_SCALE)
    value = round(fraction * 0xFFFF)
    return max(0, min(0xFFFF, value))


class PCA9685ServoDriver:
    """Real driver, backed by an Adafruit PCA9685 over I2C."""

    def __init__(self, address: int, freq: int = PWM_FREQUENCY):
        # Imported lazily: these libraries only exist on the Pi.
        import board  # type: ignore
        import busio  # type: ignore
        from adafruit_pca9685 import PCA9685  # type: ignore

        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=address)
        self._pca.frequency = freq
        self._freq = freq

    def set_pulse_ms(self, channel: int, duty_ms: float) -> None:
        self._pca.channels[channel].duty_cycle = _ms_to_duty_cycle(duty_ms, self._freq)

    def close(self) -> None:
        try:
            self._pca.deinit()
        finally:
            self._i2c.deinit()


class MockServoDriver:
    """No-op driver that records the last pulse per channel.

    Lets the full API run on a developer machine with no hardware attached.
    """

    def __init__(self, *_args, **_kwargs):
        self.last_pulse_ms: dict[int, float] = {}
        self.calls: list[tuple[int, float]] = []

    def set_pulse_ms(self, channel: int, duty_ms: float) -> None:
        self.last_pulse_ms[channel] = duty_ms
        self.calls.append((channel, duty_ms))

    def close(self) -> None:  # pragma: no cover - nothing to release
        pass
