"""
Physical button input handling.

Provides debounced button input with support for press, release,
long-press, click, and double-click events. Used by the PCA9685
variant for physical servo calibration via three buttons:
    - Plus button (GPIO pin 11): Increment calibration values
    - Minus button (GPIO pin 13): Decrement calibration values
    - Enter button (GPIO pin 15): Confirm / advance to next setting

This module is a cleaned-up version of the original button.py with
the same event detection logic. The GPIO variant (which has no physical
buttons) does not use this module.
"""

import RPi.GPIO as GPIO
import time
from threading import Thread


# Button event constants
BUTTON_PRESSED = 1
BUTTON_RELEASED = 2
BUTTON_LONGPRESSED = 3
BUTTON_CLICKED = 4
BUTTON_DOUBLECLICKED = 5

# Timing thresholds
BUTTON_LONGPRESS_DURATION = 2    # seconds to hold for a long press
BUTTON_DOUBLECLICK_TIME = 1      # seconds to wait for second click

# Button pin assignments (BOARD numbering)
PLUS_BUTTON_PIN = 11
MINUS_BUTTON_PIN = 13
ENTER_BUTTON_PIN = 15


class _ClickThread(Thread):
    """Background thread that detects single vs double clicks.

    After a button release, this thread waits for BUTTON_DOUBLECLICK_TIME
    seconds. If a second click arrives, the main button handler cancels
    this thread and fires a double-click event. If no second click arrives,
    this thread fires a single-click event.
    """

    def __init__(self, button):
        Thread.__init__(self)
        self.daemon = True
        self.button = button
        self.is_running = True
        self.start()

    def run(self):
        start_time = time.time()
        while self.is_running and (time.time() - start_time < BUTTON_DOUBLECLICK_TIME):
            time.sleep(0.1)
        # If still running and only one click counted, fire single-click
        if self.button.click_count == 1 and not self.button.is_long_press:
            if self.button.x_listener:
                self.button.x_listener(self.button, BUTTON_CLICKED)
            self.button._click_thread = None
        self.is_running = False

    def stop(self):
        self.is_running = False


class _ButtonThread(Thread):
    """Background thread that detects long presses.

    Started when a button is pressed. If the button is still held after
    BUTTON_LONGPRESS_DURATION seconds, fires a long-press event. If the
    button is released before that, this thread is cancelled.
    """

    def __init__(self, button):
        Thread.__init__(self)
        self.daemon = True
        self.button = button
        self.is_running = False

    def run(self):
        self.is_running = True
        start_time = time.time()
        while self.is_running and (time.time() - start_time < BUTTON_LONGPRESS_DURATION):
            time.sleep(0.1)
        if self.is_running:
            if self.button.listener:
                self.button.listener(self.button, BUTTON_LONGPRESSED)

    def stop(self):
        self.is_running = False


class Button:
    """A single GPIO button with debouncing and event detection.

    Supports press, release, long-press, click, and double-click events.
    Button presses are debounced by suppressing duplicate up/down events.

    Usage:
        button = Button(pin_number)
        button.add_extended_listener(my_callback)

    The callback receives (button_instance, event_type) where event_type
    is one of the BUTTON_* constants.
    """

    def __init__(self, pin):
        """Initialize a button on the given GPIO pin.

        Args:
            pin: GPIO pin number (BOARD numbering). The pin is configured
                with an internal pull-up resistor, so the button should
                connect the pin to ground when pressed.
        """
        self.pin = pin
        self.listener = None
        self.x_listener = None
        self._button_thread = None
        self._click_thread = None
        self.click_count = 0
        self.is_long_press = False

        GPIO.setup(self.pin, GPIO.IN, GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin, GPIO.BOTH, self._on_gpio_event)

    def _on_x_event(self, button, event):
        """Internal extended event handler that implements click detection.

        This intermediary handler adds click and double-click detection
        on top of the basic press/release/long-press events.
        """
        if event == BUTTON_PRESSED:
            if self.x_listener:
                self.x_listener(button, BUTTON_PRESSED)
            self.is_long_press = False
            if self._click_thread is None:
                self.click_count = 0
                self._click_thread = _ClickThread(self)

        elif event == BUTTON_RELEASED:
            if self.x_listener:
                self.x_listener(button, BUTTON_RELEASED)
            # If this was a long press, cancel click detection
            if self.is_long_press and self._click_thread:
                self._click_thread.stop()
                self._click_thread = None
                return
            # Count clicks for double-click detection
            if self._click_thread and self._click_thread.is_running:
                self.click_count += 1
                if self.click_count == 2:
                    self._click_thread.stop()
                    self._click_thread = None
                    if self.x_listener:
                        self.x_listener(button, BUTTON_DOUBLECLICKED)
            else:
                self._click_thread = None

        elif event == BUTTON_LONGPRESSED:
            self.is_long_press = True
            if self.x_listener:
                self.x_listener(self, BUTTON_LONGPRESSED)

    def _on_gpio_event(self, channel):
        """GPIO interrupt callback for button state changes.

        Handles switch bounce by suppressing duplicate down-down and
        up-up transitions.
        """
        if GPIO.input(self.pin) == GPIO.LOW:
            # Button pressed (active low due to pull-up)
            if self._button_thread is None:  # Suppress duplicate down
                self._button_thread = _ButtonThread(self)
                self._button_thread.start()
                if self.listener:
                    self.listener(self, BUTTON_PRESSED)
        else:
            # Button released
            if self._button_thread is not None:  # Suppress duplicate up
                self._button_thread.stop()
                self._button_thread.join(200)
                self._button_thread = None
                if self.listener:
                    self.listener(self, BUTTON_RELEASED)

    def add_extended_listener(self, listener):
        """Register a callback for all button events including clicks.

        The callback receives (button, event) where event is one of:
        BUTTON_PRESSED, BUTTON_RELEASED, BUTTON_LONGPRESSED,
        BUTTON_CLICKED, BUTTON_DOUBLECLICKED.

        Args:
            listener: Callback function(button, event).
        """
        self.listener = self._on_x_event
        self.x_listener = listener

    def add_listener(self, listener):
        """Register a callback for basic press/release events only.

        The callback receives (button, event) where event is one of:
        BUTTON_PRESSED, BUTTON_RELEASED, BUTTON_LONGPRESSED.

        Args:
            listener: Callback function(button, event).
        """
        self.listener = listener
