"""
OLED display abstraction.

Provides a simple interface for showing status messages on an SSD1306
OLED display (128x64, I2C). If no display hardware is available (GPIO
variant), NullDisplay silently ignores all calls.

The PCA9685 variant includes an OLED display for showing robot status
during calibration and operation. The GPIO variant does not, so it
uses NullDisplay instead.
"""


class OLEDDisplay:
    """SSD1306 OLED display wrapper (128x64 pixels, I2C).

    Uses the luma.oled library to render text on the display. The display
    shows two lines of status text using the VCR OSD Mono font.

    Attributes:
        device: The luma.oled device instance.
        font: The TrueType font used for rendering text.
    """

    def __init__(self, font_path="/home/pi/VCR_OSD_MONO_1.001.ttf"):
        """Initialize the OLED display.

        Args:
            font_path: Path to the TrueType font file. The VCR OSD Mono
                font is used for its clear, readable characters on the
                small display.
        """
        from luma.core.interface.serial import i2c
        from luma.core.render import canvas
        from luma.oled.device import ssd1306
        from PIL import ImageFont

        serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(serial)
        self.font = ImageFont.truetype(font_path, 20)
        self._canvas = canvas

        # Show boot message immediately
        self.show("Boot", "")

    def show(self, line1, line2=""):
        """Display two lines of text on the OLED.

        Args:
            line1: Text for the top line (y=0).
            line2: Text for the bottom line (y=40). Defaults to empty.
        """
        with self._canvas(self.device) as draw:
            draw.text((5, 0), str(line1), font=self.font, fill="white")
            if line2:
                draw.text((5, 40), str(line2), font=self.font, fill="white")

    def clear(self):
        """Clear the display."""
        with self._canvas(self.device) as draw:
            pass  # Empty canvas clears the display


class NullDisplay:
    """No-op display for hardware variants without an OLED screen.

    All method calls are silently ignored. This allows the rest of the
    code to call display methods without checking whether a physical
    display is present.
    """

    def show(self, line1, line2=""):
        """No-op: ignore display calls."""
        pass

    def clear(self):
        """No-op: ignore clear calls."""
        pass
