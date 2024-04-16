# -------------------------------------------------------------------------
# CircuitPython driver for Waveshare Pico ResTouch 3.5in display.
#
# The display uses an ILI9488-chip. Waveshare attaches SPI on one side
# to a 16-bit parallel bus on the ILI9488 with two SIPO shift-registers
# in between.
#
# This setup is incompatible with any standard software-driver. The
# driver here only works with a special hacked CircuitPython-version.
#
# Although untested, the normal _INIT_SEQUENCE should work with
# ILI9488 displays with a direct MCU-SPI 4-wire connection.
#
# Author: Bernhard Bablok
# License: MIT
#
# Website: https://github.com/bablokb/circuitpython-examples
#
# -------------------------------------------------------------------------

try:
    # used for typing only
    from typing import Any
except ImportError:
    pass

import busdisplay

# minimal init-sequence for SPI-4wire connected ILI9488
# snatched from: https://github.com/ptb99/micropython-ili9488

_INIT_SEQUENCE = (
  b"\x01\x80\x78"                 # SWRESET + 120ms delay
  b"\x21\x00"                     # INVON
  b"\xc2\x01\x33"                 # PWCTRL3
  b"\x3a\x01\x55"                 # PIXFMT
  b"\xb6\x03\x02\x02\x3b"         # DFUNCTR
  b"\x11\x80\x05"                 # SLPOUT + 5ms delay
  b"\x29\x00"                     # DISPLAY_ON
  b"\x36\x01"                     # MADCTL, must be last
)

# init-sequence (16-bit) for Waveshare Pico-ResTouch-3.5in display
# note: this display also needs a special hacked version of CircuitPython

_INIT_SEQUENCE_16 = (
  b"\x01\x80\x78"                     # SWRESET + 120ms delay
  b"\x21\x00"                         # INVON
  b"\xc2\x02\x00\x33"                 # PWCTRL3
  b"\x3a\x02\x00\x55"                 # PIXFMT
  b"\xb6\x06\x00\x02\x00\x02\x00\x3b" # DFUNCTR
  b"\x11\x80\x05"                     # SLPOUT + 5ms delay
  b"\x29\x00"                         # DISPLAY_ON
  b"\x36\x02\x00"                     # MADCTL, must be last
)

_MADCTL = {
    0: (480,320,b"\xe8"),
   90: (320,480,b"\x48"),
  180: (480,320,b"\x28"),
  270: (320,480,b"\x88")
  }

class ILI9488(busdisplay.BusDisplay):
  """
  ILI9488 display driver

  :param displayio.FourWire bus: bus that the display is connected to
  """

  def __init__(self, bus: displayio.FourWire, rotation=0, **kwargs: Any):
    # fix width,height and rotation
    # instead of CP rotating the content, we delegate it to the chip
    width,height,madctl = _MADCTL[rotation]
    super().__init__(bus, _INIT_SEQUENCE_16+madctl,
                     width=width,
                     height=height,
                     rotation=0,
                     **kwargs)
