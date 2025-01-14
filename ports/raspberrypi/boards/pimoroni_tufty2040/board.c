/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "supervisor/board.h"

#include "mpconfigboard.h"
#include "common-hal/paralleldisplay/ParallelBus.h"
#include "shared-module/displayio/__init__.h"
#include "supervisor/shared/board.h"

#define DELAY 0x80

enum reg {
    SWRESET   = 0x01,
    TEOFF     = 0x34,
    TEON      = 0x35,
    MADCTL    = 0x36,
    COLMOD    = 0x3A,
    GCTRL     = 0xB7,
    VCOMS     = 0xBB,
    LCMCTRL   = 0xC0,
    VDVVRHEN  = 0xC2,
    VRHS      = 0xC3,
    VDVS      = 0xC4,
    FRCTRL2   = 0xC6,
    PWCTRL1   = 0xD0,
    PORCTRL   = 0xB2,
    GMCTRP1   = 0xE0,
    GMCTRN1   = 0xE1,
    INVOFF    = 0x20,
    SLPOUT    = 0x11,
    DISPON    = 0x29,
    GAMSET    = 0x26,
    DISPOFF   = 0x28,
    RAMWR     = 0x2C,
    INVON     = 0x21,
    CASET     = 0x2A,
    RASET     = 0x2B,
    PWMFRSEL  = 0xCC
};

enum MADCTL {
    ROW_ORDER   = 0b10000000,
    COL_ORDER   = 0b01000000,
    SWAP_XY     = 0b00100000,  // AKA "MV"
    SCAN_ORDER  = 0b00010000,
    RGB_BGR     = 0b00001000,
    HORIZ_ORDER = 0b00000100
};

uint8_t display_init_sequence[] = {
    SWRESET, DELAY, 0x64, // 100ms delay
    TEON, DELAY, 0x64,
    COLMOD, 1, 0x05,
    PORCTRL, 5, 0x0c, 0x0c, 0x00, 0x33, 0x33,
    LCMCTRL, 1, 0x2c,
    VDVVRHEN, 1, 0x01,
    VRHS, 1, 0x12,
    VDVS, 1, 0x20,
    PWCTRL1, 2, 0xa4, 0xa1,
    FRCTRL2, 1, 0x0f,
    GCTRL, 1, 0x35,
    VCOMS, 1, 0x1f,
    0xd6, 1, 0xa1,
    GMCTRP1, 14, 0xd0, 0x08, 0x11, 0x08, 0x0c, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D,
    GMCTRN1, 14, 0xd0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0b, 0x16, 0x14, 0x2f, 0x31,
    INVON, DELAY, 0x64,
    SLPOUT, DELAY, 0x64,
    DISPON, DELAY, 0x64,
    CASET, 4, 0x00, 0x00, 0x01, 0x3F,
    RASET, 4, 0x00, 0x00, 0x00, 0xEF,
    MADCTL, 1, ROW_ORDER | SWAP_XY | SCAN_ORDER,
};


void board_init(void) {
    paralleldisplay_parallelbus_obj_t *bus = &allocate_display_bus()->parallel_bus;
    bus->base.type = &paralleldisplay_parallelbus_type;
    common_hal_paralleldisplay_parallelbus_construct(bus,
        &pin_GPIO14, // Data0
        &pin_GPIO11, // Command or data
        &pin_GPIO10, // Chip select
        &pin_GPIO12, // Write
        &pin_GPIO13, // Read
        NULL, // Reset
        2500000); // Frequency (not fast enough to hit 60Hz, but seems to avoid the ghosting issue)

    displayio_display_obj_t *display = &allocate_display()->display;
    display->base.type = &displayio_display_type;
    common_hal_displayio_display_construct(display,
        bus,
        320, // Width
        240, // Height
        0, // column start
        0, // row start
        0, // rotation
        16, // Color depth
        false, // grayscale
        false, // pixels_in_byte_share_row (unused for depths > 8)
        1, // bytes per cell. Only valid for depths < 8
        false, // reverse_pixels_in_byte. Only valid for depths < 8
        true, // reverse_pixels_in_word
        CASET, // Set column command
        RASET, // Set row command
        RAMWR, // Write memory command
        display_init_sequence,
        sizeof(display_init_sequence),
        &pin_GPIO2, // Backlight pin
        NO_BRIGHTNESS_COMMAND,
        1.0f, // brightness (ignored)
        false, // single_byte_bounds
        false, // data_as_commands
        true, // auto_refresh
        60, // native_frames_per_second
        true, // backlight_on_high
        false, // SH1107_addressing
        250); // backlight pwm frequency (highest that will result 0.01 displaying an image)
    common_hal_never_reset_pin(&pin_GPIO2); // backlight pin
}

bool board_requests_safe_mode(void) {
    return false;
}

void reset_board(void) {
}

void board_deinit(void) {
    common_hal_displayio_release_displays();
}
