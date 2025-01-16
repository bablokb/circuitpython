// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2019 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

// Micropython setup

#define MICROPY_HW_BOARD_NAME       "Waveshare ESP32-S3-Pico Compat"
#define MICROPY_HW_MCU_NAME         "ESP32S3"

#define MICROPY_HW_NEOPIXEL         (&pin_GPIO21)

#define CIRCUITPY_BOARD_I2C         (1)
#define CIRCUITPY_BOARD_I2C_PIN     {{.scl = &pin_GPIO16, .sda = &pin_GPIO15}}
