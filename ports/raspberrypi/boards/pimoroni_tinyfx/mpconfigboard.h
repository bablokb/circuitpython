// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#define MICROPY_HW_BOARD_NAME "Pimoroni Tiny FX"
#define MICROPY_HW_MCU_NAME "rp2040"

#define CIRCUITPY_RGB_STATUS_INVERTED_PWM
#define CIRCUITPY_RGB_STATUS_R (&pin_GPIO13)
#define CIRCUITPY_RGB_STATUS_G (&pin_GPIO14)
#define CIRCUITPY_RGB_STATUS_B (&pin_GPIO15)

#define DEFAULT_I2C_BUS_SCL (&pin_GPIO17)
#define DEFAULT_I2C_BUS_SDA (&pin_GPIO16)
