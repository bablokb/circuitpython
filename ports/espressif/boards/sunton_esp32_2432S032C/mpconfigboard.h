// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2022 Dan Halbert for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

// Micropython setup

#define MICROPY_HW_BOARD_NAME       "sunton_esp32_2432S032C"
#define MICROPY_HW_MCU_NAME         "ESP32"
#define MICROPY_HW_LED_STATUS       (&pin_GPIO16)

#define CIRCUITPY_BOOT_BUTTON       (&pin_GPIO0)

#define DEFAULT_I2C_BUS_SDA         (&pin_GPIO21)
#define DEFAULT_I2C_BUS_SCL         (&pin_GPIO22)

#define CIRCUITPY_CONSOLE_UART_TX (&pin_GPIO1)
#define CIRCUITPY_CONSOLE_UART_RX (&pin_GPIO3)
