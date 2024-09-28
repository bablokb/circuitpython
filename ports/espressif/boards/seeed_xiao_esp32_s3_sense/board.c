// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2020 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "supervisor/board.h"
#include "mpconfigboard.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

// Use the MP_WEAK supervisor/shared/board.c versions of routines not defined here.
