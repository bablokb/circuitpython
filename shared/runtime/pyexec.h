/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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
#ifndef MICROPY_INCLUDED_LIB_UTILS_PYEXEC_H
#define MICROPY_INCLUDED_LIB_UTILS_PYEXEC_H

#include "py/obj.h"

typedef enum {
    PYEXEC_MODE_FRIENDLY_REPL,
    PYEXEC_MODE_RAW_REPL,
} pyexec_mode_kind_t;

// CIRCUITPY-CHANGE
typedef struct {
    int return_code;
    mp_obj_t exception;
    int exception_line;
    // Only store the first 32 characters of the filename. It is very unlikely that they can all be
    // seen.
    char exception_filename[33];
} pyexec_result_t;

extern pyexec_mode_kind_t pyexec_mode_kind;

#define PYEXEC_FORCED_EXIT (0x100)
// CIRCUITPY-CHANGE: additional flags
#define PYEXEC_EXCEPTION   (0x200)
#define PYEXEC_DEEP_SLEEP  (0x400)
#define PYEXEC_RELOAD      (0x800)

int pyexec_raw_repl(void);
int pyexec_friendly_repl(void);
// CIRCUITPY-CHANGE: result out argument
int pyexec_file(const char *filename, pyexec_result_t *result);
int pyexec_file_if_exists(const char *filename, pyexec_result_t *result);
int pyexec_frozen_module(const char *name, bool allow_keyboard_interrupt, pyexec_result_t *result);
int pyexec_vstr(vstr_t *str, bool allow_keyboard_interrupt, pyexec_result_t *result);
void pyexec_event_repl_init(void);
int pyexec_event_repl_process_char(int c);
extern uint8_t pyexec_repl_active;

// CIRCUITPY-CHANGE: atexit support
#if CIRCUITPY_ATEXIT
int pyexec_exit_handler(const void *source, pyexec_result_t *result);
#endif

// CIRCUITPY-CHANGE
#if CIRCUITPY_WATCHDOG
pyexec_result_t *pyexec_result(void);
#endif

#if MICROPY_REPL_INFO
mp_obj_t pyb_set_repl_info(mp_obj_t o_value);
MP_DECLARE_CONST_FUN_OBJ_1(pyb_set_repl_info_obj);
#endif

#endif // MICROPY_INCLUDED_LIB_UTILS_PYEXEC_H
