// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Mark Komus
//
// SPDX-License-Identifier: MIT

#include <stdint.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/audioeffects/__init__.h"
#include "shared-bindings/audioeffects/Echo.h"

//| """Support for audio effects
//|
//| The `audioeffects` module contains classes to provide access to audio effects.
//|
//| """

static const mp_rom_map_elem_t audioeffects_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_audioeffects) },
    { MP_ROM_QSTR(MP_QSTR_Echo), MP_ROM_PTR(&audioeffects_echo_type) },
};

static MP_DEFINE_CONST_DICT(audioeffects_module_globals, audioeffects_module_globals_table);

const mp_obj_module_t audioeffects_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&audioeffects_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_audioeffects, audioeffects_module);
