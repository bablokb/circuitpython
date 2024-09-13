// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Mark Komus
//
// SPDX-License-Identifier: MIT

#pragma once

#include "shared-module/audioeffects/Echo.h"

extern const mp_obj_type_t audioeffects_echo_type;

void common_hal_audioeffects_echo_construct(audioeffects_echo_obj_t *self,
    uint32_t delay_ms, mp_float_t decay,
    uint32_t buffer_size, uint8_t bits_per_sample, bool samples_signed,
    uint8_t channel_count, uint32_t sample_rate);

void common_hal_audioeffects_echo_deinit(audioeffects_echo_obj_t *self);
bool common_hal_audioeffects_echo_deinited(audioeffects_echo_obj_t *self);

uint32_t common_hal_audioeffects_echo_get_sample_rate(audioeffects_echo_obj_t *self);
uint8_t common_hal_audioeffects_echo_get_channel_count(audioeffects_echo_obj_t *self);
uint8_t common_hal_audioeffects_echo_get_bits_per_sample(audioeffects_echo_obj_t *self);

uint32_t common_hal_audioeffects_echo_get_delay_ms(audioeffects_echo_obj_t *self);
void common_hal_audioeffects_echo_set_delay_ms(audioeffects_echo_obj_t *self, uint32_t delay_ms);

mp_float_t common_hal_audioeffects_echo_get_decay(audioeffects_echo_obj_t *self);
void common_hal_audioeffects_echo_set_decay(audioeffects_echo_obj_t *self, mp_float_t decay);

bool common_hal_audioeffects_echo_get_playing(audioeffects_echo_obj_t *self);
void common_hal_audioeffects_echo_play(audioeffects_echo_obj_t *self, mp_obj_t sample, bool loop);
void common_hal_audioeffects_echo_stop(audioeffects_echo_obj_t *self);
