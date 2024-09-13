// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Mark Komus
//
// SPDX-License-Identifier: MIT
#include "shared-bindings/audioeffects/Echo.h"

#include <stdint.h>
#include "py/runtime.h"
#include "effects_utils.h"


void common_hal_audioeffects_echo_construct(audioeffects_echo_obj_t *self, uint32_t delay_ms, mp_float_t decay, uint32_t buffer_size, uint8_t bits_per_sample,
    bool samples_signed, uint8_t channel_count, uint32_t sample_rate) {

    self->bits_per_sample = bits_per_sample;
    self->samples_signed = samples_signed;
    self->channel_count = channel_count;
    self->sample_rate = sample_rate;

    // check that buffer_size <= echo_buffer_size
    self->buffer_len = buffer_size;
    self->buffer = m_malloc(self->buffer_len);
    if (self->buffer == NULL) {
        common_hal_audioeffects_echo_deinit(self);
        m_malloc_fail(self->buffer_len);
    }
    memset(self->buffer, 0, self->buffer_len);

    self->decay = (uint16_t)(decay * (1 << 15));

    // calculate buffer size for the set delay
    self->delay_ms = delay_ms;
    self->echo_buffer_len = self->sample_rate / 1000.0f * self->delay_ms * sizeof(uint32_t);

    self->echo_buffer = m_malloc(self->echo_buffer_len);
    if (self->echo_buffer == NULL) {
        common_hal_audioeffects_echo_deinit(self);
        m_malloc_fail(self->echo_buffer_len);
    }
    memset(self->echo_buffer, 0, self->echo_buffer_len);

    // read is where we store the incoming sample
    // write is what we send to the outgoing buffer
    self->echo_buffer_read_pos = self->buffer_len / sizeof(uint32_t);
    self->echo_buffer_write_pos = 0;
}

bool common_hal_audioeffects_echo_deinited(audioeffects_echo_obj_t *self) {
    if (self->echo_buffer == NULL) {
        return true;
    }
    return false;
}

void common_hal_audioeffects_echo_deinit(audioeffects_echo_obj_t *self) {
    if (common_hal_audioeffects_echo_deinited(self)) {
        return;
    }
    self->echo_buffer = NULL;
    self->buffer = NULL;
}


uint32_t common_hal_audioeffects_echo_get_delay_ms(audioeffects_echo_obj_t *self) {
    return self->delay_ms;
}

void common_hal_audioeffects_echo_set_delay_ms(audioeffects_echo_obj_t *self, uint32_t delay_ms) {
    self->delay_ms = delay_ms;
    self->echo_buffer_len = self->sample_rate / 1000.0f * self->delay_ms * sizeof(uint32_t);

    self->echo_buffer = m_realloc(self->echo_buffer, self->echo_buffer_len);
    if (self->echo_buffer == NULL) {
        common_hal_audioeffects_echo_deinit(self);
        m_malloc_fail(self->echo_buffer_len);
    }

    uint32_t max_ebuf_length = self->echo_buffer_len / sizeof(uint32_t);

    if (self->echo_buffer_read_pos > max_ebuf_length) {
        self->echo_buffer_read_pos = 0;
        self->echo_buffer_write_pos = max_ebuf_length - (self->buffer_len / sizeof(uint32_t));
    } else if (self->echo_buffer_write_pos > max_ebuf_length) {
        self->echo_buffer_read_pos = self->buffer_len / sizeof(uint32_t);
        self->echo_buffer_write_pos = 0;
    }
}

mp_float_t common_hal_audioeffects_echo_get_decay(audioeffects_echo_obj_t *self) {
    return (mp_float_t)self->decay / (1 << 15);
}

void common_hal_audioeffects_echo_set_decay(audioeffects_echo_obj_t *self, mp_float_t decay) {
    self->decay = (uint16_t)(decay * (1 << 15));
}

uint32_t common_hal_audioeffects_echo_get_sample_rate(audioeffects_echo_obj_t *self) {
    return self->sample_rate;
}

uint8_t common_hal_audioeffects_echo_get_channel_count(audioeffects_echo_obj_t *self) {
    return self->channel_count;
}

uint8_t common_hal_audioeffects_echo_get_bits_per_sample(audioeffects_echo_obj_t *self) {
    return self->bits_per_sample;
}

void audioeffects_echo_reset_buffer(audioeffects_echo_obj_t *self,
    bool single_channel_output,
    uint8_t channel) {

}

bool common_hal_audioeffects_echo_get_playing(audioeffects_echo_obj_t *self) {
    return self->sample != NULL;
}

void common_hal_audioeffects_echo_play(audioeffects_echo_obj_t *self, mp_obj_t sample, bool loop) {
    // check buffer size vs our buffer length
    self->sample = sample;
    return;
}

void common_hal_audioeffects_echo_stop(audioeffects_echo_obj_t *self) {
    self->sample = NULL;
    memset(self->echo_buffer, 0, self->echo_buffer_len); // clear echo
    return;
}

audioio_get_buffer_result_t audioeffects_echo_get_buffer(audioeffects_echo_obj_t *self, bool single_channel_output, uint8_t channel,
    uint8_t **buffer, uint32_t *buffer_length) {

    *buffer_length = self->buffer_len;
    *buffer = (uint8_t *)self->buffer;

    if (self->sample == NULL) {

    } else {
        // Get the sample's buffer
        uint32_t *sample_buffer;
        uint32_t sample_buffer_len;

        // audioio_get_buffer_result_t result =
        audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&sample_buffer, &sample_buffer_len);

        uint32_t n = MIN(sample_buffer_len / sizeof(uint32_t), (self->buffer_len / sizeof(uint32_t)));
        *buffer_length = n * sizeof(uint32_t);

        uint32_t echo_buf_len = self->echo_buffer_len / sizeof(uint32_t);

        // pass the sample thru to our buffer adding in the echo
        for (uint32_t i = 0; i < n; i++) {
            if (self->echo_buffer_read_pos >= echo_buf_len) {
                self->echo_buffer_read_pos = 0;
            }

            uint32_t echo = self->echo_buffer[self->echo_buffer_read_pos++];
            self->buffer[i] = add16signed(mult16signed(echo, self->decay), sample_buffer[i]);
        }

        // copy sample buffer to echo buf to play back later
        // This now includes our current sound + previous echos
        for (uint32_t i = 0; i < n; i++) {
            if (self->echo_buffer_write_pos >= echo_buf_len) {
                self->echo_buffer_write_pos = 0;
            }

            self->echo_buffer[self->echo_buffer_write_pos++] = add16signed(sample_buffer[i], self->buffer[i]);
        }
    }

    return GET_BUFFER_MORE_DATA;
}

void audioeffects_echo_get_buffer_structure(audioeffects_echo_obj_t *self, bool single_channel_output,
    bool *single_buffer, bool *samples_signed, uint32_t *max_buffer_length, uint8_t *spacing) {

    if (self->sample != NULL) {
        audiosample_get_buffer_structure(self->sample, single_channel_output, single_buffer, samples_signed, max_buffer_length, spacing);
        *max_buffer_length = self->buffer_len;
    } else {
        *single_buffer = false;
        *samples_signed = true;
        *max_buffer_length = self->buffer_len;
        *spacing = 1;
    }
}
