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
    self->echo_buffer_len = self->sample_rate / 1000.0f * self->delay_ms * (self->channel_count * (self->bits_per_sample / 8));

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

    self->sample = NULL;
    self->sample_remaining_buffer = NULL;
    self->sample_buffer_length = 0;
    self->loop = false;
    self->more_data = false;
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
    self->echo_buffer_len = self->sample_rate / 1000.0f * self->delay_ms * (self->channel_count * (self->bits_per_sample / 8));

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
    if (audiosample_sample_rate(sample) != self->sample_rate) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's sample rate does not match the effect's"));
    }
    if (audiosample_channel_count(sample) != self->channel_count) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's channel count does not match the effect's"));
    }
    if (audiosample_bits_per_sample(sample) != self->bits_per_sample) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's bits_per_sample does not match the effect's"));
    }
    bool single_buffer;
    bool samples_signed;
    uint32_t max_buffer_length;
    uint8_t spacing;
    audiosample_get_buffer_structure(sample, false, &single_buffer, &samples_signed, &max_buffer_length, &spacing);
    if (samples_signed != self->samples_signed) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's signedness does not match the effect's"));
    }
    self->sample = sample;
    self->loop = loop;

    audiosample_reset_buffer(self->sample, false, 0);
    audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);
    // Track length in terms of words.
    self->sample_buffer_length /= sizeof(uint32_t);
    self->more_data = result == GET_BUFFER_MORE_DATA;

    return;
}

void common_hal_audioeffects_echo_stop(audioeffects_echo_obj_t *self) {
    self->sample = NULL;
    // memset(self->echo_buffer, 0, self->echo_buffer_len); // clear echo
    // memset(self->buffer, 0, self->buffer_len);
    return;
}

audioio_get_buffer_result_t audioeffects_echo_get_buffer(audioeffects_echo_obj_t *self, bool single_channel_output, uint8_t channel,
    uint8_t **buffer, uint32_t *buffer_length) {

    uint32_t *word_buffer = (uint32_t *)self->buffer;
    uint32_t length = self->buffer_len / sizeof(uint32_t);
    uint32_t echo_buf_len = self->echo_buffer_len / sizeof(uint32_t);

    while (length != 0) {
        if (self->sample_buffer_length == 0) {
            if (!self->more_data) {
                if (self->loop) {
                    if (self->sample) {
                        audiosample_reset_buffer(self->sample, false, 0);
                    }
                } else {
                    self->sample = NULL;
                }
            }
            if (self->sample) {
                // Load another buffer
                audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);
                // Track length in terms of words.
                self->sample_buffer_length /= sizeof(uint32_t);
                self->more_data = result == GET_BUFFER_MORE_DATA;
            }
        }

        // If we have no sample keep the echo echoing
        if (self->sample == NULL) {
            if (MP_LIKELY(self->bits_per_sample == 16)) {
                if (MP_LIKELY(self->samples_signed)) {
                    for (uint32_t i = 0; i < length; i++) {
                        if (self->echo_buffer_read_pos >= echo_buf_len) {
                            self->echo_buffer_read_pos = 0;
                        }

                        uint32_t echo = self->echo_buffer[self->echo_buffer_read_pos++];
                        word_buffer[i] = mult16signed(echo, self->decay);

                        if (self->echo_buffer_write_pos >= echo_buf_len) {
                            self->echo_buffer_write_pos = 0;
                        }

                        self->echo_buffer[self->echo_buffer_write_pos++] = word_buffer[i];
                    }
                }
            }
            length = 0;
        } else {
            uint32_t n = MIN(self->sample_buffer_length, length);
            uint32_t *sample_src = self->sample_remaining_buffer;

            if (MP_LIKELY(self->bits_per_sample == 16)) {
                if (MP_LIKELY(self->samples_signed)) {
                    for (uint32_t i = 0; i < n; i++) {
                        uint32_t sample_word = sample_src[i];
                        uint32_t echo = self->echo_buffer[self->echo_buffer_read_pos++];
                        word_buffer[i] = add16signed(mult16signed(echo, self->decay), sample_word);
                        self->echo_buffer[self->echo_buffer_write_pos++] = add16signed(sample_word, word_buffer[i]);

                        if (self->echo_buffer_read_pos >= echo_buf_len) {
                            self->echo_buffer_read_pos = 0;
                        }
                        if (self->echo_buffer_write_pos >= echo_buf_len) {
                            self->echo_buffer_write_pos = 0;
                        }
                    }
                } else {
                    // for (uint32_t i = 0; i < n; i++) {
                    // uint32_t sample_word = sample_src[i];
                    // sample_word = tosigned16(sample_word);
                    // word_buffer[i] = add16signed(mult16signed(sample_word, self->decay), word_buffer[i]);
                    // }
                }
            } else {
                // uint16_t *hword_buffer = (uint16_t *)word_buffer;
                // uint16_t *sample_hsrc = (uint16_t *)sample_src;
                // for (uint32_t i = 0; i < n * 2; i++) {
                // uint32_t sample_word = unpack8(sample_hsrc[i]);
                // if (MP_LIKELY(!self->samples_signed)) {
                // sample_word = tosigned16(sample_word);
                // }
                // sample_word = mult16signed(sample_word, self->decay);
                // sample_word = add16signed(sample_word, unpack8(hword_buffer[i]));
                // hword_buffer[i] = pack8(sample_word);
                // }
            }

            length -= n;
            word_buffer += n;
            self->sample_remaining_buffer += n;
            self->sample_buffer_length -= n;
        }
    }
    *buffer = (uint8_t *)self->buffer;
    *buffer_length = self->buffer_len;
    return GET_BUFFER_MORE_DATA;
}

void audioeffects_echo_get_buffer_structure(audioeffects_echo_obj_t *self, bool single_channel_output,
    bool *single_buffer, bool *samples_signed, uint32_t *max_buffer_length, uint8_t *spacing) {

    if (self->sample != NULL) {
        audiosample_get_buffer_structure(self->sample, single_channel_output, single_buffer, samples_signed, max_buffer_length, spacing);
        *single_buffer = false;
        *max_buffer_length = self->buffer_len;
    } else {
        *single_buffer = true;
        *samples_signed = true;
        *max_buffer_length = self->buffer_len;
        if (single_channel_output) {
            *spacing = self->channel_count;
        } else {
            *spacing = 1;
        }
    }
}
