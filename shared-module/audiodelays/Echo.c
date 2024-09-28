// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Mark Komus
//
// SPDX-License-Identifier: MIT
#include "shared-bindings/audiodelays/Echo.h"

#include <stdint.h>
#include "py/runtime.h"
#include "shared-module/audiomixer/utils.h"


void common_hal_audiodelays_echo_construct(audiodelays_echo_obj_t *self, uint32_t max_delay_ms,
    mp_obj_t delay_ms, mp_obj_t decay, mp_obj_t mix,
    uint32_t buffer_size, uint8_t bits_per_sample,
    bool samples_signed, uint8_t channel_count, uint32_t sample_rate) {

    self->bits_per_sample = bits_per_sample;
    self->samples_signed = samples_signed;
    self->channel_count = channel_count;
    self->sample_rate = sample_rate;

    self->buffer_len = buffer_size;
    self->buffer[0] = m_malloc(self->buffer_len);
    if (self->buffer[0] == NULL) {
        common_hal_audiodelays_echo_deinit(self);
        m_malloc_fail(self->buffer_len);
    }
    memset(self->buffer[0], 0, self->buffer_len);
    self->buffer[1] = m_malloc(self->buffer_len);
    if (self->buffer[1] == NULL) {
        common_hal_audiodelays_echo_deinit(self);
        m_malloc_fail(self->buffer_len);
    }
    memset(self->buffer[1], 0, self->buffer_len);
    self->last_buf_idx = 1;

    if (decay == MP_OBJ_NULL) {
        decay = mp_obj_new_float(0.7);
    }
    synthio_block_assign_slot(decay, &self->decay, MP_QSTR_decay);

    if (delay_ms == MP_OBJ_NULL) {
        delay_ms = mp_obj_new_float(0.05);
    }
    synthio_block_assign_slot(delay_ms, &self->delay_ms, MP_QSTR_delay_ms);

    if (mix == MP_OBJ_NULL) {
        mix = mp_obj_new_float(0.5);
    }
    synthio_block_assign_slot(mix, &self->mix, MP_QSTR_mix);

    // Set the echo buffer for the max possible delay
    self->max_delay_ms = max_delay_ms;
    self->max_echo_buffer_len = self->sample_rate / 1000.0f * max_delay_ms * (self->channel_count * (self->bits_per_sample / 8));
    self->echo_buffer = m_malloc(self->max_echo_buffer_len);
    if (self->echo_buffer == NULL) {
        common_hal_audiodelays_echo_deinit(self);
        m_malloc_fail(self->max_echo_buffer_len);
    }
    memset(self->echo_buffer, 0, self->max_echo_buffer_len);

    // calculate current echo buffer size for the set delay
    mp_float_t f_delay_ms = synthio_block_slot_get(&self->delay_ms);
    self->echo_buffer_len = self->sample_rate / 1000.0f * f_delay_ms * (self->channel_count * (self->bits_per_sample / 8));

    // read is where we store the incoming sample
    // write is what we send to the outgoing buffer
    self->echo_buffer_read_pos = self->buffer_len / sizeof(uint16_t);
    self->echo_buffer_write_pos = 0;

    self->sample = NULL;
    self->sample_remaining_buffer = NULL;
    self->sample_buffer_length = 0;
    self->loop = false;
    self->more_data = false;
}

bool common_hal_audiodelays_echo_deinited(audiodelays_echo_obj_t *self) {
    if (self->echo_buffer == NULL) {
        return true;
    }
    return false;
}

void common_hal_audiodelays_echo_deinit(audiodelays_echo_obj_t *self) {
    if (common_hal_audiodelays_echo_deinited(self)) {
        return;
    }
    self->echo_buffer = NULL;
    self->buffer[0] = NULL;
    self->buffer[1] = NULL;
}


mp_obj_t common_hal_audiodelays_echo_get_delay_ms(audiodelays_echo_obj_t *self) {
    return self->delay_ms.obj;
}

void common_hal_audiodelays_echo_set_delay_ms(audiodelays_echo_obj_t *self, mp_obj_t delay_ms) {
    synthio_block_assign_slot(delay_ms, &self->delay_ms, MP_QSTR_delay_ms);

    mp_float_t f_delay_ms = synthio_block_slot_get(&self->delay_ms);
    self->echo_buffer_len = self->sample_rate / 1000.0f * f_delay_ms * (self->channel_count * (self->bits_per_sample / 8));

    uint32_t max_ebuf_length = self->echo_buffer_len / sizeof(uint32_t);

    if (self->echo_buffer_read_pos > max_ebuf_length) {
        self->echo_buffer_read_pos = 0;
        self->echo_buffer_write_pos = max_ebuf_length - (self->buffer_len / sizeof(uint16_t));
    } else if (self->echo_buffer_write_pos > max_ebuf_length) {
        self->echo_buffer_read_pos = self->buffer_len / sizeof(uint16_t);
        self->echo_buffer_write_pos = 0;
    }
}

mp_obj_t common_hal_audiodelays_echo_get_decay(audiodelays_echo_obj_t *self) {
    return self->decay.obj;
}

void common_hal_audiodelays_echo_set_decay(audiodelays_echo_obj_t *self, mp_obj_t decay) {
    synthio_block_assign_slot(decay, &self->decay, MP_QSTR_decay);
}

mp_obj_t common_hal_audiodelays_echo_get_mix(audiodelays_echo_obj_t *self) {
    return self->mix.obj;
}

void common_hal_audiodelays_echo_set_mix(audiodelays_echo_obj_t *self, mp_obj_t arg) {
    synthio_block_assign_slot(arg, &self->mix, MP_QSTR_mix);
}

uint32_t common_hal_audiodelays_echo_get_sample_rate(audiodelays_echo_obj_t *self) {
    return self->sample_rate;
}

uint8_t common_hal_audiodelays_echo_get_channel_count(audiodelays_echo_obj_t *self) {
    return self->channel_count;
}

uint8_t common_hal_audiodelays_echo_get_bits_per_sample(audiodelays_echo_obj_t *self) {
    return self->bits_per_sample;
}

void audiodelays_echo_reset_buffer(audiodelays_echo_obj_t *self,
    bool single_channel_output,
    uint8_t channel) {

}

bool common_hal_audiodelays_echo_get_playing(audiodelays_echo_obj_t *self) {
    return self->sample != NULL;
}

void common_hal_audiodelays_echo_play(audiodelays_echo_obj_t *self, mp_obj_t sample, bool loop) {
    if (audiosample_sample_rate(sample) != self->sample_rate) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's sample rate does not match"));
    }
    if (audiosample_channel_count(sample) != self->channel_count) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's channel count does not match"));
    }
    if (audiosample_bits_per_sample(sample) != self->bits_per_sample) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's bits_per_sample does not match"));
    }
    bool single_buffer;
    bool samples_signed;
    uint32_t max_buffer_length;
    uint8_t spacing;
    audiosample_get_buffer_structure(sample, false, &single_buffer, &samples_signed, &max_buffer_length, &spacing);
    if (samples_signed != self->samples_signed) {
        mp_raise_ValueError(MP_ERROR_TEXT("The sample's signedness does not match"));
    }
    self->sample = sample;
    self->loop = loop;

    audiosample_reset_buffer(self->sample, false, 0);
    audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);
    // Track length in terms of words.
    self->sample_buffer_length /= sizeof(uint16_t);
    self->more_data = result == GET_BUFFER_MORE_DATA;

    return;
}

void common_hal_audiodelays_echo_stop(audiodelays_echo_obj_t *self) {
    self->sample = NULL;
    return;
}

#define RANGE_LOW (-28000)
#define RANGE_HIGH (28000)
#define RANGE_SHIFT (16)
#define RANGE_SCALE (0xfffffff / (32768 * 4 - RANGE_HIGH))

// dynamic range compression via a downward compressor with hard knee
//
// When the output value is within the range +-28000 (about 85% of full scale),
// it is unchanged. Otherwise, it undergoes a gain reduction so that the
// largest possible values, (+32768,-32767) * 2 (2 for echo and sample),
// still fit within the output range
//
// This produces a much louder overall volume with multiple voices, without
// much additional processing.
//
// https://en.wikipedia.org/wiki/Dynamic_range_compression
static
int16_t mix_down_sample(int32_t sample) {
    if (sample < RANGE_LOW) {
        sample = (((sample - RANGE_LOW) * RANGE_SCALE) >> RANGE_SHIFT) + RANGE_LOW;
    } else if (sample > RANGE_HIGH) {
        sample = (((sample - RANGE_HIGH) * RANGE_SCALE) >> RANGE_SHIFT) + RANGE_HIGH;
    }
    return sample;
}

audioio_get_buffer_result_t audiodelays_echo_get_buffer(audiodelays_echo_obj_t *self, bool single_channel_output, uint8_t channel,
    uint8_t **buffer, uint32_t *buffer_length) {

    mp_float_t mix = MIN(1.0, MAX(synthio_block_slot_get(&self->mix), 0.0));
    mp_float_t decay = MIN(1.0, MAX(synthio_block_slot_get(&self->decay), 0.0));

    self->last_buf_idx = !self->last_buf_idx;
    int16_t *word_buffer = (int16_t *)self->buffer[self->last_buf_idx];
    uint32_t length = self->buffer_len / sizeof(uint16_t);
    int16_t *echo_buffer = (int16_t *)self->echo_buffer;
    uint32_t echo_buf_len = self->echo_buffer_len / sizeof(uint16_t);

    while (length != 0) {
        if (self->sample_buffer_length == 0) {
            if (!self->more_data) {
                if (self->loop && self->sample) {
                    audiosample_reset_buffer(self->sample, false, 0);
                } else {
                    self->sample = NULL;
                }
            }
            if (self->sample) {
                // Load another buffer
                audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);
                // Track length in terms of words.
                self->sample_buffer_length /= sizeof(uint16_t); // assuming 16 bit samples
                self->more_data = result == GET_BUFFER_MORE_DATA;
            }
        }

        // If we have no sample keep the echo echoing
        if (self->sample == NULL) {
            if (MP_LIKELY(self->bits_per_sample == 16)) {
                if (mix <= 0.01) {  // no sample sound and with no mix, no echo
                    memset(word_buffer, 0, length * sizeof(uint16_t));
                } else {
                    // sample signed/unsigned won't matter as we have no sample
                    for (uint32_t i = 0; i < length; i++) {
                        word_buffer[i] = echo_buffer[self->echo_buffer_read_pos++] * decay;

                        echo_buffer[self->echo_buffer_write_pos++] = word_buffer[i];

                        word_buffer[i] = word_buffer[i] * mix;

                        if (self->echo_buffer_read_pos >= echo_buf_len) {
                            self->echo_buffer_read_pos = 0;
                        }
                        if (self->echo_buffer_write_pos >= echo_buf_len) {
                            self->echo_buffer_write_pos = 0;
                        }
                    }

                }
            } else { // bits per sample is 8
                uint16_t *hword_buffer = (uint16_t *)word_buffer;
                uint16_t *echo_hsrc = (uint16_t *)self->echo_buffer;
                for (uint32_t i = 0; i < length * 2; i++) {
                    uint32_t echo_word = unpack8(echo_hsrc[i]);
                    echo_word = echo_word * decay;
                    hword_buffer[i] = pack8(echo_word);

                    echo_hsrc[self->echo_buffer_write_pos++] = hword_buffer[i];
                    if (self->echo_buffer_read_pos >= echo_buf_len) {
                        self->echo_buffer_read_pos = 0;
                    }
                    if (self->echo_buffer_write_pos >= echo_buf_len) {
                        self->echo_buffer_write_pos = 0;
                    }
                }
            }

            length = 0;
        } else { // we have a sample
            uint32_t n = MIN(self->sample_buffer_length, length);
            int16_t *sample_src = (int16_t *)self->sample_remaining_buffer;

            if (MP_LIKELY(self->bits_per_sample == 16)) {
                if (mix <= 0.01) { // sample only
                    for (uint32_t i = 0; i < n; i++) {
                        word_buffer[i] = sample_src[i];
                    }
                } else {
                    for (uint32_t i = 0; i < n; i++) {
                        int32_t sample_word = sample_src[i];
                        if (!self->samples_signed) {
                            sample_word = tosigned16(sample_word);
                        }

                        int32_t word = (echo_buffer[self->echo_buffer_read_pos++] * decay) + sample_word;
                        word_buffer[i] = mix_down_sample(word);

                        echo_buffer[self->echo_buffer_write_pos++] = word_buffer[i];

                        if (self->echo_buffer_read_pos >= echo_buf_len) {
                            self->echo_buffer_read_pos = 0;
                        }
                        if (self->echo_buffer_write_pos >= echo_buf_len) {
                            self->echo_buffer_write_pos = 0;
                        }

                        word_buffer[i] = (sample_word * (1.0 - mix)) + (word_buffer[i] * mix);
                    }
                }
            } else { // bits per sample is 8
                uint16_t *hword_buffer = (uint16_t *)word_buffer;
                uint16_t *sample_hsrc = (uint16_t *)sample_src;
                uint16_t *echo_hsrc = (uint16_t *)self->echo_buffer;
                for (uint32_t i = 0; i < n * 2; i++) {
                    uint32_t sample_word = unpack8(sample_hsrc[i]);
                    uint32_t echo_word = unpack8(echo_hsrc[i]);
                    if (MP_LIKELY(!self->samples_signed)) {
                        sample_word = tosigned16(sample_word);
                    }
                    echo_word = echo_word * decay;
                    sample_word = sample_word + echo_word;
                    hword_buffer[i] = pack8(sample_word);

                    echo_hsrc[self->echo_buffer_write_pos++] = pack8(sample_word + unpack8(hword_buffer[i]));
                    if (self->echo_buffer_read_pos >= echo_buf_len) {
                        self->echo_buffer_read_pos = 0;
                    }
                    if (self->echo_buffer_write_pos >= echo_buf_len) {
                        self->echo_buffer_write_pos = 0;
                    }
                }
            }

            length -= n;
            word_buffer += n;
            self->sample_remaining_buffer += n;
            self->sample_buffer_length -= n;
        }
    }

    *buffer = (uint8_t *)self->buffer[self->last_buf_idx];
    *buffer_length = self->buffer_len;
    return GET_BUFFER_MORE_DATA;
}

void audiodelays_echo_get_buffer_structure(audiodelays_echo_obj_t *self, bool single_channel_output,
    bool *single_buffer, bool *samples_signed, uint32_t *max_buffer_length, uint8_t *spacing) {

    *single_buffer = false;
    *samples_signed = true;
    *max_buffer_length = self->buffer_len;
    if (single_channel_output) {
        *spacing = self->channel_count;
    } else {
        *spacing = 1;
    }
}
