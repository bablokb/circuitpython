// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Mark Komus
//
// SPDX-License-Identifier: MIT


__attribute__((always_inline))
static inline uint32_t add16signed(uint32_t a, uint32_t b) {
    #if (defined(__ARM_ARCH_7EM__) && (__ARM_ARCH_7EM__ == 1))
    return __QADD16(a, b);
    #else
    uint32_t result = 0;
    for (int8_t i = 0; i < 2; i++) {
        int16_t ai = a >> (sizeof(int16_t) * 8 * i);
        int16_t bi = b >> (sizeof(int16_t) * 8 * i);
        int32_t intermediate = (int32_t)ai + bi;
        if (intermediate > SHRT_MAX) {
            intermediate = SHRT_MAX;
        } else if (intermediate < SHRT_MIN) {
            intermediate = SHRT_MIN;
        }
        result |= (((uint32_t)intermediate) & 0xffff) << (sizeof(int16_t) * 8 * i);
    }
    return result;
    #endif
}

__attribute__((always_inline))
static inline uint32_t mult16signed(uint32_t val, int32_t mul) {
    #if (defined(__ARM_ARCH_7EM__) && (__ARM_ARCH_7EM__ == 1))
    mul <<= 16;
    int32_t hi, lo;
    enum { bits = 16 }; // saturate to 16 bits
    enum { shift = 15 }; // shift is done automatically
    asm volatile ("smulwb %0, %1, %2" : "=r" (lo) : "r" (mul), "r" (val));
    asm volatile ("smulwt %0, %1, %2" : "=r" (hi) : "r" (mul), "r" (val));
    asm volatile ("ssat %0, %1, %2, asr %3" : "=r" (lo) : "I" (bits), "r" (lo), "I" (shift));
    asm volatile ("ssat %0, %1, %2, asr %3" : "=r" (hi) : "I" (bits), "r" (hi), "I" (shift));
    asm volatile ("pkhbt %0, %1, %2, lsl #16" : "=r" (val) : "r" (lo), "r" (hi)); // pack
    return val;
    #else
    uint32_t result = 0;
    float mod_mul = (float)mul / (float)((1 << 15) - 1);
    for (int8_t i = 0; i < 2; i++) {
        int16_t ai = (val >> (sizeof(uint16_t) * 8 * i));
        int32_t intermediate = (int32_t)(ai * mod_mul);
        if (intermediate > SHRT_MAX) {
            intermediate = SHRT_MAX;
        } else if (intermediate < SHRT_MIN) {
            intermediate = SHRT_MIN;
        }
        intermediate &= 0x0000FFFF;
        result |= (((uint32_t)intermediate)) << (sizeof(int16_t) * 8 * i);
    }
    return result;
    #endif
}
