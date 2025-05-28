// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 Lucian Copeland for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "py/runtime.h"

#include "shared-bindings/alarm/__init__.h"
#include "shared-bindings/alarm/pin/PinAlarm.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "common-hal/microcontroller/__init__.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"

#ifdef PICO_RP2350
#include "hardware/powman.h"
#endif

#ifdef SLEEP_DEBUG
#include <inttypes.h>
#include "py/mpprint.h"
#define DEBUG_PRINT(fmt, ...) ((void)mp_printf(&mp_plat_print, "DBG:%s:%04d: " fmt "\n", __FILE__, __LINE__,##__VA_ARGS__))
#else
#define DEBUG_PRINT(fmt, ...)((void)0)
#endif

#if USE_POWMAN
#include "py/mphal.h"
static int gpio_wakeup_nums_in_use = 0;  // instances 0-3 available
#endif

static bool woke_up;
static uint64_t alarm_triggered_pins; // 36 actual pins
static uint64_t alarm_reserved_pins; // 36 actual pins
static bool _not_yet_deep_sleeping = false;

#define GPIO_IRQ_ALL_EVENTS 0x15u

static void gpio_callback(uint gpio, uint32_t events) {
    #if !USE_POWMAN
    alarm_triggered_pins |= (1 << gpio);
    woke_up = true;

    // gpio_acknowledge_irq(gpio, events) is called automatically, before this callback is called.

    if (_not_yet_deep_sleeping) {
        // Event went off prematurely, before we went to sleep, so set it again.
        gpio_set_irq_enabled(gpio, events, true);
    } else {
        // Went off during sleep.
        // Disable IRQ automatically.
        gpio_set_irq_enabled(gpio, events, false);
        gpio_set_dormant_irq_enabled(gpio, events, false);
    }
    #endif
}

void alarm_pin_pinalarm_entering_deep_sleep() {
    _not_yet_deep_sleeping = false;
}

void common_hal_alarm_pin_pinalarm_construct(alarm_pin_pinalarm_obj_t *self, const mcu_pin_obj_t *pin, bool value, bool edge, bool pull) {
    self->pin = pin;
    self->value = value;
    self->edge = edge;
    self->pull = pull;
}

const mcu_pin_obj_t *common_hal_alarm_pin_pinalarm_get_pin(alarm_pin_pinalarm_obj_t *self) {
    return self->pin;
}

bool common_hal_alarm_pin_pinalarm_get_value(alarm_pin_pinalarm_obj_t *self) {
    return self->value;
}

bool common_hal_alarm_pin_pinalarm_get_edge(alarm_pin_pinalarm_obj_t *self) {
    return self->edge;
}

bool common_hal_alarm_pin_pinalarm_get_pull(alarm_pin_pinalarm_obj_t *self) {
    return self->pull;
}

bool alarm_pin_pinalarm_woke_this_cycle(void) {
    return woke_up;
}

mp_obj_t alarm_pin_pinalarm_find_triggered_alarm(size_t n_alarms, const mp_obj_t *alarms) {
    for (size_t i = 0; i < n_alarms; i++) {
        if (!mp_obj_is_type(alarms[i], &alarm_pin_pinalarm_type)) {
            continue;
        }
        alarm_pin_pinalarm_obj_t *alarm = MP_OBJ_TO_PTR(alarms[i]);
        if (alarm_triggered_pins & (1 << alarm->pin->number)) {
            return alarms[i];
        }
    }
    return mp_const_none;
}

mp_obj_t alarm_pin_pinalarm_record_wake_alarm(void) {
    alarm_pin_pinalarm_obj_t *const alarm = &alarm_wake_alarm.pin_alarm;

    alarm->base.type = &alarm_pin_pinalarm_type;
    // TODO: how to obtain the correct pin from memory?
    alarm->pin = NULL;
    return alarm;
}

void alarm_pin_pinalarm_reset(void) {
    alarm_triggered_pins = 0;
    woke_up = false;

    // Clear all GPIO interrupts
    #ifdef PICO_RP2040
    for (uint8_t i = 0; i < 4; i++) {
    #else
    for (uint8_t i = 0; i < 6; i++) {
        #endif
        iobank0_hw->intr[i] = 0;
    }

    // Reset pins and pin IRQs
    for (size_t i = 0; i < NUM_BANK0_GPIOS; i++) {
        if (alarm_reserved_pins & (1 << i)) {
            gpio_set_irq_enabled(i, GPIO_IRQ_ALL_EVENTS, false);
            gpio_set_dormant_irq_enabled(i, GPIO_IRQ_ALL_EVENTS, false);
            reset_pin_number(i);
        }
    }
    alarm_reserved_pins = 0;
    #if USE_POWMAN
    powman_disable_all_wakeups();
    gpio_wakeup_nums_in_use = 0;
    #endif
}

void alarm_pin_pinalarm_set_alarms(bool deep_sleep, size_t n_alarms, const mp_obj_t *alarms) {
    for (size_t i = 0; i < n_alarms; i++) {
        if (mp_obj_is_type(alarms[i], &alarm_pin_pinalarm_type)) {
            alarm_pin_pinalarm_obj_t *alarm = MP_OBJ_TO_PTR(alarms[i]);

            gpio_init(alarm->pin->number);
            gpio_set_dir(alarm->pin->number, GPIO_IN);
            if (alarm->pull) {
                // If value is high, the pullup should be off, and vice versa
                gpio_set_pulls(alarm->pin->number, !alarm->value, alarm->value);
            } else {
                // Clear in case the pulls are already on
                gpio_set_pulls(alarm->pin->number, false, false);
            }
            #if USE_POWMAN
            int gpio_wakeup_num = 0;
            if (deep_sleep) {
                if (gpio_wakeup_nums_in_use < 4) {
                    gpio_wakeup_num = gpio_wakeup_nums_in_use++;
                } else {
                    // ignore for now, maybe raise exception??
                }
                if (gpio_get(alarm->pin->number) == alarm->value) {
                    while(gpio_get(alarm->pin->number) == alarm->value) {
                        mp_hal_delay_ms(10);
                    }
                }
                DEBUG_PRINT("enabling PowMan GPIO-wakeup on GPIO %d",alarm->pin->number);
                DEBUG_PRINT("using wakeup-num: %d", gpio_wakeup_num);
                powman_enable_gpio_wakeup(gpio_wakeup_num, alarm->pin->number,
                                          alarm->edge, alarm->value);
                _not_yet_deep_sleeping = true;
                continue;
            }
            #endif

            // Don't reset at end of VM (instead, pinalarm_reset will reset before next VM)
            common_hal_never_reset_pin(alarm->pin);
            alarm_reserved_pins |= (1 << alarm->pin->number);

            uint32_t event;
            if (alarm->value == true && alarm->edge == true) {
                event = GPIO_IRQ_EDGE_RISE;
            } else if (alarm->value == false && alarm->edge == true) {
                event = GPIO_IRQ_EDGE_FALL;
            } else if (alarm->value == true && alarm->edge == false) {
                event = GPIO_IRQ_LEVEL_HIGH;
            } else { // both false
                event = GPIO_IRQ_LEVEL_LOW;
            }

            gpio_set_irq_enabled_with_callback((uint)alarm->pin->number, event, true, &gpio_callback);
            gpio_set_dormant_irq_enabled((uint)alarm->pin->number, event, true);

            _not_yet_deep_sleeping = true;
        }
    }
}
