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

#if CIRCUITPY_POWMAN
#include "py/mphal.h"
#define POWMAN_MAX_WAKEUP_SLOTS (count_of(powman_hw->pwrup))
static unsigned _powman_wakeup_slots = 0;  // instances 0-3 available
typedef struct {
    uint32_t pin_number;
    bool edge;
    bool value;
    bool pull;
} powman_gpio_wakeup_data;
static powman_gpio_wakeup_data _powman_gpio_data[POWMAN_MAX_WAKEUP_SLOTS];
#endif

static bool woke_up;
static uint64_t alarm_triggered_pins; // 36 actual pins
static uint64_t alarm_reserved_pins; // 36 actual pins
static bool _not_yet_deep_sleeping = false;

#define GPIO_IRQ_ALL_EVENTS 0x15u

static void gpio_callback(uint gpio, uint32_t events) {
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
    #if CIRCUITPY_POWMAN
    powman_disable_all_wakeups();
    _powman_wakeup_slots = 0;
    #endif
}

void alarm_pin_pinalarm_set_alarms(bool deep_sleep, size_t n_alarms, const mp_obj_t *alarms) {
    #if CIRCUITPY_POWMAN
    int wakeup_slot = 0;
    powman_gpio_wakeup_data *gpio_info;
    #endif
    for (size_t i = 0; i < n_alarms; i++) {
        if (mp_obj_is_type(alarms[i], &alarm_pin_pinalarm_type)) {
            alarm_pin_pinalarm_obj_t *alarm = MP_OBJ_TO_PTR(alarms[i]);

            #if CIRCUITPY_POWMAN
            if (deep_sleep) {
               if (_powman_wakeup_slots < POWMAN_MAX_WAKEUP_SLOTS) {
                   wakeup_slot = _powman_wakeup_slots++;
               } else {
                   // ignore for now, maybe raise exception??
                   continue;
               }
               // save now for later use
               DEBUG_PRINT("saving GPIO-wakeup for GPIO %d in slot %d",
                           alarm->pin->number,wakeup_slot);
               gpio_info = &(_powman_gpio_data[wakeup_slot]);
               gpio_info->pin_number = alarm->pin->number;
               gpio_info->edge = alarm->edge;
               gpio_info->value = alarm->value;
               gpio_info->pull = alarm->pull;
            }
            #endif

            // configure gpio for "normal" wakeup, even in the powman
            // case. This is to support fake deep-sleep.
            gpio_init(alarm->pin->number);
            gpio_set_dir(alarm->pin->number, GPIO_IN);
            if (alarm->pull) {
                // If value is high, the pullup should be off, and vice versa
                gpio_set_pulls(alarm->pin->number, !alarm->value, alarm->value);
            } else {
                // Clear in case the pulls are already on
                gpio_set_pulls(alarm->pin->number, false, false);
            }

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

#if CIRCUITPY_POWMAN
void alarm_pin_powman_set_gpio_wakeup(void) {
    powman_gpio_wakeup_data gpio_info;
    for (unsigned i=0; i<_powman_wakeup_slots; ++i) {
        gpio_info = _powman_gpio_data[i];
        gpio_deinit(gpio_info.pin_number);
        gpio_init(gpio_info.pin_number);
        gpio_set_dir(gpio_info.pin_number, GPIO_IN);
        if (gpio_info.pull) {
            // If value is high, the pullup should be off, and vice versa
            gpio_set_pulls(gpio_info.pin_number, !gpio_info.value, gpio_info.value);
        } else {
            // Clear in case the pulls are already on
            gpio_set_pulls(gpio_info.pin_number, false, false);
        }
        if (gpio_get(gpio_info.pin_number) == gpio_info.value) {
            while(gpio_get(gpio_info.pin_number) == gpio_info.value) {
                mp_hal_delay_ms(10);
            }
        }
        DEBUG_PRINT("enabling PowMan GPIO-wakeup on GPIO %d, slot %d",
                    gpio_info.pin_number,i);
        powman_enable_gpio_wakeup(i, gpio_info.pin_number,
                                  gpio_info.edge, gpio_info.value);
    }
}
#endif
