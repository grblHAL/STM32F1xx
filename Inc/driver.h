/*

  driver.h - driver code for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"
#include "grbl/driver_opts.h"

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

// Define timer allocations.
#define SPINDLE_PWM_TIMER TIM1
#define STEPPER_TIMER TIM2
#define PULSE_TIMER TIM3
#define DEBOUNCE_TIMER TIM4

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "cnc3040_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#elif defined(BTT_SKR_MINI_E3_V20)
  #include "btt_skr_mini_e3_2.0_map.h"
#else // default board
  #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#ifndef I2C_PORT
#define I2C_PORT 2
#endif
#endif

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

#if SDCARD_ENABLE && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif

#if TRINAMIC_ENABLE && !(defined(BOARD_CNC_BOOSTERPACK) || defined(BTT_SKR_MINI_E3_V20) || defined(TRINAMIC_DEBUG))
#error Trinamic plugin not supported!
#endif

#ifndef RESET_PORT
#define RESET_PORT CONTROL_PORT
#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT CONTROL_PORT
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT CONTROL_PORT
#endif
#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PORT)
#define SAFETY_DOOR_PORT CONTROL_PORT
#endif

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

bool driver_init (void);

#endif // __DRIVER_H__
