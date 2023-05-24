/*

  driver.h - driver code for STM32F103xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

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
#define DIGITAL_IN(port, pin) BITBAND_PERI(port->IDR, pin)
#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI((port)->ODR, pin) = on; }

#define timer(t) timerN(t)
#define timerN(t) TIM ## t
#define timerINT(t) timerint(t)
#define timerint(t) TIM ## t ## _IRQn
#define timerHANDLER(t) timerhandler(t)
#define timerhandler(t) TIM ## t ## _IRQHandler
#define timerCCEN(c, n) timerccen(c, n)
#define timerccen(c, n) TIM_CCER_CC ## c ## n ## E
#define timerCCMR(p, c) timerccmr(p, c)
#define timerccmr(p, c) TIM ## p->CCMR ## c
#define timerOCM(p, c) timerocm(p, c)
#define timerocm(p, c) TIM_CCMR ## p ##_OC ## c ## M_1|TIM_CCMR ## p ##_OC ## c ## M_2
#define timerOCMC(p, c) timerocmc(p, c)
#define timerocmc(p, c) (TIM_CCMR ## p ##_OC ## c ## M|TIM_CCMR ## p ##_CC ## c ## S)
#define timerCCR(t, c) timerccr(t, c)
#define timerccr(t, c) TIM ## t->CCR ## c
#define timerCCP(c, n) timerccp(c, n)
#define timerccp(c, n) TIM_CCER_CC ## c ## n ## P
#define timerCR2OIS(c, n) timercr2ois(c, n)
#define timercr2ois(c, n) TIM_CR2_OIS ## c ## n
#define timerAF(t, f) timeraf(t, f)
#define timeraf(t, f) GPIO_AF ## f ## _TIM ## t
#define timerCLKENA(t) timercken(t)
#define timercken(t) __HAL_RCC_TIM ## t ## _CLK_ENABLE

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

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "cnc3040_map.h"
#elif defined(BOARD_BTT_SKR_MINI_E3_V20)
  #include "btt_skr_mini_e3_2.0_map.h"
#elif defined(BOARD_BTT_SKR_MINI_E3_V20_ALT2)
  #include "btt_skr_mini_e3_2.0_alt2_map.h"
#elif defined(BOARD_MACH3_BOB)
  #include "mach3_bob_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#elif defined(BOARD_SVM)
  #include "svm_map.h"
#else // default board
  #include "generic_map.h"
#endif

// Define timer allocations.
#define STEPPER_TIMER TIM2
#define PULSE_TIMER TIM3
#define DEBOUNCE_TIMER TIM4

#ifdef SPINDLE_PWM_PORT_BASE

#if SPINDLE_PWM_PORT_BASE == GPIOA_BASE
  #if SPINDLE_PWM_PIN == 1 // PA1 - TIM5_CH2
    #define SPINDLE_PWM_TIMER_N     5
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
  #elif SPINDLE_PWM_PIN == 8 // PA8 - TIM1_CH1
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
  #elif SPINDLE_PWM_PIN == 1 // PA1 - TIM2_CH2
    #define SPINDLE_PWM_TIMER_N 2
    #define SPINDLE_PWM_TIMER_CH 2
    #define SPINDLE_PWM_TIMER_INV 0
    #define SPINDLE_PWM_AF_REMAP 0
  #endif
#elif SPINDLE_PWM_PORT_BASE == GPIOB_BASE
  #if SPINDLE_PWM_PIN == 0 // PB0 - TIM1_CH2N
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   1
    #define SPINDLE_PWM_AF_REMAP    0b01
  #endif
#endif

#if SPINDLE_PWM_AF_REMAP && SPINDLE_PWM_TIMER_N > 3
#error 'Timer 4+ pins cannot be remapped!'
#endif

#if SPINDLE_PWM_TIMER_CH == 1 || SPINDLE_PWM_TIMER_CH == 2
#define SPINDLE_PWM_CCR 1
#else
#define SPINDLE_PWM_CCR 2
#endif
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_CCR       timerCCR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_TIMER_CCMR      timerCCMR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_CCR)
#define SPINDLE_PWM_CCMR_OCM_SET    timerOCM(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_CCMR_OCM_CLR    timerOCMC(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#if SPINDLE_PWM_TIMER_INV
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, N)
#else
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, )
#endif

#define SPINDLE_PWM_PORT            ((GPIO_TypeDef *)SPINDLE_PWM_PORT_BASE)
#define SPINDLE_PWM_CLOCK_ENA       timerCLKENA(SPINDLE_PWM_TIMER_N)

#endif // SPINDLE_PWM_PORT_BASE

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

#if TRINAMIC_ENABLE && !(defined(BOARD_CNC_BOOSTERPACK) || defined(BOARD_BTT_SKR_MINI_E3_V20) || defined(BOARD_BTT_SKR_MINI_E3_V20_ALT2))
#error Trinamic plugin not supported!
#endif

#if (!USB_SERIAL_CDC || MPG_ENABLE) && !defined(SERIAL_MOD)
#define SERIAL_MOD 1
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
#ifdef HAS_IOPORTS
void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (uint32_t bit);
#endif

#endif // __DRIVER_H__
