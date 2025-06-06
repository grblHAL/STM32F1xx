/*

  driver.h - driver code for STM32F103xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

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

#define usart(t) usartN(t)
#define usartN(t) USART ## t
#define usartINT(t) usartint(t)
#define usartint(t) USART ## t ## _IRQn
#define usartHANDLER(t) usarthandler(t)
#define usarthandler(t) USART ## t ## _IRQHandler
#define usartCLKEN(t) usartclken(t)
#define usartclken(t) __HAL_RCC_USART ## t ## _CLK_ENABLE

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

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE (CONTROL_HALT|CONTROL_FEED_HOLD|CONTROL_CYCLE_START)
#endif

#ifdef BOARD_CNC_BOOSTERPACK
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_BTT_SKR_MINI_E3_V20)
  #include "boards/btt_skr_mini_e3_2.0_map.h"
#elif defined(BOARD_BTT_SKR_MINI_E3_V20_ALT2)
  #include "boards/btt_skr_mini_e3_2.0_alt2_map.h"
#elif defined(BOARD_MACH3_BOB)
  #include "boards/mach3_bob_map.h"
#elif defined(BOARD_SVM)
  #include "boards/svm_map.h"
#elif defined(BOARD_SUPERGERBIL)
  #include "boards/supergerbil_map.h"
#elif defined(BOARD_CREALITY_V4_2_2) || defined(BOARD_CREALITY_V4_2_7)
  #include "boards/creality_v4.2.x_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#else // default board
  #include "boards/generic_map.h"
#endif

#ifdef SPINDLE_PWM_PORT_BASE

#if SPINDLE_PWM_PORT_BASE == GPIOA_BASE
  #if SPINDLE_PWM_PIN == 1 // PA1
   #ifdef STM32F103xE      // - TIM5_CH2
    #define SPINDLE_PWM_TIMER_N     5
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
   #else                   // - TIM2_CH2
    #define SPINDLE_PWM_TIMER_N     2
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
   #endif
  #elif SPINDLE_PWM_PIN == 8 // PA8 - TIM1_CH1
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
  #elif SPINDLE_PWM_PIN == 10 // PA10 - TIM1_CH3
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    3
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
  #endif
#elif SPINDLE_PWM_PORT_BASE == GPIOB_BASE
  #if SPINDLE_PWM_PIN == 0 // PB0 - TIM1_CH2N
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   1
    #define SPINDLE_PWM_AF_REMAP    0b01
  #elif SPINDLE_PWM_PIN == 9 // PB9 - TIM4_CH4
    #define SPINDLE_PWM_TIMER_N     4
    #define SPINDLE_PWM_TIMER_CH    4
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
  #endif
#elif SPINDLE_PWM_PORT_BASE == GPIOC_BASE
  #if SPINDLE_PWM_PIN == 9 // PC9
   #ifdef STM32F103xE     // - TIM8_CH4
    #define SPINDLE_PWM_TIMER_N     8
    #define SPINDLE_PWM_TIMER_CH    4
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0
   #else                   // - TIM3_CH4
    #define SPINDLE_PWM_TIMER_N     3
    #define SPINDLE_PWM_TIMER_CH    4
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_AF_REMAP    0b11
   #endif
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

#ifndef SPINDLE_PWM_PORT
#define SPINDLE_PWM_PORT            ((GPIO_TypeDef *)SPINDLE_PWM_PORT_BASE)
#endif
#define SPINDLE_PWM_CLOCK_ENA       timerCLKENA(SPINDLE_PWM_TIMER_N)

#endif // SPINDLE_PWM_PORT_BASE

#if SPINDLE_PWM_TIMER_N == 2
#define STEPPER_TIMER_N 1
#else
#define STEPPER_TIMER_N 2
#endif
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_CLKEN         timerCLKENA(STEPPER_TIMER_N)
#if STEPPER_TIMER_N == 1
#define STEPPER_TIMER_IRQn          TIM1_UP_IRQn
#define STEPPER_TIMER_IRQHandler    TIM1_UP_IRQHandler
#else
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)
#endif

// Adjust these values to get more accurate step pulse timings when required, e.g if using high step rates.
// The default values are calibrated for 5 microsecond pulses.
// NOTE: step output mode, number of axes and compiler optimization setting may all affect these values.

// Minimum pulse off time
#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.5f
#endif
// Time from main stepper interrupt to pulse output, must be less than STEP_PULSE_TOFF
// Adjust for correct pulse off time after configuring and running at a step rate > max possible.
#ifndef STEP_PULSE_TON_LATENCY
#define STEP_PULSE_TON_LATENCY 1.8f
#endif
// Time from step out to step reset.
// Adjust for correct step pulse time.
#ifndef STEP_PULSE_TOFF_LATENCY
#define STEP_PULSE_TOFF_LATENCY 1.9f
#endif

// End configuration

#include "grbl/driver_opts2.h"

#if TRINAMIC_ENABLE
#include "trinamic/common.h"
#endif

#if I2C_ENABLE && !defined(I2C_PORT)
#define I2C_PORT 2
#endif

#if SDCARD_ENABLE && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif

#if TRINAMIC_ENABLE && !(defined(BOARD_CNC_BOOSTERPACK) || defined(BOARD_BTT_SKR_MINI_E3_V20) || defined(BOARD_BTT_SKR_MINI_E3_V20_ALT2))
#error Trinamic plugin not supported!
#endif

#if (!USB_SERIAL_CDC || MPG_ENABLE) && !defined(SERIAL_PORT)
#define SERIAL_PORT 1
#endif

typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint32_t bit;
    GPIO_TypeDef *port;
    pin_group_t group;
    uint8_t user_port;
    volatile bool active;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    pin_mode_t mode;
    uint8_t pin;
    GPIO_TypeDef *port;
    pin_group_t group;
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
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);
void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
