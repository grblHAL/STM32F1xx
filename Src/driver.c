/*

  driver.c - driver code for STM32F103xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

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

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "driver.h"
#include "serial.h"

#if defined(HAS_IOPORTS)
#define AUX_DEVICES
#endif

#include "grbl/protocol.h"
#include "grbl/machine_limits.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"

#if defined(STM32F103xB)
#undef AUX_CONTROLS_ENABLED
#define AUX_CONTROLS_ENABLED 0
//#undef AUXINPUT_MASK
//#define AUXINPUT_MASK 0
#endif

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|DEVICES_IRQ_MASK)

#if DRIVER_IRQMASK != (LIMIT_MASK_SUM+CONTROL_MASK_SUM+DEVICES_IRQ_MASK_SUM)
#error Interrupt enabled input pins must have unique pin numbers!
#endif

#include "grbl/stepdir_map.h"

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = RESET_PORT,       .pin = RESET_PIN,           .group = PinGroup_Control },
#else
    { .id = Input_Reset,          .port = RESET_PORT,       .pin = RESET_PIN,           .group = PinGroup_Control },
#endif
    { .id = Input_FeedHold,       .port = FEED_HOLD_PORT,   .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = CYCLE_START_PORT, .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
#ifndef AUX_DEVICES
  #if SAFETY_DOOR_ENABLE
    { .id = Input_SafetyDoor,     .port = SAFETY_DOOR_PORT, .pin = SAFETY_DOOR_PIN,     .group = PinGroup_Control },
  #endif
  #ifdef PROBE_PIN
    { .id = Input_Probe,          .port = PROBE_PORT,       .pin = PROBE_PIN,           .group = PinGroup_Probe },
  #endif
  #ifdef I2C_STROBE_PIN
    { .id = Input_I2CStrobe,      .port = I2C_STROBE_PORT,  .pin = I2C_STROBE_PIN,      .group = PinGroup_Keypad },
  #endif
  #ifdef MPG_MODE_PIN
    { .id = Input_MPGSelect,      .port = MPG_MODE_PORT,    .pin = MPG_MODE_PIN,        .group = PinGroup_MPG },
  #endif
#endif // AUX_DEVICES
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,     .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = X2_LIMIT_PORT,    .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,     .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = Y2_LIMIT_PORT,    .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,     .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,       .port = Z2_LIMIT_PORT,    .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,         .port = A_LIMIT_PORT,     .pin = A_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = B_LIMIT_PORT,     .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = C_LIMIT_PORT,     .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
// Aux input pins must be consecutive in this array
#ifdef HAS_IOPORTS
  #ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,           .port = AUXINPUT0_PORT,   .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput },
  #endif
  #ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,           .port = AUXINPUT1_PORT,   .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput },
  #endif
  #ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,           .port = AUXINPUT2_PORT,   .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput },
  #endif
  #ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,           .port = AUXINPUT3_PORT,   .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput },
  #endif
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep, },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep, },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep, },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable, },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = X2_ENABLE_PORT,         .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = Y2_ENABLE_PORT,         .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = Z2_ENABLE_PORT,         .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#endif // !TRINAMIC_MOTOR_ENABLE
#ifdef SPINDLE_ENABLE_PIN
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#endif
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,             .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef HAS_IOPORTS
  #ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
  #endif
  #ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
  #endif
  #ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
  #endif
  #ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
  #endif
#endif
};

#if AUX_CONTROLS_ENABLED
static uint8_t probe_port;
static void aux_irq_handler (uint8_t port, bool state);
#endif

extern __IO uint32_t uwTick;
static uint32_t pulse_length, pulse_delay, aux_irq = 0;
static bool IOInitDone = false;
#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#endif
#if DRIVER_SPINDLE_PWM_ENABLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#endif
static pin_group_pins_t limit_inputs = {0};
static axes_signals_t next_step_outbits;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static input_signal_t *pin_irq[16] = {0};
static on_unknown_sys_command_ptr on_unknown_sys_command;
#ifndef STM32F103xB
static periph_signal_t *periph_pins = NULL;
#endif

#ifdef SAFETY_DOOR_PIN
static pin_debounce_t debounce;
#endif

#if AUX_CONTROLS_ENABLED
static void aux_irq_handler (uint8_t port, bool state);
#endif

#ifdef PROBE_PIN
static uint8_t probe_port;
static probe_state_t probe = {
    .connected = On
};
#endif // PROBE_PIN

#if I2C_STROBE_BIT || SPI_IRQ_BIT

#if I2C_STROBE_BIT
static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };
#endif

#if SPI_IRQ_BIT
static driver_irq_handler_t spi_irq = { .type = IRQ_SPI };
#endif

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok = false;

    switch(irq) {

#if I2C_STROBE_BIT
        case IRQ_I2C_Strobe:
            if((ok = i2c_strobe.callback == NULL))
                i2c_strobe.callback = handler;
            break;
#endif

#ifdef SPI_IRQ_BIT
        case IRQ_SPI:
            if((ok = spi_irq.callback == NULL))
                spi_irq.callback = handler;
            break;
#endif

        default:
            break;
    }

    return ok;
}

#endif // I2C_STROBE_BIT || SPI_IRQ_BIT


void disk_timerproc (void);

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        // Restart systick...
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if !TRINAMIC_MOTOR_ENABLE
  #ifdef STEPPERS_ENABLE_PORT
    DIGITAL_OUT(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_PIN, enable.x);
  #else
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, enable.x);
   #ifdef X2_ENABLE_PIN
    DIGITAL_OUT(X2_ENABLE_PORT, X2_ENABLE_PIN, enable.x);
   #endif
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, enable.y);
   #ifdef Y2_ENABLE_PIN
    DIGITAL_OUT(Y2_ENABLE_PORT, Y2_ENABLE_PIN, enable.y);
   #endif
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, enable.z);
   #ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(Z2_ENABLE_PORT, Z2_ENABLE_PIN, enable.z);
   #endif
   #ifdef A_ENABLE_PORT
    DIGITAL_OUT(A_ENABLE_PORT, A_ENABLE_PIN, enable.a);
   #endif
   #ifdef B_ENABLE_PORT
    DIGITAL_OUT(B_ENABLE_PORT, B_ENABLE_PIN, enable.b);
   #endif
   #ifdef C_ENABLE_PORT
    DIGITAL_OUT(C_ENABLE_PORT, C_ENABLE_PIN, enable.c);
   #endif
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->ARR = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
    STEPPER_TIMER->EGR = TIM_EGR_UG;
    STEPPER_TIMER->SR = ~TIM_SR_UIF;
    STEPPER_TIMER->CR1 |= TIM_CR1_CEN;
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->CNT = 0;
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
    // Set timer prescaling for normal step generation
    if (cycles_per_tick < (1UL << 16)) { // < 65536  (1.1ms @ 72MHz)
        STEPPER_TIMER->PSC = 0; // DIV 1
    } else if (cycles_per_tick < (1UL << 19)) { // < 524288 (8.8ms @ 72MHz)
        STEPPER_TIMER->PSC = 7; // DIV 8
        cycles_per_tick = cycles_per_tick >> 3;
    } else {
        STEPPER_TIMER->PSC = 63; // DIV64
        cycles_per_tick = cycles_per_tick >> 6;
    }
    STEPPER_TIMER->ARR = (uint16_t)(cycles_per_tick - 1);
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;

#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_outbits_1.x);
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_outbits_1.y);
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_outbits_1.z);
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif
 #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_outbits_1.a);
 #endif
 #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_outbits_1.b);
 #endif
 #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_outbits_1.c);
 #endif

#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits_1.value & motors_1.mask];
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif

#else // STEP_OUTMODE == GPIO_SHIFTx
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (((step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // SQUARING DISABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.mask ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_outbits.x);
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x);
  #endif
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_outbits.y);
  #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y);
  #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_outbits.z);
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z);
  #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_outbits.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_outbits.c);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits.value];
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x ^ settings.steppers.step_invert.x);
  #endif
  #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y ^ settings.steppers.step_invert.y);
  #endif
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z ^ settings.steppers.step_invert.z);
  #endif
#else // STEP_OUTMODE == GPIO_SHIFTx
 #if N_GANGED
   step_outbits.mask ^= settings.steppers.step_invert.mask;
   STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_outbits.mask << STEP_OUTMODE);
  #ifdef X2_STEP_PIN
   DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x);
  #endif
  #ifdef Y2_PIN
   DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y);
  #endif
  #ifdef Z2_STEP_PIN
   DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z);
  #endif
 #else
   STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
 #endif
#endif
}

#endif

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif

        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif

        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif

        #if Y_GANGED
            ganged.y = On;
        #endif

        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
}

#endif

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, dir_outbits.z);
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_outbits.z);
  #endif
 #endif
 #ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_PIN, dir_outbits.a);
 #endif
 #ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_PIN, dir_outbits.b);
 #endif
 #ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_PIN, dir_outbits.c);
 #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, (dir_outbits.x ^ settings.steppers.dir_invert.x) ^ settings.steppers.ganged_dir_invert.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, (dir_outbits.y ^ settings.steppers.dir_invert.y) ^ settings.steppers.ganged_dir_invert.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, (dir_outbits.z ^ settings.steppers.dir_invert.z) ^ settings.steppers.ganged_dir_invert.z);
  #endif
#else // DIRECTION_OUTMODE = SHIFTx
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (dir_outbits.mask << DIRECTION_OUTMODE);
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_outbits.z);
  #endif
 #else
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | ((dir_outbits.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
 #endif
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        stepperSetDirOutputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            next_step_outbits = stepper->step_outbits; // Store out_bits
            PULSE_TIMER->ARR = pulse_delay;
            PULSE_TIMER->EGR = TIM_EGR_UG;
            PULSE_TIMER->CR1 |= TIM_CR1_CEN;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    axes_signals_t pin;
    input_signal_t *limit;
    uint_fast8_t idx = limit_inputs.n_pins;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        limit = &limit_inputs.pins.inputs[--idx];
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit->id);
            disable = limit->group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        gpio_irq_enable(limit, disable ? IRQ_Mode_None : limit->mode.irq_mode);
    } while(idx);
}


// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

#if LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_PIN);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_PIN);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN);
  #ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_PIN);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_PIN);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PORT, C_LIMIT_PIN);
  #endif
#elif LIMIT_INMODE == GPIO_MAP
    uint32_t bits = LIMIT_PORT->IDR;
    signals.min.x = !!(bits & X_LIMIT_BIT);
    signals.min.y = !!(bits & Y_LIMIT_BIT);
    signals.min.z = !!(bits & Z_LIMIT_BIT);
  #ifdef A_LIMIT_PIN
    signals.min.a = !!(bits & A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = !!(bits & B_LIMIT_BIT);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = !!(bits & C_LIMIT_BIT);
  #endif
#else
    signals.min.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PORT, X2_LIMIT_PIN);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PORT, Y2_LIMIT_PIN);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PORT, Z2_LIMIT_PIN);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_PIN_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_PIN_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_PIN_MAX);
#endif

    if (settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

#if CONTROL_INMODE == GPIO_BITBAND
  #if ESTOP_ENABLE
    signals.e_stop = BITBAND_PERI(RESET_PORT->IDR, RESET_PIN);
  #else
    signals.reset = BITBAND_PERI(CONTROL_PORT->IDR, RESET_PIN);
  #endif
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IDR, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IDR, CYCLE_START_PIN);
 #if SAFETY_DOOR_BIT
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IDR, SAFETY_DOOR_PIN);
 #endif
#elif CONTROL_INMODE == GPIO_MAP
    uint32_t bits = CONTROL_PORT->IDR;
 #if ESTOP_ENABLE
    signals.e_stop = (bits & RESET_BIT) != 0;
 #else
    signals.reset = (bits & RESET_BIT) != 0;
 #endif
    signals.feed_hold = (bits & FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CYCLE_START_BIT) != 0;
  #if SAFETY_DOOR_BIT
    signals.safety_door_ajar = (bits & SAFETY_DOOR_BIT) != 0;
 #endif
#else
    signals.value &= ~(CONTROL_MASK >> CONTROL_INMODE);
    signals.value |= (uint16_t)((CONTROL_PORT->IDR & CONTROL_MASK) >> CONTROL_INMODE);
 #if ESTOP_ENABLE
    signals.e_stop = signals.reset;
    signals.reset = settings.control_invert.reset;
 #endif
#endif

#if AUX_CONTROLS_ENABLED

  #ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif
  #ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN);
  #endif
  #ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PORT, MOTOR_WARNING_PIN);
  #endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

  #if AUX_CONTROLS_SCAN
    signals = aux_ctrl_scan_status(signals);
  #endif

#else

  #ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

#endif // AUX_CONTROLS_ENABLED

    return signals;
}

#ifdef PROBE_PIN

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe.connected = !probe.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

#ifdef AUX_DEVICES

    if(hal.signals_cap.probe_triggered) {
        probe.is_probing = Off;
        probe.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe.triggered ? (probe.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe.irq_enabled = hal.port.register_interrupt_handler(probe_port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    }

    if(!probe.irq_enabled)
        probe.triggered = Off;

#else
    probe.triggered = Off;
#endif

    probe.is_probing = probing;
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
#ifdef AUX_DEVICES
    state.triggered = probe.is_probing && probe.irq_enabled ? probe.triggered : !!(PROBE_PORT->IDR & (1 << PROBE_PIN)) ^ probe.inverted;
#else
    state.triggered = !!(PROBE_PORT->IDR & (1 << PROBE_PIN)) ^ probe.inverted;
#endif

    return state;
}

#endif // PROBE_PIN

#if MPG_MODE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0);
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode != (DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0))
        stream_mpg_enable(true);
}

#endif // MPG_MODE == 1

#if AUX_CONTROLS_ENABLED

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};

    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#ifdef PROBE_PIN
            case Input_Probe:
                if(probe.is_probing) {
                    probe.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                protocol_enqueue_foreground_task(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(pin->irq_mode == IRQ_Mode_Change && pin->function != Input_Probe)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    if(ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL)) {
        ioport_assign_function(aux_ctrl, &((input_signal_t *)aux_ctrl->input)->id);
#ifdef PROBE_PIN
        if(aux_ctrl->function == Input_Probe) {
            probe_port = aux_ctrl->aux_port;
            hal.probe.get_state = probeGetState;
            hal.probe.configure = probeConfigure;
            hal.probe.connected_toggle = probeConnectedToggle;
            hal.driver_cap.probe_pull_up = On;
            hal.signals_cap.probe_triggered = aux_ctrl->irq_mode != IRQ_Mode_None;
        }
#endif
#if defined(SAFETY_DOOR_PIN)
        if(aux_ctrl->function == Input_SafetyDoor)
            ((input_signal_t *)aux_ctrl->input)->mode.debounce = On;
#endif
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif // AUX_CONTROLS_ENABLED

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->ODR, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
}

inline static void spindle_on (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->ODR, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
    BITBAND_PERI(SPINDLE_DIRECTION_PORT->ODR, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    if(!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

#if DRIVER_SPINDLE_PWM_ENABLE

// Variable spindle control functions

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if (pwm_value == spindle->context.pwm->off_value) {
        pwmEnabled = false;
        if(spindle->context.pwm->settings->flags.enable_rpm_controlled) {
            if(spindle->context.pwm->cloned)
                spindle_dir(false);
            else
                spindle_off();
        }
        if(spindle->context.pwm->always_on) {
            SPINDLE_PWM_TIMER_CCR = spindle->context.pwm->off_value;
#if SPINDLE_PWM_TIMER_N == 1
            SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
            SPINDLE_PWM_TIMER_CCR = pwm_value;
        } else
#if SPINDLE_PWM_TIMER_N == 1
            SPINDLE_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
#else
            SPINDLE_PWM_TIMER_CCR = 0;
#endif
    } else {
        if(!pwmEnabled) {
            if(spindle->context.pwm->cloned)
                spindle_dir(true);
            else
                spindle_on();
            pwmEnabled = true;
        }
        SPINDLE_PWM_TIMER_CCR = pwm_value;
#if SPINDLE_PWM_TIMER_N == 1
        SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
#ifdef SPINDLE_DIRECTION_PIN
    if(state.on || spindle->context.pwm->cloned)
        spindle_dir(state.ccw);
#endif
    if(!spindle->context.pwm->settings->flags.enable_rpm_controlled) {
        if(state.on)
            spindle_on();
        else
            spindle_off();
    }

    spindleSetSpeed(spindle, state.on || (state.ccw && spindle->context.pwm->cloned)
                              ? spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false)
                              : spindle->context.pwm->off_value);
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    uint32_t prescaler = 1;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, SystemCoreClock)) {

        while(spindle_pwm.period > 65534) {
            prescaler++;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, SystemCoreClock / prescaler);
        }

        spindle->set_state = spindleSetStateVariable;

        SPINDLE_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

        TIM_Base_InitTypeDef timerInitStructure = {
            .Prescaler = prescaler - 1,
            .CounterMode = TIM_COUNTERMODE_UP,
            .Period = spindle_pwm.period - 1,
            .ClockDivision = TIM_CLOCKDIVISION_DIV1,
            .RepetitionCounter = 0
        };

        TIM_Base_SetConfig(SPINDLE_PWM_TIMER, &timerInitStructure);

        SPINDLE_PWM_TIMER->CCER &= ~SPINDLE_PWM_CCER_EN;
        SPINDLE_PWM_TIMER_CCMR &= ~SPINDLE_PWM_CCMR_OCM_CLR;
        SPINDLE_PWM_TIMER_CCMR |= SPINDLE_PWM_CCMR_OCM_SET;
        SPINDLE_PWM_TIMER_CCR = 0;
#if SPINDLE_PWM_TIMER_N == 1
        SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
#endif
        if(settings.spindle.invert.pwm) {
            SPINDLE_PWM_TIMER->CCER |= SPINDLE_PWM_CCER_POL;
            SPINDLE_PWM_TIMER->CR2 |= SPINDLE_PWM_CR2_OIS;
        } else {
            SPINDLE_PWM_TIMER->CCER &= ~SPINDLE_PWM_CCER_POL;
            SPINDLE_PWM_TIMER->CR2 &= ~SPINDLE_PWM_CR2_OIS;
        }
        SPINDLE_PWM_TIMER->CCER |= SPINDLE_PWM_CCER_EN;
        SPINDLE_PWM_TIMER->CR1 |= TIM_CR1_CEN;

    } else {
        if(pwmEnabled)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#endif // DRIVER_SPINDLE_PWM_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { settings.spindle.invert.mask };

    UNUSED(spindle);

    state.on = (SPINDLE_ENABLE_PORT->IDR & SPINDLE_ENABLE_BIT) != 0;
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = (SPINDLE_DIRECTION_PORT->IDR & SPINDLE_DIRECTION_BIT) != 0;
#endif
    state.value ^= settings.spindle.invert.mask;

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = mode.flood;
#ifdef COOLANT_MIST_PIN
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = mode.mist;
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state;

    state.mask = settings.coolant_invert.mask;
    state.flood = (COOLANT_FLOOD_PORT->IDR & COOLANT_FLOOD_BIT) != 0;
#ifdef COOLANT_MIST_PIN
    state.mist  = (COOLANT_MIST_PORT->IDR & COOLANT_MIST_BIT) != 0;
#endif
    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    if(irq_mode == IRQ_Mode_Rising) {
        EXTI->RTSR |= input->bit;
        EXTI->FTSR &= ~input->bit;
    } else if(irq_mode == IRQ_Mode_Falling) {
        EXTI->RTSR &= ~input->bit;
        EXTI->FTSR |= input->bit;
    } else if(irq_mode == IRQ_Mode_Change) {
        EXTI->RTSR |= input->bit;
        EXTI->FTSR |= input->bit;
    } else
        EXTI->IMR &= ~input->bit;   // Disable pin interrupt

    if(irq_mode != IRQ_Mode_None)
        EXTI->IMR |= input->bit;    // Enable pin interrupt
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
#if USE_STEPDIR_MAP
    stepdirmap_init (settings);
#endif

    stepperSetStepOutputs((axes_signals_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

    if(IOInitDone) {

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH
        };

#if DRIVER_SPINDLE_PWM_ENABLE
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif // DRIVER_SPINDLE_PWM_ENABLE

        pulse_length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            pulse_delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 1.1f));
            if(pulse_delay < 2)
                pulse_delay = 2;
            else if(pulse_delay == pulse_length)
                pulse_delay++;
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        } else {
            pulse_delay = 0;
            hal.stepper.pulse_start = &stepperPulseStart;
        }

        PULSE_TIMER->ARR = pulse_length;
        PULSE_TIMER->EGR = TIM_EGR_UG;

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0xFC00
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            input = &inputpin[--i];

            if(input->group != PinGroup_AuxInput)
                input->mode.irq_mode = IRQ_Mode_None;

            switch(input->id) {
#if ESTOP_ENABLE
                case Input_EStop:
                    input->mode.pull_mode = settings->control_disable_pullup.e_stop ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.e_stop ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#else
                case Input_Reset:
                    input->mode.pull_mode = settings->control_disable_pullup.reset ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
                case Input_FeedHold:
                    input->mode.pull_mode = settings->control_disable_pullup.feed_hold ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    input->mode.pull_mode = settings->control_disable_pullup.cycle_start ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

#if N_AXIS > 3
                case Input_LimitA:
                case Input_LimitA_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#if N_AXIS > 4
                case Input_LimitB:
                case Input_LimitB_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#if N_AXIS > 5
                case Input_LimitC:
                case Input_LimitC_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#if !AUX_CONTROLS_ENABLED
                case Input_SafetyDoor:
                    input->mode.pull_mode = settings->control_disable_pullup.safety_door_ajar ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifndef AUX_DEVICES
                case Input_Probe:
                    input->mode.pull_mode = settings->probe.disable_probe_pullup ? PullMode_Down : PullMode_Up;
                    break;

                case Input_MPGSelect:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;

                case Input_I2CStrobe:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
                default:
                    break;
            }

#if AUXINPUT_MASK

            if(input->group == PinGroup_AuxInput) {
                if(input->cap.irq_mode != IRQ_Mode_None) {
                    // Map interrupt to pin
                    uint32_t extireg = AFIO->EXTICR[input->pin >> 2] & ~(0b1111 << ((input->pin & 0b11) << 2));
                    extireg |= ((uint32_t)(GPIO_GET_INDEX(input->port)) << ((input->pin & 0b11) << 2));
                    AFIO->EXTICR[input->pin >> 2] = extireg;
                }
            }

#endif

            GPIO_Init.Pin = input->bit;
            GPIO_Init.Pull = input->mode.pull_mode == PullMode_Up ? GPIO_PULLUP : GPIO_PULLDOWN;

            switch(input->mode.irq_mode) {
                case IRQ_Mode_Rising:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING;
                    break;
                case IRQ_Mode_Falling:
                    GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
                    break;
                case IRQ_Mode_Change:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
                    break;
                default:
                    GPIO_Init.Mode = GPIO_MODE_INPUT;
                    break;
            }
            HAL_GPIO_Init(input->port, &GPIO_Init);

            input->mode.debounce = false;

        } while(i);

        uint32_t irq_mask = DRIVER_IRQMASK|aux_irq;

        __HAL_GPIO_EXTI_CLEAR_IT(irq_mask);

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)
        if(irq_mask & (1<<0)) {
            HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)
        if(irq_mask & (1<<1)) {
            HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)
        if(irq_mask & (1<<2)) {
            HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)
        if(irq_mask & (1<<3)) {
            HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)
        if(irq_mask & (1<<4)) {
            HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0
        if(irq_mask & 0x03E0) {
            HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0xFC00
        if(irq_mask & 0xFC00) {
            HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
#endif

        hal.limits.enable(settings->limits.flags.hard_enabled, (axes_signals_t){0});

#if AUX_CONTROLS_ENABLED
        aux_ctrl_irq_enable(settings, aux_irq_handler);
#endif
    }
}

static char *port2char (GPIO_TypeDef *port)
{
    static char name[3] = "P?";

    name[1] = 'A' + GPIO_GET_INDEX(port);

    return name;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};

    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.description = inputpin[i].description;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

#ifndef STM32F103xB

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);
    } while((ppin = ppin->next));

#endif

#if DRIVER_SPINDLE_PWM_ENABLE
    pin.pin = SPINDLE_PWM_PIN;
    pin.function = Output_SpindlePWM;
    pin.group = PinGroup_SpindlePWM;
    pin.port = low_level ? (void *)SPINDLE_PWM_PORT : (void *)port2char(SPINDLE_PWM_PORT);
    pin.description = NULL;
    pin_info(&pin, data);
#endif
}

#ifndef STM32F103xB

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

#endif

static status_code_t jtag_enable (uint_fast16_t state, char *line)
{
    if(!strcmp(line, "$PGM")) {
        __HAL_AFIO_REMAP_SWJ_ENABLE();
        on_unknown_sys_command = NULL;
        return Status_OK;
    }

    return on_unknown_sys_command ? on_unknown_sys_command(state, line) : Status_Unhandled;
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  //    Interrupt_disableSleepOnIsrExit();

    PULSE_TIMER_CLKEN();
    STEPPER_TIMER_CLKEN();

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {

        if(outputpin[i].group == PinGroup_MotorChipSelect ||
            outputpin[i].group == PinGroup_MotorUART ||
             outputpin[i].id == Output_SPICS ||
              outputpin[i].group == PinGroup_StepperEnable)
            DIGITAL_OUT(outputpin[i].port, outputpin[i].pin, 1);

        GPIO_Init.Pin = 1 << outputpin[i].pin;
        GPIO_Init.Mode = outputpin[i].mode.open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(outputpin[i].port, &GPIO_Init);
    }

 // Stepper init

    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->SR &= ~TIM_SR_UIF;
    STEPPER_TIMER->CNT = 0;
    STEPPER_TIMER->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(STEPPER_TIMER_IRQn, 1);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQn);

    // Single-shot 0.1 us per tick
    PULSE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PULSE_TIMER->PSC = hal.f_step_timer / 10000000UL - 1;
    PULSE_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PULSE_TIMER->CNT = 0;
    PULSE_TIMER->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(PULSE_TIMER_IRQn, 0);
    NVIC_EnableIRQ(PULSE_TIMER_IRQn);

 // Limit pins init

    if(settings->limits.flags.hard_enabled)
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x02, 0x02);

 // Spindle init

#if DRIVER_SPINDLE_PWM_ENABLE

    SPINDLE_PWM_CLOCK_ENA();

#if SPINDLE_PWM_AF_REMAP
    AFIO->MAPR |= (SPINDLE_PWM_AF_REMAP << (4 + 2 * SPINDLE_PWM_TIMER_N));
#endif

    GPIO_Init.Pin = 1 << SPINDLE_PWM_PIN;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(SPINDLE_PWM_PORT, &GPIO_Init);

#endif // DRIVER_SPINDLE_PWM_ENABLE

 // Coolant init (??)

    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = 1;
    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = 0;

#ifdef COOLANT_MIST_PIN
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = 1;
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = 0;
#endif

#if SDCARD_ENABLE

    BITBAND_PERI(SD_CS_PORT->ODR, SD_CS_PIN) = 1;

    sdcard_init();

#endif

    on_unknown_sys_command = grbl.on_unknown_sys_command;
    grbl.on_unknown_sys_command = jtag_enable;

    IOInitDone = settings->version == 22;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
#ifdef HAS_BOOTLOADER
    extern uint8_t _FLASH_VectorTable;
    __disable_irq();
    SCB->VTOR = (uint32_t)&_FLASH_VectorTable;
    __DSB();
    __enable_irq();
#endif

#if MPG_MODE == 1

    // Drive MPG mode input pin low until setup complete

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = 1 << MPG_MODE_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    HAL_GPIO_Init(MPG_MODE_PORT, &GPIO_Init);

    DIGITAL_OUT(MPG_MODE_PORT, MPG_MODE_PIN, 1);

#endif

    // Enable EEPROM and serial port here for the core to be able to configure itself and report any errors

    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); // ??? Disable JTAG and SWD!?? Bug?

#ifdef I2C_PORT
    i2c_init();
#endif

    __HAL_AFIO_REMAP_SWJ_NOJTAG();

#ifndef STM32F103xB
    hal.info = "STM32F103RC";
#else
    hal.info = "STM32F103CB";
#endif
    hal.driver_version = "240220";
    hal.driver_url = GRBL_URL "/STM32F1xx";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTickPrescaled;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#if defined(PROBE_PIN) && !defined(AUX_DEVICES)
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
    hal.probe.connected_toggle = probeConnectedToggle;
    hal.driver_cap.probe_pull_up = On;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_PWM_ENABLE

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
#ifndef STM32F103xB
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;
#endif

#ifdef STM32F103xB
  #if USB_SERIAL_CDC
    stream_connect(usbInit());
  #else
    stream_connect(serialInit(115200));
  #endif
#else
    serialRegisterStreams();
  #if USB_SERIAL_CDC
    stream_connect(usbInit());
  #else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
  #endif
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
    hal.signals_cap.reset = Off;
#endif
#if SAFETY_DOOR_BIT
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;

#ifdef HAS_IOPORTS

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    uint32_t i;
    input_signal_t *input;

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        input->bit = 1 << input->pin;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->user_port = aux_inputs.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->mode.pull_mode = PullMode_Up;
            input->cap.pull_mode = PullMode_UpDown;
            if((input->cap.irq_mode = ((DRIVER_IRQMASK|aux_irq) & input->bit) ? IRQ_Mode_None : IRQ_Mode_Edges) != IRQ_Mode_None) {
                aux_irq |= input->bit;
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
            }
            input->cap.debounce = !!input->cap.irq_mode;
  #if AUX_CONTROLS_ENABLED
            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit(input->port, input->pin, input->user_port, input))) {
                if(aux_remap->function == Input_Probe && input->cap.irq_mode == IRQ_Mode_Edges)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }
  #endif

        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            input->mode.debounce = hal.driver_cap.software_debounce;
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            if(LIMIT_MASK & input->bit) {
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
                input->mode.debounce = hal.driver_cap.software_debounce;
            }
            limit_inputs.n_pins++;
        }
    }

    output_signal_t *output;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins++);
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

  #if AUX_CONTROLS_ENABLED
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
  #elif defined(SAFETY_DOOR_PIN)
    hal.signals_cap.safety_door = On;
  #endif

#else

    uint32_t i;
    input_signal_t *input;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->bit = 1 << input->pin;
        if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            if(LIMIT_MASK & input->bit) {
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
                input->mode.debounce = hal.driver_cap.software_debounce;
            }
            limit_inputs.n_pins++;
        } else if(input->group == PinGroup_Control && (CONTROL_MASK & input->bit)) {
            pin_irq[__builtin_ffs(input->bit) - 1] = input;
#if xSAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                input->mode.debounce = hal.driver_cap.software_debounce;
#endif
        }
    }

#endif // STM32F103xB

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if MPG_MODE == 1
  #if KEYPAD_ENABLE == 2
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, keypad_enqueue_keycode)))
        protocol_enqueue_foreground_task(mpg_enable, NULL);
  #else
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL)))
        protocol_enqueue_foreground_task(mpg_enable, NULL);
  #endif
#elif MPG_MODE == 2
    hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, keypad_enqueue_keycode);
#elif MPG_MODE == 3
    hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#elif KEYPAD_ENABLE == 2
    stream_open_instance(KEYPAD_STREAM, 115200, keypad_enqueue_keycode, "Keypad");
#endif

#include "grbl/plugins_init.h"

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_TIMER_IRQHandler (void)
{
    if ((STEPPER_TIMER->SR & TIM_SR_UIF) != 0)                  // check interrupt source
    {
        STEPPER_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag
        hal.stepper.interrupt_callback();
    }
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/

// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void PULSE_TIMER_IRQHandler (void)
{
    PULSE_TIMER->SR &= ~TIM_SR_UIF;                 // Clear UIF flag

    if (PULSE_TIMER->ARR == pulse_delay)            // Delayed step pulse?
    {
        PULSE_TIMER->ARR = pulse_length;
        stepperSetStepOutputs(next_step_outbits);   // begin step pulse
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    } else
        stepperSetStepOutputs((axes_signals_t){0}); // end step pulse
}

void core_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#ifdef SAFETY_DOOR_PIN
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
         DIGITAL_IN(input->port, input->pin) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1)) {

        if(input->group & (PinGroup_Control)) {
            hal.control.interrupt_callback(systemGetState());
        }
        if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            limit_signals_t state = limitsGetState();
            if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
                hal.limits.interrupt_callback(state);
        }
    }

    EXTI->IMR |= input->bit; // Reenable pin interrupt
}

static inline void core_pin_irq (uint32_t bit)
{
    input_signal_t *input;

    if((input = pin_irq[__builtin_ffs(bit) - 1])) {

        if(input->mode.debounce && task_add_delayed(core_pin_debounce, input, 40)) {
            EXTI->IMR &= ~input->bit; // Disable pin interrupt
#if SAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                debounce.safety_door = input->mode.debounce;
#endif
        } else
            core_pin_debounce(input);
    }
}

#if AUXINPUT_MASK

void aux_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
          DIGITAL_IN(input->port, input->pin) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
        ioports_event(input);

    EXTI->IMR |= input->bit; // Reenable pin interrupt
}

static inline void aux_pin_irq (uint32_t bit)
{
    input_signal_t *input;

    if((input = pin_irq[__builtin_ffs(bit) - 1]) && input->group == PinGroup_AuxInput) {
        if(input->mode.debounce && task_add_delayed(aux_pin_debounce, input, 40)) {
            EXTI->IMR &= ~input->bit; // Disable pin interrupt
#if SAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                debounce.safety_door = input->mode.debounce;
#endif
        } else
            ioports_event(input);
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)

void EXTI0_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<0)
        core_pin_irq(ifg);
#elif LIMIT_MASK & (1<<0)
        core_pin_irq(ifg);
#elif MPG_MODE_BIT && (1<<0)
        protocol_enqueue_foreground_task(mpg_select, NULL);
#elif I2C_STROBE_BIT & (1<<0)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif SPI_IRQ_BIT & (1<<0)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<0)
        aux_pin_irq(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)

void EXTI1_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<1);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<1)
        core_pin_irq(ifg);
#elif LIMIT_MASK & (1<<1)
        core_pin_irq(ifg);
#elif MPG_MODE_BIT && (1<<1)
        protocol_enqueue_foreground_task(mpg_select, NULL);
#elif I2C_STROBE_BIT & (1<<1)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif SPI_IRQ_BIT & (1<<1)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<1)
        aux_pin_irq(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)

void EXTI2_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<2);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<2)
        core_pin_irq(ifg);
#elif LIMIT_MASK & (1<<2)
        core_pin_irq(ifg);
#elif MPG_MODE_BIT && (1<<2)
        protocol_enqueue_foreground_task(mpg_select, NULL);
#elif I2C_STROBE_BIT & (1<<2)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif SPI_IRQ_BIT & (1<<2)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<2)
        aux_pin_irq(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)

void EXTI3_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<3);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<3)
        core_pin_irq(ifg);
#elif LIMIT_MASK & (1<<3)
        core_pin_irq(ifg);
#elif MPG_MODE_BIT && (1<<3)
        protocol_enqueue_foreground_task(mpg_select, NULL);
#elif I2C_STROBE_BIT & (1<<3)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif SPI_IRQ_BIT & (1<<3)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<3)
        aux_pin_irq(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)

void EXTI4_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<4);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<4)
        core_pin_irq(ifg);
#elif LIMIT_MASK & (1<<4)
        core_pin_irq(ifg);
#elif MPG_MODE_BIT && (1<<4)
        protocol_enqueue_foreground_task(mpg_select);
#elif I2C_STROBE_BIT & (1<<4)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif SPI_IRQ_BIT & (1<<4)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<4)
        aux_pin_irq(ifg);
#endif
    }
}

#endif

#if ((DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0)

void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0x03E0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if SPI_IRQ_BIT & 0x03E0
        if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#endif
#if CONTROL_MASK & 0x03E0
        if(ifg & CONTROL_MASK)
            core_pin_irq(ifg);
#endif
#if LIMIT_MASK & 0x03E0
        if(ifg & LIMIT_MASK)
            core_pin_irq(ifg);
#endif
#if I2C_STROBE_BIT & 0x03E0
        if((ifg & I2C_STROBE_BIT) && i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#endif
#if MPG_MODE_BIT & 0x03E0
        if(ifg & MPG_MODE_BIT)
            protocol_enqueue_foreground_task(mpg_select, NULL);
#endif
#if AUXINPUT_MASK & 0x03E0
        if(ifg & aux_irq)
            aux_pin_irq(ifg & aux_irq);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (0xFC00)

void EXTI15_10_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0xFC00);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if SPI_IRQ_BIT & 0xFC00
        if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#endif
#if CONTROL_MASK & 0xFC00
        if(ifg & CONTROL_MASK)
            core_pin_irq(ifg);
#endif
#if LIMIT_MASK & 0xFC00
        if(ifg & LIMIT_MASK)
            core_pin_irq(ifg);
#endif
#if I2C_STROBE_BIT & 0xFC00
        if((ifg & I2C_STROBE_BIT) && i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#endif
#if MPG_MODE_BIT & 0xFC00
        if(ifg & MPG_MODE_BIT)
            protocol_enqueue_foreground_task(mpg_select, NULL);
#endif
#if AUXINPUT_MASK & 0xFC00
        if(ifg & aux_irq)
            aux_pin_irq(ifg & aux_irq);
#endif
    }
}

#endif

// Interrupt handler for 1 ms interval timer
void HAL_IncTick(void)
{
#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
    uwTick += uwTickFreq;

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
