/*
  creality_v2.2.x_map.h - driver code for STM32F103RC ARM processors

  Part of grblHAL

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

#ifndef STM32F103xE
#error "This board has a STM32F10RCT6 processor, select a corresponding build!"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#ifdef BOARD_CREALITY_V2_2_7
#define BOARD_NAME "Creality v2.2.7"
#else
#define BOARD_NAME "Creality v2.2.2"
#endif

//#define I2C_PORT        1 ??
#define SERIAL_PORT     1 // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT    3 // GPIOB: TX = 10, RX = 11 - to Trinamic drivers

#ifdef BOARD_CREALITY_V2_2_7

// Define step pulse output pins.
#define X_STEP_PORT             GPIOB
#define X_STEP_PIN              9
#define Y_STEP_PORT             GPIOB
#define Y_STEP_PIN              7
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              5
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOC
#define X_DIRECTION_PIN         2
#define Y_DIRECTION_PORT        GPIOB
#define Y_DIRECTION_PIN         8
#define Z_DIRECTION_PORT        GPIOB
#define Z_DIRECTION_PIN         6
#define DIRECTION_OUTMODE       GPIO_BITBAND

#else

// Define step pulse output pins.
#define X_STEP_PORT             GPIOC
#define X_STEP_PIN              2
#define Y_STEP_PORT             GPIOB
#define Y_STEP_PIN              9
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              6
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB
#define X_DIRECTION_PIN         9
#define Y_DIRECTION_PORT        GPIOB
#define Y_DIRECTION_PIN         7
#define Z_DIRECTION_PORT        GPIOB
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_BITBAND

#endif

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOC
#define STEPPERS_ENABLE_PIN     3

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOA
#define X_LIMIT_PIN             5
#define Y_LIMIT_PIN             6
#define Z_LIMIT_PIN             7
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#ifdef BOARD_CREALITY_V2_2_7
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             3
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        4
#else
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             4
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        3
#endif
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         1
#else
#define AUXOUTPUT0_PORT         GPIOA
#define AUXOUTPUT0_PIN          1
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  GPIOC
#define SPINDLE_DIRECTION_PIN   6
#else
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          6
#endif

#if DRIVER_SPINDLE_ENABLE // FAN1
#define SPINDLE_ENABLE_PORT     GPIOC
#define SPINDLE_ENABLE_PIN      7
#else
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          7
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOA
#define COOLANT_FLOOD_PIN       2
#define COOLANT_MIST_PORT       GPIOA
#define COOLANT_MIST_PIN        0

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOB
#define RESET_PIN               15
#define FEED_HOLD_PIN           13
#define CYCLE_START_PIN         12
#define CONTROL_INMODE GPIO_MAP

#define AUXINPUT0_PORT          GPIOC
#define AUXINPUT0_PIN           6
#define AUXINPUT1_PORT          GPIOB
#define AUXINPUT1_PIN           2
#define AUXINPUT2_PORT          GPIOB
#define AUXINPUT2_PIN           1

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT2_PORT
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT1_PORT
#define I2C_STROBE_PIN          AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif
