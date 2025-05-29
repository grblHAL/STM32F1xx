/*
  mach3_bob_map.h - driver code for STM32F103RC ARM processors

  Part of grblHAL

  Copyright (c) 2023-2024 @r3l4x-pt

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

/*

LED PC2

10 pin IDC connector, top view

+-----  -----+
|  9 7 5 3 1 |
| 10 8 6 4 2 |
+------------+
 1 5v
 2 gnd
 3 PB8  - feed hold
 4 PB9  - reset or e-stop
 5 PB6  - safety door or aux in 0
 6 PB7  - cycle start
 7 PB4  - probe
 8 PB3  - MPG mode or aux in 1
 9 PD2  - UART5 Rx - remove pull up/pull down resistors!
10 PC12 - UART5 Tx - remove pull up/pull down resistors!

Programming port, top view (not mounted)

+---   ---+
| 1 2 3 4 |
+---------+

1 3v3
2 gnd
3 SWDIO
4 SWDCK

*/

#ifndef STM32F103xE
#error "This board has a STM32F10RCT6 processor, select a corresponding build!"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "MACH3 USB BOB (BSMCE04U)"
//#define BOARD_URL ""

#define SERIAL_PORT             5 // Tx: 10pin IDC, 10 - Rx: 10pin IDC, 9
#define HAS_BOARD_INIT

// Define step pulse output pins.
#define X_STEP_PORT             GPIOB
#define X_STEP_PIN              1 // XP
#define Y_STEP_PORT             GPIOC
#define Y_STEP_PIN              5 // YP
#define Z_STEP_PORT             GPIOA
#define Z_STEP_PIN              7 // ZP
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB
#define X_DIRECTION_PIN         0 // XD
#define Y_DIRECTION_PORT        GPIOC
#define Y_DIRECTION_PIN         4 // YD
#define Z_DIRECTION_PORT        GPIOA
#define Z_DIRECTION_PIN         6 // ZD
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     10 // 74HC541 OE1

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12 // IN1
#define Y_LIMIT_PIN             13 // IN2
#define Z_LIMIT_PIN             14 // IN3
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA
#define M3_STEP_PIN             5  // AP
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        3  // AD
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            15 // IN4
#endif

#define AUXOUTPUT0_PORT         GPIOA // Spindle PWM,  AVI + ACM
#define AUXOUTPUT0_PIN          8
#define AUXOUTPUT1_PORT         GPIOC // Spindle direction, OUT2
#define AUXOUTPUT1_PIN          7
#define AUXOUTPUT2_PORT         GPIOC // Spindle enable, OUT1
#define AUXOUTPUT2_PIN          6
#define AUXOUTPUT3_PORT         GPIOC // Coolant flood, OUT3
#define AUXOUTPUT3_PIN          8
#define AUXOUTPUT4_PORT         GPIOC // Coolant mist, OUT4
#define AUXOUTPUT4_PIN          9

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          GPIOB // Safety door - 10pin IDC, 5
#define AUXINPUT0_PIN           6
#define AUXINPUT1_PORT          GPIOB // 10pin IDC, 8
#define AUXINPUT1_PIN           3
#define AUXINPUT2_PORT          GPIOB // Probe - 10pin IDC, 7
#define AUXINPUT2_PIN           4
#define AUXINPUT3_PORT          GPIOB // Reset/EStop
#define AUXINPUT3_PIN           9
#define AUXINPUT4_PORT          GPIOB // Feed hold
#define AUXINPUT4_PIN           8
#define AUXINPUT5_PORT          GPIOB // Cycle start
#define AUXINPUT5_PIN           7

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT3_PORT
#define RESET_PIN               AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT4_PORT
#define FEED_HOLD_PIN           AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT5_PORT
#define CYCLE_START_PIN         AUXINPUT5_PIN
#endif
#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT2_PORT
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#if MPG_ENABLE == 1
#define MPG_MODE_PORT           AUXINPUT1_PORT
#define MPG_MODE_PIN            AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

/**/
