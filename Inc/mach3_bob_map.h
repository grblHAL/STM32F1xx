/*
  mach3_bob_map.h - driver code for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2023 @r3l4x-pt

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

/*
LED PC2

10 pin connector from A D to USB top view

1 3 5 7 9
2 4 6 8 10

1 5v
2 gnd
3 PB9
4 PB8
5 PB7
6 PB6
7 *
8 PB4
9 PC12
10 PD2

* Cant find where it connects. probaly a problem with the board and i assume its
PB3 or PB5

*/

#if N_ABC_MOTORS > 1 || N_GANGED
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "MACH3 USB BOB"

#define HAS_BOARD_INIT

// Define step pulse output pins.
#define X_STEP_PORT             GPIOB
#define X_STEP_PIN              1 //PB1
#define Y_STEP_PORT             GPIOC
#define Y_STEP_PIN              5 //PC5
#define Z_STEP_PORT             GPIOA
#define Z_STEP_PIN              7 //PA7
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB
#define X_DIRECTION_PIN         0 //PB0
#define Y_DIRECTION_PORT        GPIOC
#define Y_DIRECTION_PIN         4 //PC4
#define Z_DIRECTION_PORT        GPIOA
#define Z_DIRECTION_PIN         6 //PA6
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     10 //PB10

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12 //PB12
#define Y_LIMIT_PIN             13 //PB13
#define Z_LIMIT_PIN             14 //PB14
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA
#define M3_STEP_PIN             5 //PA5
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        3 //PA3
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            15 //PB15
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOC
#define SPINDLE_ENABLE_PIN      6 //PC6
#define SPINDLE_DIRECTION_PORT  GPIOC
#define SPINDLE_DIRECTION_PIN   7 //PC7

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8 //PA8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOC
#define COOLANT_FLOOD_PIN       8 //PC 8
#define COOLANT_MIST_PORT       GPIOC
#define COOLANT_MIST_PIN        9 //PC 9

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOB
#define RESET_PIN               9 //PB9
#define FEED_HOLD_PIN           8 //PB8
#define CYCLE_START_PIN         7 //PB7
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN         6 //PB6
#endif
#define CONTROL_INMODE          GPIO_SHIFT5

// Define probe switch input pin.
#define PROBE_PORT              GPIOC
#define PROBE_PIN               12 //PC12

/**/
