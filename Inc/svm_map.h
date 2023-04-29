/*
  svm_map.h - driver code for STM32F103RC ARM processors

  Part of grblHAL

  Copyright (c) 2023 @macbef

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

#if EEPROM_ENABLE
#error EEPROM plugin not supported!
#endif

#ifndef STM32F103xE
#error "This board has a STM32F10RCT6 processor, select a corresponding build!"
#endif

#define BOARD_NAME "SVM"

#if N_ABC_MOTORS > 3
#error Axis configuration is not supported!
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              2
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PIN         3
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     9

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            15 //PB15
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOB
#define M4_STEP_PIN             5
#define M4_DIRECTION_PORT       GPIOB
#define M4_DIRECTION_PIN        6
#define M4_LIMIT_PORT           GPIOB
#define M4_LIMIT_PIN            8 //PB8
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PORT            GPIOC
#define M5_STEP_PIN             0
#define M5_DIRECTION_PORT       GPIOC
#define M5_DIRECTION_PIN        1
#define M5_LIMIT_PORT           GPIOC
#define M5_LIMIT_PIN            2
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      1
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   0

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       3
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOC
#define RESET_PIN               7
#define FEED_HOLD_PIN           6
#define CYCLE_START_PIN         5
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         4
#endif
#define CONTROL_INMODE          GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT              GPIOC
#define PROBE_PIN               3

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIOB
#define I2C_STROBE_PIN          15
#endif

#if SDCARD_ENABLE
#define SPI1_REMAP
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               3
// The following defines are not used but defined for reference
// Port init and remap is done by HAL_SPI_MspInit() in stm32f1xx_hal_msp.c
#define SD_IO_PORT              GPIOB
#define SD_SCK_PIN              3
#define SD_MISO_PIN             4
#define SD_MOSI_PIN             5
#endif
