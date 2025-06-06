/*
  svm_map.h - driver code for STM32F103RC ARM processors

  Part of grblHAL

  Copyright (c) 2023 @macbef

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

10 pin IDC connector for RS233, top view

+-----  -----+
|  9 7 5 3 1 |
| 10 8 6 4 2 |
+------------+
 1 5v
 2
 3 PB10 - UART3 TX, via MAX3232, DOUT1
 4
 5 PB11 - UART3 RX, via MAX3232, RIN1
 6
 7
 8
 9 GND
10


J3, top view (not mounted)

+---  ---+
|  2 4 6 |
| 1 3 5  |
+--------+

1 PA9  - UART1 TX
2 PA10 - UART1 RX
3 GND
4 ?
5 3v3
6 BOOT0 (?)

Programming port, top view (not mounted)

+---   ---+
| 1 2 3 4 |
+---------+

1 3v3
2 SWDCK
3 SWDIO
4 GND

*/

#if EEPROM_ENABLE
//#error EEPROM plugin not supported!
#endif

#ifndef STM32F103xE
#error "This board has a STM32F10RCT6 processor, select a corresponding build!"
#endif

#define BOARD_NAME "SVM"
//#define HAS_IOPORTS
#define SERIAL_PORT             1 // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT            3 // GPIOB: TX = 10, RX = 11 - Cannot be enabled if I2C port 2 is enabled!
//#undef MPG_STREAM
//#define MPG_STREAM 1

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


#define A_STEP_PORT            STEP_PORT
#define A_STEP_PIN             6

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

#define AUXOUTPUT0_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT0_PIN          8
#define AUXOUTPUT1_PORT         GPIOB // Spindle direction
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         GPIOB // Spindle enable
#define AUXOUTPUT2_PIN          1
#define AUXOUTPUT3_PORT         GPIOB // Coolant flood
#define AUXOUTPUT3_PIN          3
#define AUXOUTPUT4_PORT         GPIOB // Coolant mist
#define AUXOUTPUT4_PIN          4

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

#define AUXINPUT0_PORT          GPIOC // Safety door
#define AUXINPUT0_PIN           4
#define AUXINPUT1_PORT          GPIOC // Probe
#define AUXINPUT1_PIN           3
#define AUXINPUT2_PORT          GPIOB
#define AUXINPUT2_PIN           15
#define AUXINPUT3_PORT          GPIOC // Reset/EStop
#define AUXINPUT3_PIN           7
#define AUXINPUT4_PORT          GPIOC // Feed hold
#define AUXINPUT4_PIN           6
#define AUXINPUT5_PORT          GPIOC // Cycle start
#define AUXINPUT5_PIN           5

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

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT2_PORT
#define I2C_STROBE_PIN          AUXINPUT2_PIN
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
