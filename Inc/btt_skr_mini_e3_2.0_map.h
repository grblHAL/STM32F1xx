/*
  btt_skr_mini_e3_2.0_map.h - driver code for STM32F103RC ARM processors

  Part of grblHAL

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

#if N_AXIS == 4
#define BOARD_NAME "BTT SKR MINI E3 V2.0 4-axis"
#else
#define BOARD_NAME "BTT SKR MINI E3 V2.0"
#endif

// Define step pulse output pins.
#define STEP_PORT       GPIOB
#define X_STEP_PIN      13 //PB13
#define Y_STEP_PIN      10 //PB10
#define Z_STEP_PIN      0  //PB0
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)

#if N_AXIS > 3
#define A_STEP_PIN      3  //PB3
#define A_STEP_BIT      (1<<A_STEP_PIN)
#endif

#define STEP_OUTMODE GPIO_MAP

// Define step direction output pins.
#define DIRECTION_PORT      GPIOB
#define X_DIRECTION_PIN     12 //PB12
#define Y_DIRECTION_PIN     0  //PB2
#define Z_DIRECTION_PIN     3  //PC3 //********
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)

#if N_AXIS > 3
#define A_DIRECTION_PIN     4  //PB4
#define A_DIRECTION_BIT     (1<<A_DIRECTION_PIN)
#endif

#define DIRECTION_OUTMODE   GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define X_ENABLE_PIN            14 //PB14
#define Y_ENABLE_PIN            11 //PB11
#define Z_ENABLE_PIN            1  //PB1
#define X_ENABLE_BIT            (1<<X_ENABLE_PIN)
#define Y_ENABLE_BIT            (1<<Y_ENABLE_PIN)
#define Z_ENABLE_BIT            (1<<Z_ENABLE_PIN)

#if N_AXIS > 3
#define A_ENABLE_PIN            2  //PD2 //********
#define A_ENABLE_BIT            (1<<A_ENABLE_PIN)
#endif

// Define homing/hard limit switch input pins.
#define LIMIT_PORT      GPIOC
#define X_LIMIT_PIN     0 //PC0
#define Y_LIMIT_PIN     1 //PC1
#define Z_LIMIT_PIN     2 //PC2
#define X_LIMIT_BIT     (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT     (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT     (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE    GPIO_BITBAND

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOC
#define SPINDLE_ENABLE_PIN          6 //PC6
#define SPINDLE_ENABLE_BIT          (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT      GPIOC
#define SPINDLE_DIRECTION_PIN       6 //PC6
#define SPINDLE_DIRECTION_BIT       (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIOC
#define SPINDLE_PWM_PIN             7 //PC7
#define SPINDLE_PWM_BIT             (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOC
#define COOLANT_FLOOD_PIN           8 //PC8
#define COOLANT_FLOOD_BIT           (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT           GPIOC
#define COOLANT_MIST_PIN            9 //PC9
#define COOLANT_MIST_BIT            (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT                GPIOC
#define CONTROL_RESET_PIN           15 //PC15
#define CONTROL_RESET_BIT           (1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_PIN       13 //PC13
#define CONTROL_FEED_HOLD_BIT       (1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_PIN     3 //PC3
#define CONTROL_CYCLE_START_BIT     (1<<CONTROL_CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define CONTROL_SAFETY_DOOR_PIN     12 //PC12
#define CONTROL_SAFETY_DOOR_BIT     (1<<CONTROL_SAFETY_DOOR_PIN)
#else
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT)
#endif
#define CONTROL_INMODE GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT                  GPIOC
#define PROBE_PIN                   14 //PC14
#define PROBE_BIT                   (1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_PORT                 GPIOA
#define KEYPAD_STROBE_PIN           1 //PA1
#define KEYPAD_STROBE_BIT           (1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT  GPIOA
#define SD_CS_PIN   4 //PA4
#define SD_CS_BIT   (1<<SD_CS_PIN)
// The following defines are not used but defined for reference
// Port init and remap is done by HAL_SPI_MspInit() in stm32f1xx_hal_msp.c
#define SD_IO_PORT  GPIOA
#define SD_SCK_PIN  5 //PA5
#define SD_SCK_BIT  (1<<SD_SCK_PIN)
#define SD_MISO_PIN 6 //PA6
#define SD_MISO_BIT (1<<SD_MISO_PIN)
#define SD_MOSI_PIN 7 //PA7
#define SD_MOSI_BIT (1<<SD_MOSI_PIN)
#endif
