/*
  my_machine.h - configuration for STM32F103xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied wrranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_CNC3040
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MACH3_BOB
//#define BOARD_BTT_SKR_MINI_E3_V20
//#define BOARD_BTT_SKR_MINI_E3_V20_ALT2
//#define BOARD_SVM
//#define BOARD_CREALITY_V4_2_2 // Not tested!
//#define BOARD_CREALITY_V4_2_7 // Not tested!
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC            1 // Serial communication via native USB. Comment out for UART communication.
#endif
//#define SDCARD_ENABLE           1 // Run gcode programs from SD card, requires sdcard plugin.
//#define MPG_ENABLE              1 // Enable MPG interface. Requires a serial stream and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial stream for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.
//#define DISPLAY_ENABLE          1 // Set to 9 for I2C display protocol, 17 for I2C LED protocol. Other options may be available via plugins.
//#define ODOMETER_ENABLE         1 // Odometer plugin.
//#define TRINAMIC_ENABLE      2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE      5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C            0 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV            1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define ESTOP_ENABLE            0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                                    // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.
// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
// NOTE: STM32F103C8Tx variants only supports the safety door input!
//#define PROBE_ENABLE            0 // Default enabled, remove comment to disable probe input.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED            1
//#define X_AUTO_SQUARE       1
//#define Y_GANGED            1
//#define Y_AUTO_SQUARE       1
//#define Z_GANGED            1
//#define Z_AUTO_SQUARE       1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX    1
//#define Y_GANGED_LIM_MAX    1
//#define Z_GANGED_LIM_MAX    1
//
