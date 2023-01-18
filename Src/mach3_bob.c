/*

  mach3_bob.c - driver code for STM32F103C8 ARM processors

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

#include "driver.h"

#ifdef BOARD_MACH3_BOB

void board_init (void)
{
#if USB_SERIAL_CDC
    /* PC11 must be set low to enable USB D+ */
    GPIO_InitTypeDef GPIO_Init = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = GPIO_PIN_11|GPIO_PIN_2,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
#endif
}

#endif

