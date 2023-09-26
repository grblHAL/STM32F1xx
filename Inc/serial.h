/*

  serial.h - low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io

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

#include <stdint.h>
#include <stdbool.h>

#include "driver.h"
#include "grbl/stream.h"

//  1 - GPIOA: TX = 9, RX = 10
//  3 - GPIOB: TX = 10, RX = 11
// 31 - GPIOC: TX = 10, RX = 11
//  5 - GPIOC: TX = 12, GPIOD: RX = 2

#ifdef STM32F103xB

#ifdef SERIAL_PORT
const io_stream_t *serialInit (uint32_t baud_rate);
#endif

#ifdef SERIAL1_PORT
const io_stream_t *serial1Init (uint32_t baud_rate);
#endif

#else
void serialRegisterStreams (void);
#endif


/*EOF*/
