/*

  serial.c - serial port implementation for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

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

#include "serial.h"

#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"

#ifdef SERIAL_PORT
static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
#ifndef STM32F103xB
static const io_stream_t *serialInit (uint32_t baud_rate);
#endif
#else
#define SERIAL_PORT 0
#endif

#ifdef SERIAL1_PORT
static stream_rx_buffer_t rxbuf1 = {0};
static stream_tx_buffer_t txbuf1 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command1 = protocol_enqueue_realtime_command;
#ifndef STM32F103xB
static const io_stream_t *serial1Init(uint32_t baud_rate);
#endif
#else
#define SERIAL1_PORT 0
#endif

#if SERIAL_PORT

#if SERIAL_PORT == 1
#define USART USART1
#define USART_IRQ USART1_IRQn
#define USART_IRQHandler USART1_IRQHandler
#define USART_CLK HAL_RCC_GetPCLK2Freq
#define USART_CLK_ENABLE __HAL_RCC_USART1_CLK_ENABLE
#else
#define USART UART5
#define USART_IRQ UART5_IRQn
#define USART_IRQHandler UART5_IRQHandler
#define USART_CLK HAL_RCC_GetPCLK1Freq
#define USART_CLK_ENABLE __HAL_RCC_UART5_CLK_ENABLE
#endif

#endif

#if SERIAL1_PORT

#define UART2 USART3
#define UART2_IRQ USART3_IRQn
#define UART2_IRQHandler USART3_IRQHandler
#define UART2_CLK HAL_RCC_GetPCLK1Freq
#define UART2_CLK_ENABLE __HAL_RCC_USART3_CLK_ENABLE

#endif

static io_stream_properties_t serial[] = {
#if SERIAL_PORT
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit
    },
#endif
#if SERIAL1_PORT
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial1Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

#ifndef STM32F103xB

#if SERIAL_PORT

 #if SERIAL_PORT == 1

  static const periph_pin_t tx0 = {
      .function = Output_TX,
      .group = PinGroup_UART1,
      .port  = GPIOA,
      .pin   = 9,
      .mode  = { .mask = PINMODE_OUTPUT },
      .description = "UART1"
  };

  static const periph_pin_t rx0 = {
      .function = Input_RX,
      .group = PinGroup_UART1,
      .port = GPIOA,
      .pin = 10,
      .mode = { .mask = PINMODE_NONE },
      .description = "UART1"
  };

 #else // UART5

  static const periph_pin_t tx0 = {
    .function = Output_TX,
    .group = PinGroup_UART1,
    .port  = GPIOC,
    .pin   = 12,
    .mode  = { .mask = PINMODE_OUTPUT },
    .description = "UART1"
  };

  static const periph_pin_t rx0 = {
    .function = Input_RX,
    .group = PinGroup_UART1,
    .port = GPIOD,
    .pin = 2,
    .mode = { .mask = PINMODE_NONE },
    .description = "UART1"
  };

 #endif

  hal.periph_port.register_pin(&rx0);
  hal.periph_port.register_pin(&tx0);

#endif

#if SERIAL1_PORT

 #if SERIAL1_PORT == 31

  static const periph_pin_t tx1 = {
      .function = Output_TX,
      .group = PinGroup_UART2,
      .port  = GPIOC,
      .pin   = 10,
      .mode  = { .mask = PINMODE_OUTPUT },
      .description = "UART2"
  };

  static const periph_pin_t rx1 = {
      .function = Input_RX,
      .group = PinGroup_UART2,
      .port = GPIOC,
      .pin = 11,
      .mode = { .mask = PINMODE_NONE },
      .description = "UART2"
  };

 #else

  static const periph_pin_t tx1 = {
      .function = Output_TX,
      .group = PinGroup_UART2,
      .port  = GPIOB,
      .pin   = 10,
      .mode  = { .mask = PINMODE_OUTPUT },
      .description = "UART2"
  };

  static const periph_pin_t rx1 = {
      .function = Input_RX,
      .group = PinGroup_UART2,
      .port = GPIOB,
      .pin = 11,
      .mode = { .mask = PINMODE_NONE },
      .description = "UART2"
  };

 #endif

  hal.periph_port.register_pin(&rx1);
  hal.periph_port.register_pin(&tx1);

#endif

#endif // STM32F103xB

    stream_register_streams(&streams);
}

#if SERIAL_PORT || SERIAL1_PORT

static bool serialClaimPort (uint8_t instance)
{
    bool ok = false;
    uint_fast8_t idx = sizeof(serial) / sizeof(io_stream_properties_t);

    do {
        if(serial[--idx].instance == instance) {
            if((ok = serial[idx].flags.claimable && !serial[idx].flags.claimed))
                serial[idx].flags.claimed = On;
            break;
        }

    } while(idx);

    return ok;
}

#endif

#if SERIAL_PORT

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
    uint32_t tail = rxbuf.tail, head = rxbuf.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);    // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                    // While TX buffer full
        if(!hal.stream_blocking_callback())             // check if blocking for space,
            return false;                               // exit if not (leaves TX buffer in an inconsistent state)
    }

    txbuf.data[txbuf.head] = c;                         // Add data to buffer,
    txbuf.head = next_head;                             // update head pointer and
    USART->CR1 |= USART_CR1_TXEIE;                     // enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//

static void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serialTxFlush (void)
{
    USART->CR1 &= ~USART_CR1_TXEIE;     // Disable TX interrupts
    txbuf.tail = txbuf.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serialTxCount (void)
{
    uint32_t tail = txbuf.tail, head = txbuf.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (USART->SR & USART_SR_TC ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
    USART->CR1 = USART_CR1_RE|USART_CR1_TE;
    USART->BRR = UART_BRR_SAMPLING16(USART_CLK(), baud_rate);
    USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)
        USART->CR1 &= ~USART_CR1_RXNEIE;
    else
        USART->CR1 |= USART_CR1_RXNEIE;

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

#ifndef STM32F103xB
static
#endif
const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = On,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_n =  serialWrite,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(!serialClaimPort(stream.instance))
        return NULL;

    USART_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode = GPIO_MODE_AF_PP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };

#if SERIAL_PORT == 1

    GPIO_InitStructure.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

#else // UART5

    GPIO_InitStructure.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

#endif

    serialSetBaudRate(baud_rate);

    HAL_NVIC_SetPriority(USART_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(USART_IRQ);

    return &stream;
}

void USART_IRQHandler (void)
{
    if(USART->SR & USART_SR_RXNE) {
        char data = USART->DR;
        if(!enqueue_realtime_command(data)) {                   // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = data;                  // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
    }

    if((USART->SR & USART_SR_TXE) && (USART->CR1 & USART_CR1_TXEIE)) {
        uint_fast16_t tail = txbuf.tail;            // Get buffer pointer
        USART->DR = txbuf.data[tail];               // Send next character
        txbuf.tail = tail = BUFNEXT(tail, txbuf);   // and increment pointer
        if(tail == txbuf.head)                      // If buffer empty then
            USART->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
   }
}

#endif // SERIAL_PORT

#if SERIAL1_PORT

//
// Returns number of free characters in serial1 input buffer
//
static uint16_t serial1RxFree (void)
{
    uint16_t tail = rxbuf1.tail, head = rxbuf1.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial1RxCount (void)
{
    uint32_t tail = rxbuf1.tail, head = rxbuf1.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial1 input buffer
//
static void serial1RxFlush (void)
{
    rxbuf1.tail = rxbuf1.head;
}

//
// Flushes and adds a CAN character to the serial1 input buffer
//
static void serial1RxCancel (void)
{
    rxbuf1.data[rxbuf1.head] = ASCII_CAN;
    rxbuf1.tail = rxbuf1.head;
    rxbuf1.head = BUFNEXT(rxbuf1.head, rxbuf1);
}

//
// Attempt to send a character bypassing buffering
//
inline static bool serial1PutCNonBlocking (const char c)
{
    bool ok;

    if((ok = !(UART2->CR1 & USART_CR1_TXEIE) && !(UART2->SR & USART_SR_TXE)))
        UART2->DR = c;

    return ok;
}

//
// Writes a character to the serial1 output stream
//
static bool serial1PutC (const char c)
{
//    if(txbuf1.head != txbuf1.tail || !serialPutCNonBlocking(c)) {           // Try to send character without buffering...

        uint16_t next_head = BUFNEXT(txbuf1.head, txbuf1);    // .. if not, get pointer to next free slot in buffer

        while(txbuf1.tail == next_head) {                    // While TX buffer full
            if(!hal.stream_blocking_callback())             // check if blocking for space,
                return false;                               // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf1.data[txbuf1.head] = c;                         // Add data to buffer,
        txbuf1.head = next_head;                             // update head pointer and
        UART2->CR1 |= USART_CR1_TXEIE;                      // enable TX interrupts
//    }

    return true;
}

//
// Writes a null terminated string to the serial1 output stream, blocks if buffer full
//
static void serial1WriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serial1PutC(c);
}

//
// Writes a number of characters from string to the serial1 output stream followed by EOL, blocks if buffer full
//

static void serial1Write(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial1PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial1TxFlush (void)
{
    UART2->CR1 &= ~USART_CR1_TXEIE;     // Disable TX interrupts
    txbuf1.tail = txbuf1.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial1TxCount (void)
{
    uint32_t tail = txbuf1.tail, head = txbuf1.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (UART2->SR & USART_SR_TC ? 0 : 1);
}

//
// serial1GetC - returns -1 if no data available
//
static int16_t serial1GetC (void)
{
    uint_fast16_t tail = rxbuf1.tail;    // Get buffer pointer

    if(tail == rxbuf1.head)
        return -1; // no data available

    char data = rxbuf1.data[tail];       // Get next character
    rxbuf1.tail = BUFNEXT(tail, rxbuf1);  // and update pointer

    return (int16_t)data;
}

static bool serial1SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf1, suspend);
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    UART2->CR1 = USART_CR1_RE|USART_CR1_TE;
    UART2->BRR = UART_BRR_SAMPLING16(UART2_CLK(), baud_rate);
    UART2->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

    return true;
}

static bool serial1Disable (bool disable)
{
    if(disable)
        UART2->CR1 &= ~USART_CR1_RXNEIE;
    else
        UART2->CR1 |= USART_CR1_RXNEIE;

    return true;
}

static bool serial1EnqueueRtCommand (char c)
{
    return enqueue_realtime_command1(c);
}

static enqueue_realtime_command_ptr serial1SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command1;

    if(handler)
        enqueue_realtime_command1 = handler;

    return prev;
}

#ifndef STM32F103xB
static
#endif
const io_stream_t *serial1Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .state.connected = On,
        .read = serial1GetC,
        .write = serial1WriteS,
        .write_n =  serial1Write,
        .write_char = serial1PutC,
        .enqueue_rt_command = serial1EnqueueRtCommand,
        .get_rx_buffer_free = serial1RxFree,
        .get_rx_buffer_count = serial1RxCount,
        .get_tx_buffer_count = serial1TxCount,
        .reset_write_buffer = serial1TxFlush,
        .reset_read_buffer = serial1RxFlush,
        .cancel_read_buffer = serial1RxCancel,
        .suspend_read = serial1SuspendInput,
        .disable_rx = serial1Disable,
        .set_baud_rate = serial1SetBaudRate,
        .set_enqueue_rt_handler = serial1SetRtHandler
    };

    if(!serialClaimPort(stream.instance))
        return NULL;

    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode = GPIO_MODE_AF_PP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };

    UART2_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

#if SERIAL1_PORT == 31
    __HAL_AFIO_REMAP_USART3_PARTIAL();
#else
    __HAL_AFIO_REMAP_USART3_DISABLE();
#endif

    serial1SetBaudRate(115200);

    HAL_NVIC_SetPriority(UART2_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(UART2_IRQ);

    return &stream;
}

void UART2_IRQHandler (void)
{
    if(UART2->SR & USART_SR_RXNE) {
        char data = UART2->DR;
        if(!enqueue_realtime_command1(data)) {                   // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf1.head, rxbuf1);   // Get and increment buffer pointer
            if(next_head == rxbuf1.tail)                         // If buffer full
                rxbuf1.overflow = 1;                             // flag overflow
            else {
                rxbuf1.data[rxbuf1.head] = data;                 // if not add data to buffer
                rxbuf1.head = next_head;                         // and update pointer
            }
        }
    }

    if((UART2->SR & USART_SR_TXE) && (UART2->CR1 & USART_CR1_TXEIE)) {
        uint_fast16_t tail = txbuf1.tail;           // Get buffer pointer
        UART2->DR = txbuf1.data[tail];              // Send next character
        txbuf1.tail = tail = BUFNEXT(tail, txbuf1); // and increment pointer
        if(tail == txbuf1.head)                     // If buffer empty then
            UART2->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
   }
}

#endif // SERIAL2_PORT
