/**
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for MarlinSerial.cpp
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* NOTE: written to keep MarlinSerial.h unchanged. Functions and
   variables that probably should be part of the MarlinSerial class
   are written as MarlinSerial_* rather than MarlinSerial::*. Yes,
   an ugly workaround.
*/

#define USB_USES_DTR  1
#define USB_USES_MUX  0

#define MS(x) MarlinSerial_ ##x
#define CS(x) CustomSerial_ ##x

#include "MarlinConfig.h"
#include "MarlinSerial.h"
#include "Marlin.h"

#if ENABLED(EMERGENCY_PARSER)
#include "emergency_parser.h"
static void emergency_parser_update(const uint8_t c)
{
    // declared inline, so wrap it to save space
    emergency_parser.update(c);
}
#define EMERGENCY_PARSER_UPDATE(c) emergency_parser_update(c)
#else
#define EMERGENCY_PARSER_UPDATE(c)
#endif

#if ENABLED(SERIAL_STATS_DROPPED_RX)
uint8_t rx_dropped_bytes = 0;
#define UPDATE_RX_DROPPED_BYTES() \
    if(! ++rx_dropped_bytes) rx_dropped_bytes--
#else
#define UPDATE_RX_DROPPED_BYTES()
#endif

#if ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
uint8_t rx_buffer_overruns = 0;
#define UPDATE_RX_BUFFER_OVERRUNS() \
    if(! ++rx_buffer_overruns) rx_buffer_overruns--
#else
#define UPDATE_RX_BUFFER_OVERRUNS()
#endif

#if ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS)
uint8_t rx_framing_errors = 0;
#define UPDATE_RX_FRAMING_ERRORS() \
    if(! ++rx_framing_errors) rx_framing_errors--
#else
#define UPDATE_RX_FRAMING_ERRORS()
#endif

#if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
ring_buffer_pos_t rx_max_enqueued = 0;
#define UM(n,m) if((n)>m)m=(n)
#define UPDATE_MAX_RX_QUEUED(n) UM(n,rx_max_enqueued)
#else
#define UPDATE_MAX_RX_QUEUED(n)
#endif

#define kUSB     0
#define kUSART1  1
#define kUSART2  2

#if MB(MALYAN_M300)
extern CustomSerial Serial1;
#if !defined(USE_USART2)
#define MARLIN_SERIAL  USB
#define CUSTOM_SERIAL  USART1
#define kMARLIN_SERIAL  kUSB
#define kCUSTOM_SERIAL  kUSART1
#define MULTIPLEX_MARLINSERIAL  USB_USES_MUX
#else
#define MARLIN_SERIAL  USART2
#define CUSTOM_SERIAL  USART1
#define kMARLIN_SERIAL  kUSART2
#define kCUSTOM_SERIAL  kUSART1
#define MULTIPLEX_MARLINSERIAL  0
#endif
#endif

#if kMARLIN_SERIAL == kCUSTOM_SERIAL
#error "MarlinSerial and CustomSerial cannot refer to the same port"
#endif

extern void MS(usart_isr)(void);
extern void CS(usart_isr)(void);

static void MS(process_outgoing)(void);
static void MS(process_incoming)(void);

void ptimer_isr (void)
{
#if kMARLIN_SERIAL == kUSB
    MS(process_outgoing)();
#endif
}

void qtimer_isr (void)
{
#if kMARLIN_SERIAL == kUSB
    MS(process_incoming)();
#endif
}

void usart1_isr (void)
{
#if kMARLIN_SERIAL == kUSART1
    MS(usart_isr)();
#endif
#if kCUSTOM_SERIAL == kUSART1
    CS(usart_isr)();
#endif
}

void usart2_isr (void)
{
#if kMARLIN_SERIAL == kUSART2
    MS(usart_isr)();
#endif
#if kCUSTOM_SERIAL == kUSART2
    CS(usart_isr)();
#endif
}

// workaround for malyanlcd.cpp

void Print::write(const char * s, uint8_t n)
{
    CustomSerial::write(s, n);
}

// MarlinSerial print functions

void MarlinSerial::print(char c, int base)
{
    print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base)
{
    print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base)
{
    print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base)
{
    print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base)
{
    if (base == BYTE) {
	write(n);
	return;
    }
    if (base == DEC) {
	if (n < 0) { print('-'); n = -n; }
	printNumber(n, DEC);
	return;
    }
    printNumber(n, base);
}

void MarlinSerial::print(unsigned long n, int base)
{
    if (base == BYTE) {
	write(n);
	return;
    }
    printNumber(n, base);
}

void MarlinSerial::print(double n, int digits)
{
    printFloat(n, digits);
}

void MarlinSerial::println(void)
{
    print('\r');
    print('\n');
}

void MarlinSerial::println(const String& s)
{
    print(s);
    println();
}

void MarlinSerial::println(const char c[])
{
    print(c);
    println();
}

void MarlinSerial::println(char c, int base)
{
    print(c, base);
    println();
}

void MarlinSerial::println(unsigned char b, int base)
{
    print(b, base);
    println();
}

void MarlinSerial::println(int n, int base)
{
    print(n, base);
    println();
}

void MarlinSerial::println(unsigned int n, int base)
{
    print(n, base);
    println();
}

void MarlinSerial::println(long n, int base)
{
    print(n, base);
    println();
}

void MarlinSerial::println(unsigned long n, int base)
{
    print(n, base);
    println();
}

void MarlinSerial::println(double n, int digits)
{
    print(n, digits);
    println();
}

void MarlinSerial::printNumber(unsigned long n, uint8_t base)
{
    if (n) {
	// enough space for a base 2 representation
	unsigned char s[sizeof(unsigned long)*8];
	int8_t i;
	for (i = 0; n; n /= base) s[i++] = n % base;
	while (i--) print((char) (s[i]+((s[i]<10)?'0':'A'-10)));
    }
    else
	print('0');
}

void MarlinSerial::printFloat(double number, uint8_t digits)
{
    // handle negative numbers
    if (number < 0.0) {
	print('-');
	number = -number;
    }
    // round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) {
	rounding *= 0.1;
    }
    number += rounding;
    // extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double) int_part;
    print(int_part);
    // print the decimal point, but only if there are digits beyond
    if (digits) {
	print('.');
	// extract digits from the remainder one at a time
	while (digits--) {
	    remainder *= 10.0;
	    int toPrint = int(remainder);
	    print(toPrint);
	    remainder -= toPrint;
	}
    }
}

#if defined(MARLIN_SERIAL) && defined(kMARLIN_SERIAL)
#if kMARLIN_SERIAL == kUSB
/**
 * USB flavor of MarlinSerial
 */

#include "usbd_desc.h"

#define USER_BUFFER_SIZE  RX_BUFFER_SIZE
#define RXTX_BUFFER_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
#define USER_BUFFER_MASK  (USER_BUFFER_SIZE-1)
#define RXTX_BUFFER_MASK  (RXTX_BUFFER_SIZE-1)

#if (USER_BUFFER_SIZE < RXTX_BUFFER_SIZE)
#error USER_BUFFER_SIZE should not be less than RXTX_BUFFER_SIZE
#endif

#if !IS_POWER_OF_2(USER_BUFFER_SIZE)
#error USER_BUFFER_SIZE must be a power of 2
#endif

#if !IS_POWER_OF_2(RXTX_BUFFER_SIZE)
#error RXTX_BUFFER_SIZE must be a power of 2
#endif

struct {
    unsigned char buffer[USER_BUFFER_SIZE];
    volatile ring_buffer_pos_t head, tail;
} MS(rx) = { { 0 }, 0, 0 };

unsigned char MS(rx_buffer)[RXTX_BUFFER_SIZE];
static volatile uint8_t * MS(rx_headp) = NULL;
static volatile uint32_t  MS(rx_count) = 0;
unsigned char MS(tx_buffer)[RXTX_BUFFER_SIZE];
volatile ring_buffer_pos_t MS(tx_head) = 0;
volatile ring_buffer_pos_t MS(tx_tail) = 0;


// usb cdc interface

USBD_HandleTypeDef MS(usbd);

#if USB_USES_DTR
volatile uint8_t MS(dtr) = 0;
#define USB_IS_CONNECTED() (MS(dtr))
#else
#define USB_IS_CONNECTED() (MS(usbd).dev_state == USBD_STATE_CONFIGURED)
#endif

USBD_CDC_LineCodingTypeDef MS(lc) = {
    BAUDRATE, // baud rate
    0x00,     // stop bits - 1
    0x00,     // parity - none
    0x08      // data bits - 8
};

int8_t MS(fops_Init)(void)
{
#if USB_USES_DTR
	MS(dtr) = 0;
#endif
    // timer should previously configured and running
    USBD_CDC_SetTxBuffer(&MS(usbd), MS(tx_buffer), 0);
    USBD_CDC_SetRxBuffer(&MS(usbd), MS(rx_buffer));
    return USBD_OK;
}

int8_t MS(fops_DeInit)(void)
{
#if USB_USES_DTR
	MS(dtr) = 0;
#endif
    return USBD_OK;
}

int8_t MS(fops_Control)(uint8_t cmd, uint8_t * pbuf, uint16_t lng)
{
    switch (cmd) {
    case CDC_GET_LINE_CODING:
	pbuf[0] = (uint8_t)(MS(lc).bitrate);
	pbuf[1] = (uint8_t)(MS(lc).bitrate >> 8);
	pbuf[2] = (uint8_t)(MS(lc).bitrate >> 16);
	pbuf[3] = (uint8_t)(MS(lc).bitrate >> 24);
	pbuf[4] = MS(lc).format;
	pbuf[5] = MS(lc).paritytype;
	pbuf[6] = MS(lc).datatype;
	break;
#if USB_USES_DTR
    case CDC_SET_CONTROL_LINE_STATE:
	// "dtr" modem signal (dte is present)
	MS(dtr) = pbuf[0] & 1;
	break;
#endif
#if 0 // unnecessary
    case CDC_SET_LINE_CODING:
	MS(lc).bitrate = (uint32_t)
	    pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24);
	MS(lc).format = pbuf[4];
	MS(lc).paritytype = pbuf[5];
	MS(lc).datatype = pbuf[6];
	break;
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SEND_BREAK:
#endif
    default:
	break;
    }
    return USBD_OK;
}

int8_t MS(fops_Receive)(uint8_t * pbuf, uint32_t * len)
{
    MS(rx_count) = *len;
    MS(rx_headp) = pbuf;
    return USBD_OK;
}

const USBD_CDC_ItfTypeDef MS(fops) = {
    MS(fops_Init),
    MS(fops_DeInit),
    MS(fops_Control),
    MS(fops_Receive)
};

static void MS(process_incoming)(void)
{
    if (! MS(rx_headp))
	// nothing pending in the receiver
	return;

    uint8_t * pbuf = (uint8_t *) MS(rx_headp);
    uint32_t n = MS(rx_count);
    uint32_t v = USER_BUFFER_MASK
	- ((MS(rx).tail - MS(rx).head) & USER_BUFFER_MASK)

    UPDATE_MAX_RX_QUEUED(n);

    while (v && n) {
	uint8_t c = *pbuf++;
	EMERGENCY_PARSER_UPDATE(c);
	MS(rx).buffer[MS(rx).tail] = c;
	MS(rx).tail = (MS(rx).tail+1) & USER_BUFFER_MASK;
	v--;
	n--;
    }
#if ENABLED(SERIAL_STATS_DROPPED_RX)
    if (n) {
	UPDATE_RX_DROPPED_BYTES();
    }
#endif
#if ENABLED(EMERGENCY_PARSER)
    while (n--) {
	uint8_t c = *pbuf++;
	EMERGENCY_PARSER_UPDATE(c);
    }
#endif
    MS(rx_headp) = NULL;
    USBD_CDC_ReceivePacket(&MS(usbd));
}

static void MS(process_outgoing)(void)
{
    if (MS(usbd).dev_state != USBD_STATE_CONFIGURED)
	// wait for the usb driver to be completely configured
	return;

    if ((! MS(usbd).pClassData) ||
	(((USBD_CDC_HandleTypeDef *) MS(usbd).pClassData)->TxState))
	// don't interrupt the usb handler that's transmitting
	return;

    if (MS(tx_head) == MS(tx_tail))
	// there's nothing to transmit
	return;

    if (MS(tx_tail) < MS(tx_head)) {
	if (USBD_CDC_SetTxBuffer(&MS(usbd),
				 &MS(tx_buffer)[MS(tx_head)],
				 RXTX_BUFFER_SIZE - MS(tx_head))
	    == USBD_OK)
	    if (USBD_CDC_TransmitPacket(&MS(usbd)) == USBD_OK)
		MS(tx_head) = 0;
    }
    else {
	if (USBD_CDC_SetTxBuffer(&MS(usbd),
				 &MS(tx_buffer)[MS(tx_head)],
				 MS(tx_tail) - MS(tx_head))
	    == USBD_OK)
	    if (USBD_CDC_TransmitPacket(&MS(usbd)) == USBD_OK)
		MS(tx_head) = MS(tx_tail);
    }
}

void MarlinSerial::begin(const long baud)
{
    MS(lc).bitrate = (uint32_t) baud;
    // timer configured elsewhere
    USBD_Init(&MS(usbd), &VCP_Desc, 0);
    USBD_RegisterClass(&MS(usbd), &USBD_CDC);
    USBD_CDC_RegisterInterface(&MS(usbd), (USBD_CDC_ItfTypeDef *) &MS(fops));
    USBD_Start(&MS(usbd));
    HAL_Delay(20);
}

void MarlinSerial::end(void)
{

}

int MarlinSerial::peek(void)
{
    return (MS(rx).head != MS(rx).tail) ? MS(rx).buffer[MS(rx).head] : -1;
}

int MarlinSerial::read(void)
{
    int c = -1;
    if (MS(rx).head != MS(rx).tail) {
	c = MS(rx).buffer[MS(rx).head];
	MS(rx).head = (MS(rx).head+1) & USER_BUFFER_MASK;
    }
    return c;
}

ring_buffer_pos_t MarlinSerial::available(void)
{
    return (MS(rx).tail - MS(rx).head) & USER_BUFFER_MASK;
}

void MarlinSerial::flush(void)
{
    MS(rx).tail = MS(rx).head;
}

#if OVERLY_SIMPLISTIC_OUTPUT_LOGGING_HACK
#include "cardreader.h"

static SdFile MS(LogFile);

bool MS(log)(const char * s)
{
    SdFile * cwd = card.getWorkDir();

    if (MS(LogFile).isOpen())
	MS(LogFile).close();

    return (s)
	? MS(LogFile).open(cwd, s, O_CREAT | O_WRITE)
	: false;
}
#endif

void MarlinSerial::write(const uint8_t c)
{
    if (USB_IS_CONNECTED()) {
	ring_buffer_pos_t next = (MS(tx_tail)+1) & RXTX_BUFFER_MASK;
	if (next == MS(tx_head)) {
	    HAL_Delay(10);
	}
	if (next != MS(tx_head)) {
	    MS(tx_buffer)[MS(tx_tail)] = c;
	    MS(tx_tail) = next;
	}
    }
#if MULTIPLEX_MARLINSERIAL
    else
	if (MS(usbd).dev_state != USBD_STATE_CONFIGURED)
	    Serial1.write(c);
#endif
#if OVERLY_SIMPLISTIC_OUTPUT_LOGGING_HACK
    if (MS(LogFile).isOpen())
	MS(LogFile).write(c);
#endif
}

void MarlinSerial::flushTX(void)
{
    MS(tx_head) = MS(tx_tail);
}
#endif
#endif


#if defined(MARLIN_SERIAL) && defined(kMARLIN_SERIAL)
#if kMARLIN_SERIAL != kUSB
/**
 * USART flavor of MarlinSerial
 */

#if !IS_POWER_OF_2(RX_BUFFER_SIZE)
#error RX_BUFFER_SIZE must be a power of 2
#endif

#if !IS_POWER_OF_2(TX_BUFFER_SIZE)
#error TX_BUFFER_SIZE must be a power of 2
#endif

struct {
    unsigned char buffer[RX_BUFFER_SIZE];
    volatile ring_buffer_pos_t head, tail;
} MS(rx) = { { 0 }, 0, 0 };

#if TX_BUFFER_SIZE > 0
struct {
    unsigned char buffer[TX_BUFFER_SIZE];
    volatile uint8_t head, tail;
} MS(tx) = { { 0 }, 0, 0 };
#endif

void MarlinSerial::begin(const long baud)
{
    HAL_usart_init(MARLIN_SERIAL, (uint32_t) baud);
}

void MarlinSerial::end(void)
{

}

int MarlinSerial::peek(void)
{
    return (MS(rx).head != MS(rx).tail) ? MS(rx).buffer[MS(rx).head] : -1;
}

int MarlinSerial::read(void)
{
    int c = -1;
    if (MS(rx).head != MS(rx).tail) {
	// could update max_rx_queued in isr
	UPDATE_MAX_RX_QUEUED(available());
	c = MS(rx).buffer[MS(rx).head];
	MS(rx).head = (MS(rx).head+1) & RX_BUFFER_MASK;
    }
    return c;
}

ring_buffer_pos_t MarlinSerial::available(void)
{
    return (MS(rx).tail - MS(rx).head) & RX_BUFFER_MASK;
}

void MarlinSerial::flush(void)
{
    MS(rx).tail = MS(rx).head;
}

void MarlinSerial::write(const uint8_t c)
{
#if TX_BUFFER_SIZE > 0
    ring_buffer_pos_t next = (MS(tx).tail+1) & TX_BUFFER_MASK;
    HAL_usart_txe_1(MARLIN_SERIAL);  // enable txe interrupt
    while (next == MS(tx).head);
    MS(tx).buffer[MS(tx).tail] = c;
    MS(tx).tail = next;
#else
    while(! HAL_usart_check(MARLIN_SERIAL, USART_TXE));
    HAL_usart_send(MARLIN_SERIAL, c);
#endif
}

void MarlinSerial::flushTX(void)
{
#if TX_BUFFER_SIZE > 0
    MS(rx).head = MS(rx).tail;
#endif
}

void MS(usart_isr)(void)
{
    if(HAL_usart_check(MARLIN_SERIAL, USART_RXNE)) {
	uint8_t c = (char) HAL_usart_read(MARLIN_SERIAL);
	ring_buffer_pos_t next = (MS(rx).tail+1) & RX_BUFFER_MASK;
	EMERGENCY_PARSER_UPDATE(c);
	if (next != MS(rx).head) {
	    MS(rx).buffer[MS(rx).tail] = c;
	    MS(rx).tail = next;
	}
#if ENABLED(SERIAL_STATS_DROPPED_RX)
	else {
	    UPDATE_RX_DROPPED_BYTES();
	}
    }
#endif
#if TX_BUFFER_SIZE > 0
    if(HAL_usart_check(MARLIN_SERIAL, USART_TXE)) {
	if (MS(tx).head != MS(tx).tail) {
	    HAL_usart_send(MARLIN_SERIAL, MS(tx).buffer[MS(tx).head]);
	    MS(tx).head = (MS(tx).head+1) & TX_BUFFER_MASK;
	}
	else {
	    // disable txe interrupt
	    HAL_usart_txe_0(MARLIN_SERIAL);
	}
    }
#endif
#if ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
    if(HAL_usart_check(MARLIN_SERIAL, USART_ORE)) {
	UPDATE_RX_BUFFER_OVERUNS();
	HAL_usart_clear(MARLIN_SERIAL, USART_ORE);
    }
#endif
#if ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS)
    if(HAL_usart_check(MARLIN_SERIAL, USART_FE)) {
	UPDATE_RX_FRAMING_ERRORS();
	HAL_usart_clear(MARLIN_SERIAL, USART_FE);
    }
#endif
}
#endif
#endif


#if defined(CUSTOM_SERIAL) && defined(kCUSTOM_SERIAL)
/**
 * CustomSerial
 */

#if MB(MALYAN_M300)
#undef  RX_BUFFER_SIZE
#define RX_BUFFER_SIZE  64
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE-1)
#undef  TX_BUFFER_SIZE
#define TX_BUFFER_SIZE  32
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE-1)
#endif

#if !IS_POWER_OF_2(RX_BUFFER_SIZE)
#error RX_BUFFER_SIZE must be a power of 2
#endif

#if !IS_POWER_OF_2(TX_BUFFER_SIZE)
#error TX_BUFFER_SIZE must be a power of 2
#endif

struct {
    unsigned char buffer[RX_BUFFER_SIZE];
    volatile ring_buffer_pos_t head, tail;
} CS(rx) = { { 0 }, 0, 0 };

#if TX_BUFFER_SIZE > 0
struct {
    unsigned char buffer[TX_BUFFER_SIZE];
    volatile uint8_t head, tail;
} CS(tx) = { { 0 }, 0, 0 };
#endif

void CustomSerial::begin(const long baud)
{
    HAL_usart_init(CUSTOM_SERIAL, (uint32_t) baud);
}

void CustomSerial::end(void)
{

}

int CustomSerial::peek(void)
{
    return (CS(rx).head != CS(rx).tail) ? CS(rx).buffer[CS(rx).head] : -1;
}

int CustomSerial::read(void)
{
    int c = -1;
    if (CS(rx).head != CS(rx).tail) {
	c = CS(rx).buffer[CS(rx).head];
	CS(rx).head = (CS(rx).head+1) & RX_BUFFER_MASK;
    }
    return c;
}

uint16_t CustomSerial::available(void)
{
    return (CS(rx).tail - CS(rx).head) & RX_BUFFER_MASK;
}

void CustomSerial::flush(void)
{
    CS(rx).tail = CS(rx).head;
}

void CustomSerial::flushTX(void)
{
#if TX_BUFFER_SIZE > 0
    CS(rx).head = CS(rx).tail;
#endif
}

void CustomSerial::write(const uint8_t c)
{
#if TX_BUFFER_SIZE > 0
    ring_buffer_pos_t next = (CS(tx).tail+1) & TX_BUFFER_MASK;
    HAL_usart_txe_1(CUSTOM_SERIAL);  // enable txe interrupt
    while (next == CS(tx).head);
    CS(tx).buffer[CS(tx).tail] = c;
    CS(tx).tail = next;
#else
    while(! HAL_usart_check(CUSTOM_SERIAL, USART_TXE));
    HAL_usart_send(CUSTOM_SERIAL, c);
#endif
}

void CustomSerial::write(const char * s)
{
    while (*s) write(*s++);
}

void CustomSerial::write(const uint8_t * b, size_t n)
{
    while (n--) write(*b++);
}

void CS(usart_isr)(void)
{
    if(HAL_usart_check(CUSTOM_SERIAL, USART_RXNE)) {
	uint8_t c = (char) HAL_usart_read(CUSTOM_SERIAL);
#if MULTIPLEX_MARLINSERIAL
	if ((c & 0x80) || (MS(usbd).dev_state != USBD_STATE_CONFIGURED)) {
	    ring_buffer_pos_t next = (CS(rx).tail+1) & RX_BUFFER_MASK;
	    if (next != CS(rx).head) {
		CS(rx).buffer[CS(rx).tail] = c;
		CS(rx).tail = next;
	    }
	}
	else {
	    EMERGENCY_PARSER_UPDATE(c);
	    ring_buffer_pos_t next = (MS(rx).tail+1) & USER_BUFFER_MASK;
	    if (next != MS(rx).head) {
		MS(rx).buffer[MS(rx).tail] = c;
		MS(rx).tail = next;
	    }
	}
#else
	ring_buffer_pos_t next = (CS(rx).tail+1) & RX_BUFFER_MASK;
	if (next != CS(rx).head) {
	    CS(rx).buffer[CS(rx).tail] = c;
	    CS(rx).tail = next;
	}
#endif
    }
#if TX_BUFFER_SIZE > 0
    if(HAL_usart_check(CUSTOM_SERIAL, USART_TXE)) {
	if (CS(tx).head != CS(tx).tail) {
	    HAL_usart_send(CUSTOM_SERIAL, CS(tx).buffer[CS(tx).head]);
	    CS(tx).head = (CS(tx).head+1) & TX_BUFFER_MASK;
	}
	else {
	    // disable txe interrupt
	    HAL_usart_txe_0(CUSTOM_SERIAL);
	}
    }
#endif
}
#endif

/**
 * INSTANTIATE
 */

MarlinSerial customizedSerial;

#if MB(MALYAN_M300)
CustomSerial Serial1;
#endif

/**
 * NOTE: endstop_interrupts.h defines endstop_ISR(), but HAL_stm32.h
 * "preempts" enstop_interrupts.h, so the file and the definition of
 * endstop_ISR() are never really "included". Rather than create yet
 * another file for this simple function, we include our version here.
 */

#include "endstops.h"

void endstop_isr(void)
{
    Endstops::update();
}
