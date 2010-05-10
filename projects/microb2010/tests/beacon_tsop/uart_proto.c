/*  
 *  Copyright Droids Corporation (2010)
 *  Olivier Matz <zer0@droids-corp.org>
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: main.c,v 1.8 2009-05-02 10:08:09 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/wait.h>

#include <pid.h>
#include <pwm_ng.h>
#include <parse.h>
#include <rdline.h>
#include <uart.h>

#include "cmdline.h"
#include "main.h"

#define UART_NUM 0

#if UART_NUM == 0

#define UCSRxA UCSR0A
#define UCSRxB UCSR0B
#define UCSRxC UCSR0C
#define RXCx RXC0
#define UDRx UDR0
#define UDREx UDRE0
#define U2Xx U2X0
#define RXENx RXEN0
#define TXENx TXEN0
#define UCSZx0 UCSZ00
#define UCSZx1 UCSZ01
#define UBRRxL UBRR0L
#define UBRRxH UBRR0H

#elif  UART_NUM == 1

#define UCSRxA UCSR1A
#define UCSRxB UCSR1B
#define UCSRxC UCSR1C
#define RXCx RXC1
#define UDRx UDR1
#define UDREx UDRE1
#define U2Xx U2X1
#define RXENx RXEN1
#define TXENx TXEN1
#define UCSZx0 UCSZ10
#define UCSZx1 UCSZ11
#define UBRRxL UBRR1L
#define UBRRxH UBRR1H

#endif

void uart_proto_init(void)
{
	UCSRxA = _BV(U2Xx);
	UCSRxB = _BV(RXENx) | _BV(TXENx);
	UCSRxC = _BV(UCSZx1) | _BV(UCSZx0); /* 8 bits no parity 1 stop */
	UBRRxL = 34; /* 57600 at 16 Mhz */
	UBRRxH = 0;
}

static void uart_proto_send(char c)
{
	while ( !( UCSRxA & (1<<UDREx)) ) ;
	UDRx = c;
}

int16_t uart_proto_recv(void)
{
	if ( !(UCSRxA & (1<<RXCx)) )
		return -1;
	return UDRx;
}

/* transmit an integer between 0 and 16384 */
static void xmit_int14(uint16_t x)
{
	uint8_t c;

	c = x & 0x7f;
	c |= 0x80;
	uart_proto_send(c);

	x >>= 7;
	c = x & 0x7f;
	c |= 0x80;
	uart_proto_send(c);
}

void xmit_opp(uint16_t d, uint16_t a)
{
	uart_proto_send(0);
	xmit_int14(d);
	xmit_int14(a);
}

void xmit_static(uint16_t x, uint16_t y, uint16_t a)
{
	uart_proto_send(1);
	xmit_int14(x);
	xmit_int14(y);
	xmit_int14(a);
}
