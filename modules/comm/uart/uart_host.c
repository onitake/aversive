/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: uart_host.c,v 1.3.4.3 2008-12-27 16:29:08 zer0 Exp $
 *
 */

/* Olivier MATZ, Droids-corp 2004 - 2009 */

#include <uart.h>
#include <uart_private.h>

#include <fcntl.h>

/* this file os a stub for host */

void uart_init(void)
{
}

/* global vars are initialized to 0 (NULL) */
event *rx_event[UART_HW_NUM];
event *tx_event[UART_HW_NUM];

void uart_host_rx_event(char c)
{
	/* only one uart */
	if (rx_event[0])
		rx_event[0](c);
}

void uart_host_tx_event(char c)
{
	/* only one uart */
	if (tx_event[0])
		tx_event[0](c);
}

int8_t uart_setconf(uint8_t num, struct uart_config *u)
{
	/* XXX todo */
	return 0;
}

void uart_getconf(uint8_t num, struct uart_config *u)
{
	return;
}

int uart_recv(uint8_t num)
{
	fcntl(0, F_SETFL, 0);
	return getchar();
}

int uart_recv_nowait(uint8_t num)
{
	fcntl(0, F_SETFL, O_NONBLOCK);
	return getchar();
}

int uart_send_nowait(uint8_t num, char c)
{
	return putchar(c);
}

int uart_send(uint8_t num, char c)
{
	return putchar(c);
}
