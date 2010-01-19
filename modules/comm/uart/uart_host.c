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

/* this file os a stub for host */

void uart_init(void)
{
}

/* global vars are initialized to 0 (NULL) */
event *rx_event[UART_HW_NUM];
event *tx_event[UART_HW_NUM];

int8_t uart_setconf(uint8_t num, struct uart_config *u)
{
	return 0;
}

void uart_getconf(uint8_t num, struct uart_config *u)
{
}

int uart_recv(uint8_t num)
{
	return getchar();
}

int uart_send_nowait(uint8_t num, char c)
{
	return putchar(c);
}

int uart_send(uint8_t num, char c)
{
	return put_char(c);
}

