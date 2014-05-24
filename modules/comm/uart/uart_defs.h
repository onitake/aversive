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
 *  Revision : $Id: uart_defs.h,v 1.2.4.13 2009-06-29 20:28:27 zer0 Exp $
 *
 */

/* Olivier MATZ, Droids-corp 2004 - 2006
 * Uart specific definitions
 */

#ifndef _UART_DEFS_H_
#define _UART_DEFS_H_

#define UART_PARTITY_NONE 0
#define UART_PARTITY_ODD 1
#define UART_PARTITY_EVEN 2

#define UART_STOP_BITS_1 0
#define UART_STOP_BITS_2 1

#if (defined UDR3)
#define UART_HW_NUM 4
#elif (defined UDR2)
#define UART_HW_NUM 3
#elif (defined UDR1)
#define UART_HW_NUM 2
#else /* assume 1 uart */
#define UART_HW_NUM 1
#endif


#if !defined(UDR0) && defined(UDR)
#define UDR0 UDR
#endif
#ifndef UCSR0A
#define UCSR0A UCSRA
#endif
#ifndef UCSR0B
#define UCSR0B UCSRB
#endif
#ifndef UCSR0C
#define UCSR0C UCSRC
#endif
#ifndef UBRR0L
#define UBRR0L UBRRL
#endif
#ifndef UBRR0H
#define UBRR0H UBRRH
#endif
#if !defined(U2X) && defined(U2X0)
#define U2X U2X0
#endif
#if !defined(UCSZ0) && defined(UCSZ00)
#define UCSZ0 UCSZ00
#endif
#if !defined(UCSZ1) && defined(UCSZ01)
#define UCSZ1 UCSZ01
#endif
#if !defined(UCSZ2) && defined(UCSZ02)
#define UCSZ2 UCSZ02
#endif
#if !defined(UPM0) && defined(UPM00)
#define UPM0 UPM00
#endif
#if !defined(UPM1) && defined(UPM01)
#define UPM1 UPM01
#endif
#if !defined(USBS) && defined(USBS0)
#define USBS USBS0
#endif
#if !defined(TXEN) && defined(TXEN0)
#define TXEN TXEN0
#endif
#if !defined(TXCIE) && defined(TXCIE0)
#define TXCIE TXCIE0
#endif
#if !defined(RXEN) && defined(RXEN0)
#define RXEN RXEN0
#endif
#if !defined(RXCIE) && defined(RXCIE0)
#define RXCIE RXCIE0
#endif
#if !defined(TXC) && defined(TXC0)
#define TXC TXC0
#endif
#if !defined(RXC) && defined(RXC0)
#define RXC RXC0
#endif
#if !defined(RXB8) && defined(RXB80)
#define RXB8 RXB80
#endif
#if !defined(UDRIE) && defined(UDRIE0)
#define UDRIE UDRIE0
#endif
#if !defined(UDRE) && defined(UDRE0)
#define UDRE UDRE0
#endif
#if !defined(U2X) && defined(U2X1)
#define U2X U2X1
#endif
#if !defined(UCSZ1) && defined(UCSZ10)
#define UCSZ0 UCSZ10
#endif
#if !defined(UCSZ1) && defined(UCSZ11)
#define UCSZ1 UCSZ11
#endif
#if !defined(UCSZ2) && defined(UCSZ12)
#define UCSZ2 UCSZ12
#endif
#if !defined(UPM1) && defined(UPM10)
#define UPM0 UPM10
#endif
#if !defined(UPM1) && defined(UPM11)
#define UPM1 UPM11
#endif
#if !defined(USBS) && defined(USBS1)
#define USBS USBS1
#endif
#if !defined(TXEN) && defined(TXEN1)
#define TXEN TXEN1
#endif
#if !defined(TXCIE) && defined(TXCIE1)
#define TXCIE TXCIE1
#endif
#if !defined(RXEN) && defined(RXEN1)
#define RXEN RXEN1
#endif
#if !defined(RXCIE) && defined(RXCIE1)
#define RXCIE RXCIE1
#endif
#if !defined(TXC) && defined(TXC1)
#define TXC TXC1
#endif
#if !defined(RXC) && defined(RXC1)
#define RXC RXC1
#endif
#if !defined(RXB8) && defined(RXB81)
#define RXB8 RXB81
#endif
#if !defined(UDRIE) && defined(UDRIE1)
#define UDRIE UDRIE1
#endif
#if !defined(UDRIE) && defined(UDRIE1)
#define UDRIE UDRIE1
#endif
#if !defined(UDRE) && defined(UDRE1)
#define UDRE UDRE1
#endif

/* makes functions more generic, we associate USR and UCR with UCSRA
 * and UCSRB, respectively */
#if ( ! defined UCSRA ) && ( defined USR )
#define UCSRA USR
#endif

#if ( ! defined UCSRB ) && ( defined UCR )
#define UCSRB UCR
#endif

/* UBRR is UBRRL */
#ifndef UBRRL
#define UBRRL UBRR
#endif


/* workaround for libc incomplete headers when using CAN AVR
 * (avr/iocanxx.h): USART is valid.
 * see http://savannah.nongnu.org/bugs/?18964
 */
#if defined (__AVR_AT90CAN128__) || defined (__AVR_AT90CAN64__) || defined (__AVR_AT90CAN32__)

#ifndef SIG_USART0_RECV
#define SIG_USART0_RECV SIG_UART0_RECV
#define SIG_USART1_RECV SIG_UART1_RECV
#define SIG_USART0_DATA SIG_UART0_DATA
#define SIG_USART1_DATA SIG_UART1_DATA
#define SIG_USART0_TRANS SIG_UART0_TRANS
#define SIG_USART1_TRANS SIG_UART1_TRANS
#endif

#endif


/* if the signal USART is defined, the uC has a USART. */
#if defined(UART_TX_vect)
#define UART_IS_USART 0
#else
#define UART_IS_USART 1
#endif

/* if the U2X macro is defined, the uC has the U2X option. */
#ifdef U2X
#define UART_HAS_U2X 1
#else
#define UART_HAS_U2X 0
#endif

#endif //_UART_DEFS_H_
