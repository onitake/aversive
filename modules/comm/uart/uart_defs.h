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

#define UART_TYPE_USART 0
#define UART_TYPE_UART  1
#define UART_TYPE_LIN   2


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


// detect uart type
#if defined(UART0_RX_vect)
	#define UART_TYPE UART_TYPE_UART
#elif defined(UART_RX_vect)
	#define UART_TYPE UART_TYPE_UART
#elif defined(LIN_TC_vect)
	#define UART_TYPE UART_TYPE_LIN
#endif






	// specific for LIN (tested on atmega32m1
#if (UART_TYPE == UART_TYPE_LIN)
	
	// interrupts are not remapped
	
	// registers renamed / mapped
	#define UDR0   LINDAT
	#define UBRR0L LINBRRL
	#define UBRR0H LINBRRH
	#define UCSR0A LINSIR
	#define UCSR0B LINCR
	#define UCSR0C LINENIR // UCSRC stands for LINENIR
	
	// bits renamed / mapped
	#define RXEN LCMD1
	#define TXEN LCMD0
	#define RXC  LRXOK
	#define TXC  LTXOK
	#define UDRE LTXOK
	#define FE   LERR
	#define DOR  LERR
	#define PE   LERR
	#define RXCIE LENRXOK
	#define TXCIE LENTXOK
	#define UDRIE LENTXOK
	
	#define USBS 0
	
	#define UART_HAS_U2X 0
	
	#define REGISTER_FOR_UART_IE ucsrc
#else



#ifdef UCR
#define REGISTER_FOR_UART_IE ucsra
#else
#define REGISTER_FOR_UART_IE ucsrb
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
#ifndef UCSRC
#define UCSRC UCSRB
#endif
#define UCSR0C UCSRC
#endif
#ifndef UBRR0L
#ifndef UBRRL
#define UBRRL UBRR
#endif
#define UBRR0L UBRRL
#endif
#ifndef UBRR0H
#ifndef UBRRH
#define UBRRH UBRRHI
#endif
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



/* if the U2X macro is defined, the uC has the U2X option. */
#ifdef U2X
#define UART_HAS_U2X 1
#else
#define UART_HAS_U2X 0
#endif

#endif /*UART_TYPE!=UART_TYPE_LIN*/

#endif //_UART_DEFS_H_
