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
 *  Revision : $Id: uart_config.h,v 1.3 2009-05-27 20:04:06 zer0 Exp $
 *
 */

/* Droids-corp 2004 - Zer0
 * config for uart module
 */

#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#if UART_NUM == 0

/* compile uart0 fonctions, undefine it to pass compilation */
#define UART0_COMPILE  

/* enable uart0 if == 1, disable if == 0 */
#define UART0_ENABLED  1

/* enable uart0 interrupts if == 1, disable if == 0 */
#define UART0_INTERRUPT_ENABLED  1

#define UART0_BAUDRATE 57600

#define UART0_USE_DOUBLE_SPEED 0

#define UART0_RX_FIFO_SIZE 64
#define UART0_TX_FIFO_SIZE 16

#define UART0_NBITS 8
#define UART0_PARITY UART_PARTITY_NONE
#define UART0_STOP_BIT UART_STOP_BITS_1

#elif UART_NUM == 1

/* compile uart1 fonctions, undefine it to pass compilation */
#define UART1_COMPILE  

/* enable uart1 if == 1, disable if == 0 */
#define UART1_ENABLED  1

/* enable uart1 interrupts if == 1, disable if == 0 */
#define UART1_INTERRUPT_ENABLED  1

#define UART1_BAUDRATE 57600

#define UART1_USE_DOUBLE_SPEED 0

#define UART1_RX_FIFO_SIZE 32
#define UART1_TX_FIFO_SIZE 4

#define UART1_NBITS 8
#define UART1_PARITY UART_PARTITY_NONE
#define UART1_STOP_BIT UART_STOP_BITS_1

#elif UART_NUM == 2

/* compile uart2 fonctions, undefine it to pass compilation */
#define UART2_COMPILE  

/* enable uart2 if == 1, disable if == 0 */
#define UART2_ENABLED  1

/* enable uart2 interrupts if == 1, disable if == 0 */
#define UART2_INTERRUPT_ENABLED  1

#define UART2_BAUDRATE 57600

#define UART2_USE_DOUBLE_SPEED 0

#define UART2_RX_FIFO_SIZE 32
#define UART2_TX_FIFO_SIZE 4

#define UART2_NBITS 8
#define UART2_PARITY UART_PARTITY_NONE
#define UART2_STOP_BIT UART_STOP_BITS_1

#elif UART_NUM == 3

/* compile uart3 fonctions, undefine it to pass compilation */
#define UART3_COMPILE  

/* enable uart3 if == 1, disable if == 0 */
#define UART3_ENABLED  1

/* enable uart3 interrupts if == 1, disable if == 0 */
#define UART3_INTERRUPT_ENABLED  1

#define UART3_BAUDRATE 57600

#define UART3_USE_DOUBLE_SPEED 0

#define UART3_RX_FIFO_SIZE 32
#define UART3_TX_FIFO_SIZE 4

#define UART3_NBITS 8
#define UART3_PARITY UART_PARTITY_NONE
#define UART3_STOP_BIT UART_STOP_BITS_1

#endif /* uart num */

#endif

