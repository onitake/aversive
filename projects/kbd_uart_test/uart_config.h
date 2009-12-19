// Droids-corp 2004 - Zer0
// config for uart module

/*
 * This is the configuration file for the uart module.
 * This module provides :
 *   - Tx and Rx with fifo
 *   - two modes : the first one tries to be faster ; when you try to
 *     send data and the fifo is full, the byte is dropped. The second
 *     one (with UART1_DONT_LOOSE_DATA defined) can be slower ; when the 
 *     fifo is full, the writing of a data blocks in the interrupt.
 *   - Speed selection (for the moment the module don't use the UBRRxH
 *     register so the speed cannot be too low. (min is 4800 at 16 Mhz)
 *   - Parity selection (if the uC support it)
 *   - 5 to 9 data bits (if the uC support it). Warning : when you use 
 *     9 bits, the prototypes of the functions change (uint8_t become uint16_t).
 *   - 1 or 2 stop bits (if the uC support it).
 *   - 2 UARTs (if the uC support it).
 * 
 *
 *
 * Number of bits in frame for tx and rx are the same
 *
 * It doesn't support some USART capabilities :
 *   - Synchronous mode
 *   - Multiprocessor communication
 */


#ifndef UART_CONFIG_H
#define UART_CONFIG_H

/* 
 * Global configuration (config that is used for each uart)
 */
#define UART_MCU_QUARTZ 16000



/*
 * UART0 definitions 
 */
#define UART0_TX_ENABLED /* enable uart0 emission */
#define UART0_RX_ENABLED /* enable uart0 reception */

/* this means that the function uart_sendchar will block if the fifo is full */
#define UART0_DONT_LOOSE_DATA 

#define UART0_BAUDRATE 9600

/* 
 * if you enable this, the maximum baudrate you can reach is 
 * higher, but the precision is lower. 
 */
#define UART0_USE_DOUBLE_SPEED 0
//#define UART0_USE_DOUBLE_SPEED 1

#define UART0_RX_FIFO_SIZE 16
#define UART0_TX_FIFO_SIZE 16

//#define UART0_NBITS 5
//#define UART0_NBITS 6
//#define UART0_NBITS 7
#define UART0_NBITS 8
//#define UART0_NBITS 9

#define UART0_PARITY UART_PARTITY_NONE
//#define UART0_PARITY UART_PARTITY_ODD
//#define UART0_PARITY UART_PARTITY_EVEN

#define UART0_STOP_BIT 1
//#define UART0_STOP_BIT 2

#endif

