/*
 *  Copyright Droids Corporation
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
 *  Revision : $Id: main.c,v 1.4 2009-05-27 20:04:06 zer0 Exp $
 *
 */

/*
 * A simple bootloader example.
 */

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>

#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>
#include <avr/boot.h>

#define BRAKE_DDR()     do { DDRJ |= 0xF0; } while(0)
#define BRAKE_ON()      do { PORTJ |= 0xF0; } while(0)
#define BRAKE_OFF()     do { PORTJ &= 0x0F; } while(0)

#define LED1_ON() 	sbi(PORTJ, 2)
#define LED1_OFF() 	cbi(PORTJ, 2)

#define LED2_ON() 	sbi(PORTJ, 3)
#define LED2_OFF() 	cbi(PORTJ, 3)


#define NOECHO

#ifdef NOECHO
#define echo(c) do {} while(0)
#else
#define echo(c) uart_send(c)
#endif

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
#define UBRRx UBRR0

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
#define UBRRx UBRR1

#elif  UART_NUM == 2

#define UCSRxA UCSR2A
#define UCSRxB UCSR2B
#define UCSRxC UCSR2C
#define RXCx RXC2
#define UDRx UDR2
#define UDREx UDRE2
#define U2Xx U2X2
#define RXENx RXEN2
#define TXENx TXEN2
#define UCSZx0 UCSZ20
#define UCSZx1 UCSZ21
#define UBRRx UBRR2

#elif  UART_NUM == 3

#define UCSRxA UCSR3A
#define UCSRxB UCSR3B
#define UCSRxC UCSR3C
#define RXCx RXC3
#define UDRx UDR3
#define UDREx UDRE3
#define U2Xx U2X3
#define RXENx RXEN3
#define TXENx TXEN3
#define UCSZx0 UCSZ30
#define UCSZx1 UCSZ31
#define UBRRx UBRR3

#endif


static char uart_recv(void)
{
	while ( !(UCSRxA & (1<<RXCx)) ) ;
	return UDRx;
}

static void uart_send(char c)
{
	while ( !( UCSRxA & (1<<UDREx)) ) ;
	UDRx = c;
}

static void uart_puts(const char *buf)
{
	while (*buf)
		uart_send(*(buf++));
}

static int8_t bootloader_query_hex(uint32_t *val)
{
	uint32_t tmp = 0;
	int c;

	while (1) {
		c = uart_recv();
		echo(c);

		if (c == '\n' || c == '\r') {
			*val = tmp;
			return 0;
		}
		else if (c >= '0' && c <= '9') {
			tmp <<= 4;
			tmp += (c - '0');
		}
		else if (c >= 'a' && c <= 'f') {
			tmp <<= 4;
			tmp += (c - 'a' + 10);
		}
		else if (c >= 'A' && c <= 'F') {
			tmp <<= 4;
			tmp += (c - 'A' + 10);
		}
		else
			return -1;
	}
	return 0;
}

/* launch application */
static void launch_app(void)
{
	uart_puts("Boot...");
	MCUCR = (1 << IVCE);
	MCUCR = (0 << IVSEL);
	reset();
}

static void disp_digit(uint8_t x)
{
	if (x < 10)
		x += '0';
	else
		x += 'a' - 10 ;
	uart_send(x);
}

static void disp_hex8(uint8_t x)
{
	disp_digit(x>>4);
	disp_digit(x&0xf);
}

static void disp_hex16(uint16_t x)
{
	disp_hex8(x>>8);
	disp_hex8(x);
}

static void disp_hex32(uint32_t x)
{
	disp_hex16(x>>16);
	disp_hex16(x);
}

static void crc_app(void)
{
	uint32_t start_addr, addr, size;
	uint8_t c;
	uint16_t crc = 0xffff;
	uint16_t sum = 0;

	uart_puts("addr?\r\n");
	if (bootloader_query_hex(&start_addr))
		goto fail;
	if (start_addr > FLASHEND)
		goto fail;
	uart_puts("size?\r\n");
	if (bootloader_query_hex(&size))
		goto fail;
	if (start_addr + size > FLASHEND)
		goto fail;
	for (addr=start_addr; addr<start_addr+size; addr++) {
#if 0
		/* ignore the 2nd page, it contains microb infos */
		if (addr >= 256 && addr < 512)
			continue;
#endif
		c = pgm_read_byte_far(addr);
		crc = _crc_ccitt_update(crc, c);
		sum += c;
	}
	disp_hex16(crc);
	disp_hex16(sum);
	return;
 fail:
	uart_puts("KO");
}

static void read32(void)
{
	uint32_t start_addr, val = 0;
	uint8_t c, i;

	uart_puts("addr?\r\n");
	if (bootloader_query_hex(&start_addr))
		goto fail;
	if (start_addr > FLASHEND)
		goto fail;
	for (i=0; i<4; i++) {
		c = pgm_read_byte_far(start_addr+i);
		val <<= 8;
		val |= c;
	}
	disp_hex32(val);
	return;
 fail:
	uart_puts("KO");
}

static void prog_page(void)
{
	int c;
	uint32_t addr;
	uint16_t i;
	uint16_t crc = 0xffff;
	uint16_t sum = 0;
	uint8_t buf[SPM_PAGESIZE];

#define SPM_PAGEMASK ((uint32_t)SPM_PAGESIZE-1)
	uart_puts("addr?\r\n");
	if (bootloader_query_hex(&addr))
		goto fail;
	if (addr > FLASHEND)
		goto fail;
	/* start_addr must be page aligned */
	if (addr & SPM_PAGEMASK)
		goto fail;

	uart_puts("ok\r\n");

	PORTJ = 0xF0;

	/* data is received like the .bin format (which is already in
	 * little endian) */
	for (i=0; i<SPM_PAGESIZE; i++) {
		c = uart_recv();
		crc = _crc_ccitt_update(crc, c);
		sum += c;
		buf[i] = c;
	}
	disp_hex16(crc);
	disp_hex16(sum);
	uart_puts(" (y?)\r\n");
	c = uart_recv();
	if (c != 'y')
		goto fail;

	/* erase page */
        eeprom_busy_wait();
        boot_page_erase(addr);
        boot_spm_busy_wait();

	/* Set up little-endian word and fill tmp buf. */
	for (i=0; i<SPM_PAGESIZE; i+=2) {
		uint16_t w = buf[i] + ((uint16_t)(buf[i+1]) << 8);
		boot_page_fill(addr + i, w);
	}

	PORTJ = 0xFC;

	boot_page_write(addr);
	boot_spm_busy_wait();

	/* Reenable RWW-section again. We need this if we want to jump
	 * back to the application after bootloading. */
	boot_rww_enable();

	uart_puts("OK");
	return;
 fail:
	uart_puts("KO");
}

int main(void)
{
	int c;
	uint32_t i=0;

	/* disable all motors and switch on leds */
	DDRJ = 0xFC;
	PORTJ = 0xFC;

	UCSRxA = _BV(U2Xx);
	UCSRxB = _BV(RXENx) | _BV(TXENx);
	UCSRxC = _BV(UCSZx1) | _BV(UCSZx0); /* 8 bits no parity 1 stop */
	UBRRx = 34; /* 57600 at 16 Mhz */

	/* move interrupt vector in bootloader section */
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);

	sei();

	uart_puts("\r\ncmd> ");

	/* timeout */
	while ( !(UCSRxA & (1<<RXCx)) ) {
		i++;
		if (i>1000000) /* wait about 1 sec */
			launch_app();
	}

	while (1) {
		uart_puts("\r\ncmd> ");
		c = uart_recv();
		if (c == 'x')
			launch_app();
		else if (c == 'c')
			crc_app();
		else if (c == 'p')
			prog_page();
		else if (c == 'd')
			read32();
		else
			uart_puts("p:prog_page c:crc x:exec d:dump32");
	}

	return 0;
}
