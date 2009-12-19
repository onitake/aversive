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
 *  Revision : $Id: main.c,v 1.2 2009-02-20 21:10:01 zer0 Exp $
 *
 */

/*
 * A simple bootloader example.
 */

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>
#include <uart.h>

#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>
#include <avr/boot.h>

#if defined __AVR_ATmega128__
#define UARTNUM 0
#elif defined __AVR_ATmega2560__
#define UARTNUM 3
#endif


#define NOECHO

#ifdef NOECHO
#define echo(c) do {} while(0)
#else
#define echo(c) uart_send(UARTNUM, c)
#endif

static void bootloader_puts(const char *buf)
{
	while (*buf)
		uart_send(UARTNUM, *(buf++));
}

static uint32_t bootloader_query_hex(void)
{
	char buf[8];
	uint8_t i=0;
	int c;

	memset(buf, 0, sizeof(buf));
	while (1) {
		while ((c=uart_recv_nowait(UARTNUM)) == -1);
		if (i >= sizeof(buf) - 1)
			continue;
		if (c == '\n' || c == '\r') {
			echo('\r');
			echo('\n');
			break;
		}
		echo(c);
		buf[i++] = c;
	}
	return strtol(buf, NULL, 16);
}

static void launch_app(void)
{
	bootloader_puts("BOOT\r\n");
	wait_ms(500);
	MCUCR = (1 << IVCE);
	MCUCR = (0 << IVSEL);
	reset();
}

static void disp_digit(uint8_t x)
{
	if (x < 10)
		uart_send(UARTNUM, '0' + x);
	else
		uart_send(UARTNUM, 'a' - 10 + x);
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

static void crc_app(void)
{
	uint32_t start_addr, addr, size;
	uint8_t c;
	uint16_t crc = 0xffff;

	bootloader_puts("start addr?\r\n");
	start_addr = bootloader_query_hex();
	if (start_addr > FLASHEND)
		goto fail;
	bootloader_puts("size?\r\n");
	size = bootloader_query_hex();
	if (start_addr + size > FLASHEND)
		goto fail;
	for (addr=start_addr; addr<start_addr+size; addr++) {
		c = pgm_read_byte_far(addr);
		crc = _crc_ccitt_update(crc, c);
	}
	disp_hex16(crc);
	return;
 fail:
	bootloader_puts("KO");
}

static void prog_page(void)
{
	int c;
	uint32_t addr;
	uint16_t i;
	uint16_t crc = 0xffff;
	uint8_t buf[SPM_PAGESIZE];

#define SPM_PAGEMASK ((uint32_t)SPM_PAGESIZE-1)
	bootloader_puts("addr?\r\n");
	addr = bootloader_query_hex();
	if (addr > FLASHEND)
		goto fail;
	/* start_addr must be page aligned */
	if (addr & SPM_PAGEMASK)
		goto fail;

	bootloader_puts("addr ok\r\n");

	/* data is received like the .bin format (which is already in
	 * little endian) */
	for (i=0; i<SPM_PAGESIZE; i++) {
		while ((c=uart_recv_nowait(UARTNUM)) == -1);
		crc = _crc_ccitt_update(crc, c);
		buf[i] = c;
	}
	disp_hex16(crc);
	bootloader_puts(" (y to valid)\r\n");
	while ((c=uart_recv_nowait(UARTNUM)) == -1);
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
	
	boot_page_write(addr);
	boot_spm_busy_wait();
	
	/* Reenable RWW-section again. We need this if we want to jump
	 * back to the application after bootloading. */
	boot_rww_enable();
	
	bootloader_puts("OK");
	return;
 fail:
	bootloader_puts("KO");
}

int main(void)
{
	int c;

	uart_init();

	/* move interrupt vector in bootloader section */
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);

	sei();

	bootloader_puts("\r\n");
	while (1) {
		bootloader_puts("cmd>");
		while ((c=uart_recv_nowait(UARTNUM)) == -1);
		if (c == 'x')
			launch_app();
		else if (c == 'c')
			crc_app();
		else if (c == 'p')
			prog_page();
		else
			bootloader_puts("bad cmd (p:prog_page c:crc x:exec)");
		bootloader_puts("\r\n");
	}
	
	return 0;
}
