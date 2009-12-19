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
 *  Revision : $Id: main.c,v 1.1 2009-01-30 20:42:17 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>

#include <uart.h>
#include <parse.h>
#include <rdline.h>
#include <timer.h>
#include <scheduler.h>
#include <spi.h>

#include "main.h"

/* for cmdline interface */
struct rdline rdl;
char prompt[RDLINE_PROMPT_SIZE];
extern parse_pgm_ctx_t main_ctx[];

/******** For cmdline. See in commands.c for the list of commands. */
static void write_char(char c) 
{
	uart_send(0, c);
}

static void 
valid_buffer(const char * buf, uint8_t size) 
{
	int8_t ret;
	ret = parse(main_ctx, buf);
	if (ret == PARSE_AMBIGUOUS)
		printf_P(PSTR("Ambiguous command\r\n"));
	else if (ret == PARSE_NOMATCH)
		printf_P(PSTR("Command not found\r\n"));
	else if (ret == PARSE_BAD_ARGS)
		printf_P(PSTR("Bad arguments\r\n"));
}

static int8_t 
complete_buffer(const char * buf, char * dstbuf, uint8_t dstsize,
		int16_t * state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}

/***********************/

void do_led_blink(void * dummy)
{
#if 1 /* simple blink */
	static uint8_t a=0;

	if(a)
		LED1_ON();
	else
		LED1_OFF();
	
	a = !a;
#endif
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;

	cpt++;
	sei();

	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

/* sending "pop" on uart0 resets the robot */
static void emergency(char c) {
	static uint8_t i = 0;
	
	if( (i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p') )
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if(i == 3)
		reset();
}

int main(void)
{
	int c;
	const char * history;
	int8_t ret;

	/* SPI */
	spi_init(SPI_MODE_MASTER, SPI_FORMAT_2, SPI_CLK_RATE_16);
	spi_set_data_order(SPI_MSB_FIRST);
	spi_register_ss_line(&SS_PORT, SS_BIT);

	/* UART */
	uart_init();
 	fdevopen(uart0_dev_send, uart0_dev_recv);
	uart_register_rx_event(0, emergency);

	/* TIMER */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* SCHEDULER */
	scheduler_init();
	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);
	sei();

	printf_P(PSTR("Coucou\r\n"));
	
	rdline_init(&rdl, write_char, valid_buffer, complete_buffer);
	snprintf(prompt, sizeof(prompt), "ax12 > ");	
	rdline_newline(&rdl, prompt);

	while (1) {
		c = uart_recv_nowait(0);
		if (c == -1) 
			continue;
		ret = rdline_char_in(&rdl, c);
		if (ret != 2 && ret != 0) {
			history = rdline_get_buffer(&rdl);
			if (strlen(history) > 1)
				rdline_add_history(&rdl, history);
			rdline_newline(&rdl, prompt);
		}
	}

	return 0;
}
