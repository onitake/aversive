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
 *  Revision : $Id: cmdline.c,v 1.5 2009-05-02 10:08:09 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <parse.h>
#include <rdline.h>
#include <uart.h>
#include <pwm_ng.h>

#include <pid.h>

#include "main.h"
#include "cmdline.h"


/******** See in commands.c for the list of commands. */
extern parse_pgm_ctx_t main_ctx[];

void write_char(char c) 
{
	uart_send(CMDLINE_UART, c);
}

void valid_buffer(const char *buf, __attribute__((unused)) uint8_t size) 
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

int8_t complete_buffer(const char *buf, char *dstbuf, uint8_t dstsize,
		       int16_t *state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}

int cmdline_process(void)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	while ( (c = uart_recv_nowait(CMDLINE_UART)) != -1) {

		ret = rdline_char_in(&beacon_tsop.rdl, c);
		if (ret != 2 && ret != 0) {
			buffer = rdline_get_buffer(&beacon_tsop.rdl);
			history = rdline_get_history_item(&beacon_tsop.rdl, 0);
			if (history) {
				same = !memcmp(buffer, history, strlen(history)) &&
					buffer[strlen(history)] == '\n';
			}
			else
				same = 0;
			if (strlen(buffer) > 1 && !same)
				rdline_add_history(&beacon_tsop.rdl, buffer);
			rdline_newline(&beacon_tsop.rdl, beacon_tsop.prompt);
		}
	}

	return 0;
}
