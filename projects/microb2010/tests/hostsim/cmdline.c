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
 *  Revision : $Id: cmdline.c,v 1.7 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <hostsim.h>

#include <parse.h>
#include <rdline.h>
#include <uart.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include "main.h"
#include "cmdline.h"
#include "strat_base.h"


/******** See in commands.c for the list of commands. */
extern parse_pgm_ctx_t main_ctx[];

static void write_char(char c) 
{
	uart_send(CMDLINE_UART, c);
}

static void 
valid_buffer(const char *buf, uint8_t size) 
{
	int8_t ret;

	/* reset CTRL-C for trajectory interruption each time we
	 * receive a new command */
	interrupt_traj_reset();

	ret = parse(main_ctx, buf);
	if (ret == PARSE_AMBIGUOUS)
		printf_P(PSTR("Ambiguous command\r\n"));
	else if (ret == PARSE_NOMATCH)
		printf_P(PSTR("Command not found\r\n"));
	else if (ret == PARSE_BAD_ARGS)
		printf_P(PSTR("Bad arguments\r\n"));
}

static int8_t 
complete_buffer(const char *buf, char *dstbuf, uint8_t dstsize,
		int16_t *state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}

/* sending "pop" on cmdline uart resets the robot */
void emergency(char c)
{
	static uint8_t i = 0;

	/* interrupt traj here */
	if (c == '\003')
		interrupt_traj();
	
	if ((i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p')) 
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if (i == 3) {
		hostsim_exit();
		reset();
	}
}

/* log function, add a command to configure
 * it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;
	//	u16 stream_flags = stdout->flags;
	uint8_t i;
	time_h tv;

	if (e->severity > ERROR_SEVERITY_ERROR) {
		if (gen.log_level < e->severity)
			return;
		
		for (i=0; i<NB_LOGS+1; i++)
			if (gen.logs[i] == e->err_num)
				break;
		if (i == NB_LOGS+1)
			return;
	}

	va_start(ap, e);
	tv = time_get_time();
	printf_P(PSTR("%d.%.3d: "), (int)tv.s, (int)(tv.us/1000UL));
	
	printf_P(PSTR("(%d,%d,%d) "),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
	
	vfprintf_P(stdout, e->text, ap);
	printf_P(PSTR("\r\n"));
	va_end(ap);
	//stdout->flags = stream_flags;
}

int cmdline_interact(void)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	rdline_init(&gen.rdl, write_char, valid_buffer, complete_buffer);
	snprintf(gen.prompt, sizeof(gen.prompt), "mainboard > ");	
	rdline_newline(&gen.rdl, gen.prompt);

	while (1) {
		c = uart_recv_nowait(CMDLINE_UART);
		if (c == -1) 
			continue;
		ret = rdline_char_in(&gen.rdl, c);
		if (ret != 2 && ret != 0) {
			buffer = rdline_get_buffer(&gen.rdl);
			history = rdline_get_history_item(&gen.rdl, 0);
			if (history) {
				same = !memcmp(buffer, history, strlen(history)) &&
					buffer[strlen(history)] == '\n';
			}
			else
				same = 0;
			if (strlen(buffer) > 1 && !same)
				rdline_add_history(&gen.rdl, buffer);
			rdline_newline(&gen.rdl, gen.prompt);
		}
	}

	return 0;
}
