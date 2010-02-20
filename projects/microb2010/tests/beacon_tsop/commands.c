/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: commands.c,v 1.7 2009-04-24 19:30:41 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <pid.h>
#include <pwm_ng.h>
#include <parse.h>
#include <rdline.h>
#include <parse_string.h>

#include "main.h"

/**********************************************************/
/* Reset */

/* this structure is filled when cmd_reset is parsed successfully */
struct cmd_reset_result {
	fixed_string_t arg0;
};

/* function called when cmd_reset is parsed successfully */
static void cmd_reset_parsed(__attribute__((unused)) void *parsed_result,
			     __attribute__((unused)) void *data)
{
	reset();
}

prog_char str_reset_arg0[] = "reset";
parse_pgm_token_string_t cmd_reset_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_reset_result, arg0, str_reset_arg0);

prog_char help_reset[] = "Reset the board";
parse_pgm_inst_t cmd_reset = {
	.f = cmd_reset_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_reset,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_reset_arg0, 
		NULL,
	},
};

/**********************************************************/
/* Debug_Frame */

/* this structure is filled when cmd_debug_frame is parsed successfully */
struct cmd_debug_frame_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_debug_frame is parsed successfully */
static void cmd_debug_frame_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	struct cmd_debug_frame_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("on")))
		beacon_tsop.debug_frame = 1;
	else
		beacon_tsop.debug_frame = 0;
}

prog_char str_debug_frame_arg0[] = "debug_frame";
parse_pgm_token_string_t cmd_debug_frame_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_debug_frame_result,
				 arg0, str_debug_frame_arg0);
prog_char str_debug_frame_arg1[] = "on#off";
parse_pgm_token_string_t cmd_debug_frame_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_debug_frame_result,
				 arg1, str_debug_frame_arg1);

prog_char help_debug_frame[] = "Enable frame debug [debug_frame on|off]";
parse_pgm_inst_t cmd_debug_frame = {
	.f = cmd_debug_frame_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_debug_frame,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_debug_frame_arg0, 
		(prog_void *)&cmd_debug_frame_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Debug_Speed */

/* this structure is filled when cmd_debug_speed is parsed successfully */
struct cmd_debug_speed_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_debug_speed is parsed successfully */
static void cmd_debug_speed_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	struct cmd_debug_speed_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("on")))
		beacon_tsop.debug_speed = 1;
	else
		beacon_tsop.debug_speed = 0;
}

prog_char str_debug_speed_arg0[] = "debug_speed";
parse_pgm_token_string_t cmd_debug_speed_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_debug_speed_result,
				 arg0, str_debug_speed_arg0);
prog_char str_debug_speed_arg1[] = "on#off";
parse_pgm_token_string_t cmd_debug_speed_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_debug_speed_result,
				 arg1, str_debug_speed_arg1);

prog_char help_debug_speed[] = "Enable speed debug [debug_speed on|off]";
parse_pgm_inst_t cmd_debug_speed = {
	.f = cmd_debug_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_debug_speed,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_debug_speed_arg0, 
		(prog_void *)&cmd_debug_speed_arg1, 
		NULL,
	},
};

/**********************************************************/

/* in progmem */
parse_pgm_ctx_t main_ctx[] = {

	/* commands_gen.c */
	(parse_pgm_inst_t *)&cmd_reset,
	(parse_pgm_inst_t *)&cmd_debug_frame,
	(parse_pgm_inst_t *)&cmd_debug_speed,
	NULL,
};
