/*
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: commands_cs.c,v 1.4 2009-05-02 10:08:09 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_ng.h>
#include <pid.h>
#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cmdline.h"

extern uint32_t cs_consign;

/**********************************************************/
/* Gains for control system */

/* this structure is filled when cmd_gain is parsed successfully */
struct cmd_gain_result {
	fixed_string_t arg0;
	int16_t p;
	int16_t i;
	int16_t d;
};

/* function called when cmd_gain is parsed successfully */
static void cmd_gain_parsed(void * parsed_result, void *show)
{
	struct cmd_gain_result *res = parsed_result;

	if (!show) 
		pid_set_gains(&beacon_tsop.pid, res->p, res->i, res->d);

	printf_P(PSTR("gain %d %d %d\r\n"),
		 pid_get_gain_P(&beacon_tsop.pid),
		 pid_get_gain_I(&beacon_tsop.pid),
		 pid_get_gain_D(&beacon_tsop.pid));
}

prog_char str_gain_arg0[] = "gain";
parse_pgm_token_string_t cmd_gain_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_gain_result,
				 arg0, str_gain_arg0);
parse_pgm_token_num_t cmd_gain_p = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, p, INT16);
parse_pgm_token_num_t cmd_gain_i = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, i, INT16);
parse_pgm_token_num_t cmd_gain_d = TOKEN_NUM_INITIALIZER(struct cmd_gain_result, d, INT16);

prog_char help_gain[] = "Set gain values for PID";
parse_pgm_inst_t cmd_gain = {
	.f = cmd_gain_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_gain,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_gain_arg0, 
		(prog_void *)&cmd_gain_p, 
		(prog_void *)&cmd_gain_i, 
		(prog_void *)&cmd_gain_d, 
		NULL,
	},
};

/* show */
/* this structure is filled when cmd_gain is parsed successfully */
struct cmd_gain_show_result {
	fixed_string_t arg0;
	fixed_string_t show;
};

prog_char str_gain_show_arg[] = "show";
parse_pgm_token_string_t cmd_gain_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_gain_show_result, show, str_gain_show_arg);

prog_char help_gain_show[] = "Show gain values for PID";
parse_pgm_inst_t cmd_gain_show = {
	.f = cmd_gain_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_gain_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_gain_arg0, 
		(prog_void *)&cmd_gain_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Derivate_Filters for control system */

/* this structure is filled when cmd_derivate_filter is parsed successfully */
struct cmd_derivate_filter_result {
	fixed_string_t arg0;
	uint8_t size;
};

/* function called when cmd_derivate_filter is parsed successfully */
static void cmd_derivate_filter_parsed(void *parsed_result, void *show)
{
	struct cmd_derivate_filter_result * res = parsed_result;

	if (!show) 
		pid_set_derivate_filter(&beacon_tsop.pid, res->size);

	printf_P(PSTR("derivate_filter %u\r\n"), 
		 pid_get_derivate_filter(&beacon_tsop.pid));
}

prog_char str_derivate_filter_arg0[] = "derivate_filter";
parse_pgm_token_string_t cmd_derivate_filter_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_derivate_filter_result,
				 arg0, str_derivate_filter_arg0);
parse_pgm_token_num_t cmd_derivate_filter_size =
	TOKEN_NUM_INITIALIZER(struct cmd_derivate_filter_result, size, UINT32);

prog_char help_derivate_filter[] = "Set derivate_filter values for PID (in, I, out)";
parse_pgm_inst_t cmd_derivate_filter = {
	.f = cmd_derivate_filter_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_derivate_filter,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_derivate_filter_arg0, 
		(prog_void *)&cmd_derivate_filter_size, 
		NULL,
	},
};

/* show */

struct cmd_derivate_filter_show_result {
	fixed_string_t arg0;
	fixed_string_t show;
};

prog_char str_derivate_filter_show_arg[] = "show";
parse_pgm_token_string_t cmd_derivate_filter_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_derivate_filter_show_result, show, str_derivate_filter_show_arg);

prog_char help_derivate_filter_show[] = "Show derivate_filter values for PID";
parse_pgm_inst_t cmd_derivate_filter_show = {
	.f = cmd_derivate_filter_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_derivate_filter_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_derivate_filter_arg0, 
		(prog_void *)&cmd_derivate_filter_show_arg,
		NULL,
	},
};


/**********************************************************/
/* Maximums for control system */

/* this structure is filled when cmd_maximum is parsed successfully */
struct cmd_maximum_result {
	fixed_string_t arg0;
	uint32_t in;
	uint32_t i;
	uint32_t out;
};

/* function called when cmd_maximum is parsed successfully */
static void cmd_maximum_parsed(void *parsed_result, void *show)
{
	struct cmd_maximum_result * res = parsed_result;
	
	if (!show)
		pid_set_maximums(&beacon_tsop.pid, res->in, res->i, res->out);

	printf_P(PSTR("maximum %lu %lu %lu\r\n"), 
		 pid_get_max_in(&beacon_tsop.pid),
		 pid_get_max_I(&beacon_tsop.pid),
		 pid_get_max_out(&beacon_tsop.pid));
}

prog_char str_maximum_arg0[] = "maximum";
parse_pgm_token_string_t cmd_maximum_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_maximum_result,
				 arg0, str_maximum_arg0);
parse_pgm_token_num_t cmd_maximum_in =
	TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, in, UINT32);
parse_pgm_token_num_t cmd_maximum_i =
	TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, i, UINT32);
parse_pgm_token_num_t cmd_maximum_out =
	TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, out, UINT32);

prog_char help_maximum[] = "Set maximum values for PID (in, I, out)";
parse_pgm_inst_t cmd_maximum = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_maximum,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_maximum_in, 
		(prog_void *)&cmd_maximum_i, 
		(prog_void *)&cmd_maximum_out, 
		NULL,
	},
};

/* show */

/* this structure is filled when cmd_maximum is parsed successfully */
struct cmd_maximum_show_result {
	fixed_string_t arg0;
	fixed_string_t show;
};
prog_char str_maximum_show_arg[] = "show";
parse_pgm_token_string_t cmd_maximum_show_arg =
	TOKEN_STRING_INITIALIZER(struct cmd_maximum_show_result, show, str_maximum_show_arg);

prog_char help_maximum_show[] = "Show maximum values for PID";
parse_pgm_inst_t cmd_maximum_show = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_maximum_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_maximum_show_arg,
		NULL,
	},
};


/**********************************************************/
/* Consigns for control system */

/* this structure is filled when cmd_consign is parsed successfully */
struct cmd_consign_result {
	fixed_string_t arg0;
	uint32_t cons;
};

/* function called when cmd_consign is parsed successfully */
static void cmd_consign_parsed(void *parsed_result, void *show)
{
	struct cmd_consign_result * res = parsed_result;
	
	if (!show)
		cs_consign = res->cons;

	printf_P(PSTR("consign %lu\r\n"), cs_consign);
}

prog_char str_consign_arg0[] = "consign";
parse_pgm_token_string_t cmd_consign_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_consign_result,
				 arg0, str_consign_arg0);
parse_pgm_token_num_t cmd_consign_cons =
	TOKEN_NUM_INITIALIZER(struct cmd_consign_result, cons, UINT32);

prog_char help_consign[] = "Set consign";
parse_pgm_inst_t cmd_consign = {
	.f = cmd_consign_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_consign,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_consign_arg0, 
		(prog_void *)&cmd_consign_cons, 
		NULL,
	},
};

/* show */

/* this structure is filled when cmd_consign is parsed successfully */
struct cmd_consign_show_result {
	fixed_string_t arg0;
	fixed_string_t show;
};
prog_char str_consign_show_arg[] = "show";
parse_pgm_token_string_t cmd_consign_show_arg =
	TOKEN_STRING_INITIALIZER(struct cmd_consign_show_result, show, str_consign_show_arg);

prog_char help_consign_show[] = "Show consign values for PID";
parse_pgm_inst_t cmd_consign_show = {
	.f = cmd_consign_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_consign_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_consign_arg0, 
		(prog_void *)&cmd_consign_show_arg,
		NULL,
	},
};

