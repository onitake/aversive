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
 *  Revision : $Id: commands_ballboard.c,v 1.2 2009-04-24 19:30:42 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "state.h"
#include "cmdline.h"
#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "actuator.h"

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = 0xFF;
		if (!strcmp_P(res->arg2, PSTR("on")))
			ballboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			ballboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"),
				 (DO_ENCODERS & ballboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"),
				 (DO_CS & ballboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"),
				 (DO_BD & ballboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"),
				 (DO_POWER & ballboard.flags) ? "on":"off");
			printf_P(PSTR("errblock is %s\r\n"),
				 (DO_ERRBLOCKING & ballboard.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;
	else if (!strcmp_P(res->arg1, PSTR("errblock")))
		bit = DO_ERRBLOCKING;


	if (!strcmp_P(res->arg2, PSTR("on")))
		ballboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
			pwm_ng_set(ROLLER_PWM, 0);
			pwm_ng_set(FORKTRANS_PWM, 0);
			pwm_ng_set(FORKROT_PWM, 0);
			pwm_ng_set(BEACON_PWM, 0);
		}
		ballboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1,
		      (bit & ballboard.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#bd#power#errblock";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0,
		(prog_void *)&cmd_event_arg1,
		(prog_void *)&cmd_event_arg2,
		NULL,
	},
};

/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		ballboard.our_color = I2C_COLOR_YELLOW;
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		ballboard.our_color = I2C_COLOR_BLUE;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "blue#yellow";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
	.f = cmd_color_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_color,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_color_arg0,
		(prog_void *)&cmd_color_color,
		NULL,
	},
};


/**********************************************************/
/* State1 */

/* this structure is filled when cmd_state1 is parsed successfully */
struct cmd_state1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_state1 is parsed successfully */
static void cmd_state1_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state1_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("init")))
		state_init();
	else if (!strcmp_P(res->arg1, PSTR("off")))
		state_set_mode(I2C_BALLBOARD_MODE_OFF);
	else if (!strcmp_P(res->arg1, PSTR("eject")))
		state_set_mode(I2C_BALLBOARD_MODE_EJECT);
	else if (!strcmp_P(res->arg1, PSTR("harvest")))
		state_set_mode(I2C_BALLBOARD_MODE_HARVEST);
	else if (!strcmp_P(res->arg1, PSTR("prepare")))
		state_set_mode(I2C_BALLBOARD_MODE_PREP_FORK);
	else if (!strcmp_P(res->arg1, PSTR("take")))
		state_set_mode(I2C_BALLBOARD_MODE_TAKE_FORK);

	/* other commands */
}

prog_char str_state1_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_state1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg0, str_state1_arg0);
prog_char str_state1_arg1[] = "init#eject#harvest#off#prepare#take";
parse_pgm_token_string_t cmd_state1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg1, str_state1_arg1);

prog_char help_state1[] = "set ballboard mode";
parse_pgm_inst_t cmd_state1 = {
	.f = cmd_state1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state1_arg0,
		(prog_void *)&cmd_state1_arg1,
		NULL,
	},
};

/**********************************************************/
/* State2 */

/* this structure is filled when cmd_state2 is parsed successfully */
struct cmd_state2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_state2 is parsed successfully */
static void cmd_state2_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
}

prog_char str_state2_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "xxx";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
prog_char str_state2_arg2[] = "left#right";
parse_pgm_token_string_t cmd_state2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg2, str_state2_arg2);

prog_char help_state2[] = "set ballboard mode";
parse_pgm_inst_t cmd_state2 = {
	.f = cmd_state2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state2_arg0,
		(prog_void *)&cmd_state2_arg1,
		(prog_void *)&cmd_state2_arg2,
		NULL,
	},
};

/**********************************************************/
/* State3 */

/* this structure is filled when cmd_state3 is parsed successfully */
struct cmd_state3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
};

/* function called when cmd_state3 is parsed successfully */
static void cmd_state3_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state3_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("xxx"))) {
		/* xxx = res->arg2 */
	}
	else if (!strcmp_P(res->arg1, PSTR("yyy"))) {
	}
	state_set_mode(0);
}

prog_char str_state3_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "xxx";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, UINT8);

prog_char help_state3[] = "set ballboard mode";
parse_pgm_inst_t cmd_state3 = {
	.f = cmd_state3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state3_arg0,
		(prog_void *)&cmd_state3_arg1,
		(prog_void *)&cmd_state3_arg2,
		NULL,
	},
};

/**********************************************************/
/* State_Machine */

/* this structure is filled when cmd_state_machine is parsed successfully */
struct cmd_state_machine_result {
	fixed_string_t arg0;
};

/* function called when cmd_state_machine is parsed successfully */
static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
				     __attribute__((unused)) void *data)
{
	state_machine();
}

prog_char str_state_machine_arg0[] = "state_machine";
parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);

prog_char help_state_machine[] = "launch state machine";
parse_pgm_inst_t cmd_state_machine = {
	.f = cmd_state_machine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_machine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_machine_arg0,
		NULL,
	},
};

/**********************************************************/
/* State_Debug */

/* this structure is filled when cmd_state_debug is parsed successfully */
struct cmd_state_debug_result {
	fixed_string_t arg0;
	uint8_t on;
};

/* function called when cmd_state_debug is parsed successfully */
static void cmd_state_debug_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	struct cmd_state_debug_result *res = parsed_result;
	state_debug = res->on;
}

prog_char str_state_debug_arg0[] = "state_debug";
parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);

prog_char help_state_debug[] = "Set debug for state machine";
parse_pgm_inst_t cmd_state_debug = {
	.f = cmd_state_debug_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_debug,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_debug_arg0,
		(prog_void *)&cmd_state_debug_on,
		NULL,
	},
};

/**********************************************************/
/* Fork */

/* this structure is filled when cmd_fork is parsed successfully */
struct cmd_fork_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_fork is parsed successfully */
static void cmd_fork_parsed(void *parsed_result,
				    __attribute__((unused)) void *data)
{
	struct cmd_fork_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("deploy")))
		fork_deploy();
	else if (!strcmp_P(res->arg1, PSTR("pack")))
		fork_pack();
	else if (!strcmp_P(res->arg1, PSTR("mid1")))
		fork_mid1();
	else if (!strcmp_P(res->arg1, PSTR("mid2")))
		fork_mid2();
}

prog_char str_fork_arg0[] = "fork";
parse_pgm_token_string_t cmd_fork_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_fork_result, arg0, str_fork_arg0);
prog_char str_fork_arg1[] = "deploy#pack#mid1#mid2";
parse_pgm_token_string_t cmd_fork_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_fork_result, arg1, str_fork_arg1);

prog_char help_fork[] = "move fork";
parse_pgm_inst_t cmd_fork = {
	.f = cmd_fork_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fork,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fork_arg0,
		(prog_void *)&cmd_fork_arg1,
		NULL,
	},
};

/**********************************************************/
/* Roller */

/* this structure is filled when cmd_roller is parsed successfully */
struct cmd_roller_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_roller is parsed successfully */
static void cmd_roller_parsed(void *parsed_result,
				    __attribute__((unused)) void *data)
{
	struct cmd_roller_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("on")))
		roller_on();
	else if (!strcmp_P(res->arg1, PSTR("off")))
		roller_off();
	else if (!strcmp_P(res->arg1, PSTR("reverse")))
		roller_reverse();
}

prog_char str_roller_arg0[] = "roller";
parse_pgm_token_string_t cmd_roller_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_roller_result, arg0, str_roller_arg0);
prog_char str_roller_arg1[] = "on#off#reverse";
parse_pgm_token_string_t cmd_roller_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_roller_result, arg1, str_roller_arg1);

prog_char help_roller[] = "Servo door function";
parse_pgm_inst_t cmd_roller = {
	.f = cmd_roller_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_roller,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_roller_arg0,
		(prog_void *)&cmd_roller_arg1,
		NULL,
	},
};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(void *parsed_result, void *data)
{
	//struct cmd_test_result *res = parsed_result;
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0,
		NULL,
	},
};
