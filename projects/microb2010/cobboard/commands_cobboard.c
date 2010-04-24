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
 *  Revision : $Id: commands_cobboard.c,v 1.6 2009-11-08 17:25:00 zer0 Exp $
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

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "state.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "spickle.h"
#include "shovel.h"

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = 0xFF;
		if (!strcmp_P(res->arg2, PSTR("on")))
			cobboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			cobboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"),
				 (DO_ENCODERS & cobboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"),
				 (DO_CS & cobboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"),
				 (DO_BD & cobboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"),
				 (DO_POWER & cobboard.flags) ? "on":"off");
			printf_P(PSTR("errblock is %s\r\n"),
				 (DO_ERRBLOCKING & cobboard.flags) ? "on":"off");
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
		cobboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
			pwm_ng_set(LEFT_SPICKLE_PWM, 0);
			pwm_ng_set(RIGHT_SPICKLE_PWM, 0);
			pwm_ng_set(SHOVEL_PWM, 0);
		}
		cobboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1,
		      (bit & cobboard.flags) ? "on":"off");
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
static void cmd_color_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		cobboard.our_color = I2C_COLOR_YELLOW;
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		cobboard.our_color = I2C_COLOR_BLUE;
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
	else if (!strcmp_P(res->arg1, PSTR("eject")))
		state_set_mode(I2C_COBBOARD_MODE_EJECT);
	else if (!strcmp_P(res->arg1, PSTR("ignore_i2c")))
		state_set_i2c_ignore(1);
	else if (!strcmp_P(res->arg1, PSTR("care_i2c")))
		state_set_i2c_ignore(0);

	/* other commands */
}

prog_char str_state1_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_state1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg0, str_state1_arg0);
prog_char str_state1_arg1[] = "init#eject#ignore_i2c#care_i2c";
parse_pgm_token_string_t cmd_state1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg1, str_state1_arg1);

prog_char help_state1[] = "set cobboard mode";
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
	struct cmd_state2_result *res = parsed_result;
	uint8_t side;

	if (!strcmp_P(res->arg2, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else
		side = I2C_RIGHT_SIDE;

	if (!strcmp_P(res->arg1, PSTR("pack"))) {
		state_set_mode(I2C_COBBOARD_MODE_HARVEST);
		state_set_spickle(side, 0);
	}
	else if (!strcmp_P(res->arg1, PSTR("deploy"))) {
		state_set_mode(I2C_COBBOARD_MODE_HARVEST);
		state_set_spickle(side, I2C_COBBOARD_SPK_DEPLOY);
	}
	else if (!strcmp_P(res->arg1, PSTR("harvest"))) {
		state_set_mode(I2C_COBBOARD_MODE_HARVEST);
		state_set_spickle(side, I2C_COBBOARD_SPK_DEPLOY |
				  I2C_COBBOARD_SPK_AUTOHARVEST);
	}
}

prog_char str_state2_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "harvest#deploy#pack";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
prog_char str_state2_arg2[] = "left#right";
parse_pgm_token_string_t cmd_state2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg2, str_state2_arg2);

prog_char help_state2[] = "set cobboard mode";
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

prog_char str_state3_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "xxx";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, UINT8);

prog_char help_state3[] = "set cobboard mode";
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
/* Servo_Door */

/* this structure is filled when cmd_servo_door is parsed successfully */
struct cmd_servo_door_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_servo_door is parsed successfully */
static void cmd_servo_door_parsed(void *parsed_result,
				    __attribute__((unused)) void *data)
{
	struct cmd_servo_door_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("open")))
		servo_door_open();
	else if (!strcmp_P(res->arg1, PSTR("closed")))
		servo_door_close();
	else if (!strcmp_P(res->arg1, PSTR("block")))
		servo_door_close();
}

prog_char str_servo_door_arg0[] = "door";
parse_pgm_token_string_t cmd_servo_door_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_servo_door_result, arg0, str_servo_door_arg0);
prog_char str_servo_door_arg1[] = "open#closed#block";
parse_pgm_token_string_t cmd_servo_door_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_servo_door_result, arg1, str_servo_door_arg1);

prog_char help_servo_door[] = "Servo door function";
parse_pgm_inst_t cmd_servo_door = {
	.f = cmd_servo_door_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_servo_door,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_servo_door_arg0,
		(prog_void *)&cmd_servo_door_arg1,
		NULL,
	},
};

/**********************************************************/
/* cobroller */

/* this structure is filled when cmd_cobroller is parsed successfully */
struct cmd_cobroller_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_cobroller is parsed successfully */
static void cmd_cobroller_parsed(void *parsed_result,
				    __attribute__((unused)) void *data)
{
	struct cmd_cobroller_result *res = parsed_result;
	uint8_t side;

	if (!strcmp_P(res->arg1, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else
		side = I2C_RIGHT_SIDE;

	if (!strcmp_P(res->arg2, PSTR("on")))
		cobroller_on(side);
	else if (!strcmp_P(res->arg2, PSTR("off")))
		cobroller_off(side);
}

prog_char str_cobroller_arg0[] = "cobroller";
parse_pgm_token_string_t cmd_cobroller_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cobroller_result, arg0, str_cobroller_arg0);
prog_char str_cobroller_arg1[] = "left#right";
parse_pgm_token_string_t cmd_cobroller_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cobroller_result, arg1, str_cobroller_arg1);
prog_char str_cobroller_arg2[] = "on#off";
parse_pgm_token_string_t cmd_cobroller_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_cobroller_result, arg2, str_cobroller_arg2);

prog_char help_cobroller[] = "Servo door function";
parse_pgm_inst_t cmd_cobroller = {
	.f = cmd_cobroller_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cobroller,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cobroller_arg0,
		(prog_void *)&cmd_cobroller_arg1,
		(prog_void *)&cmd_cobroller_arg2,
		NULL,
	},
};

/**********************************************************/
/* shovel */

/* this structure is filled when cmd_shovel is parsed successfully */
struct cmd_shovel_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_shovel is parsed successfully */
static void cmd_shovel_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_shovel_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("down")))
		shovel_down();
	else if (!strcmp_P(res->arg1, PSTR("up")))
		shovel_up();
	else if (!strcmp_P(res->arg1, PSTR("mid")))
		shovel_mid();
}

prog_char str_shovel_arg0[] = "shovel";
parse_pgm_token_string_t cmd_shovel_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_shovel_result, arg0, str_shovel_arg0);
prog_char str_shovel_arg1[] = "down#up#mid";
parse_pgm_token_string_t cmd_shovel_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_shovel_result, arg1, str_shovel_arg1);

prog_char help_shovel[] = "Servo shovel function";
parse_pgm_inst_t cmd_shovel = {
	.f = cmd_shovel_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_shovel,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_shovel_arg0,
		(prog_void *)&cmd_shovel_arg1,
		NULL,
	},
};

/**********************************************************/
/* Servo_Carry */

/* this structure is filled when cmd_servo_carry is parsed successfully */
struct cmd_servo_carry_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_servo_carry is parsed successfully */
static void cmd_servo_carry_parsed(void *parsed_result,
				    __attribute__((unused)) void *data)
{
	struct cmd_servo_carry_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("open")))
		servo_carry_open();
	else if (!strcmp_P(res->arg1, PSTR("closed")))
		servo_carry_close();
}

prog_char str_servo_carry_arg0[] = "carry";
parse_pgm_token_string_t cmd_servo_carry_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_servo_carry_result, arg0, str_servo_carry_arg0);
prog_char str_servo_carry_arg1[] = "open#closed";
parse_pgm_token_string_t cmd_servo_carry_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_servo_carry_result, arg1, str_servo_carry_arg1);

prog_char help_servo_carry[] = "Servo carry function";
parse_pgm_inst_t cmd_servo_carry = {
	.f = cmd_servo_carry_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_servo_carry,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_servo_carry_arg0,
		(prog_void *)&cmd_servo_carry_arg1,
		NULL,
	},
};

/**********************************************************/
/* Spickle tests */

/* this structure is filled when cmd_spickle is parsed successfully */
struct cmd_spickle_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_spickle is parsed successfully */
static void cmd_spickle_parsed(void * parsed_result,
			       __attribute__((unused)) void *data)
{
	struct cmd_spickle_result * res = parsed_result;
	uint8_t side;

	if (!strcmp_P(res->arg1, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else
		side = I2C_RIGHT_SIDE;

	if (!strcmp_P(res->arg2, PSTR("deploy"))) {
		spickle_deploy(side);
	}
	else if (!strcmp_P(res->arg2, PSTR("pack"))) {
		spickle_pack(side);
	}
	else if (!strcmp_P(res->arg2, PSTR("mid"))) {
		spickle_mid(side);
	}
	printf_P(PSTR("done\r\n"));
}

prog_char str_spickle_arg0[] = "spickle";
parse_pgm_token_string_t cmd_spickle_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_result, arg0, str_spickle_arg0);
prog_char str_spickle_arg1[] = "left#right";
parse_pgm_token_string_t cmd_spickle_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_result, arg1, str_spickle_arg1);
prog_char str_spickle_arg2[] = "deploy#pack#mid";
parse_pgm_token_string_t cmd_spickle_arg2 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_result, arg2, str_spickle_arg2);

prog_char help_spickle[] = "move spickle";
parse_pgm_inst_t cmd_spickle = {
	.f = cmd_spickle_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spickle,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spickle_arg0,
		(prog_void *)&cmd_spickle_arg1,
		(prog_void *)&cmd_spickle_arg2,
		NULL,
	},
};

/**********************************************************/
/* Set Spickle Params */

/* this structure is filled when cmd_spickle_params is parsed successfully */
struct cmd_spickle_params_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	int32_t arg3;
	int32_t arg4;
	int32_t arg5;
};

/* function called when cmd_spickle_params is parsed successfully */
static void cmd_spickle_params_parsed(void *parsed_result,
				      __attribute__((unused)) void *data)
{
	struct cmd_spickle_params_result * res = parsed_result;
	uint8_t side;

	if (!strcmp_P(res->arg1, PSTR("show"))) {
		spickle_dump_params();
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else
		side = I2C_RIGHT_SIDE;

	if (!strcmp_P(res->arg2, PSTR("pos")))
		spickle_set_pos(side, res->arg3, res->arg4, res->arg5);
}

prog_char str_spickle_params_arg0[] = "spickle_params";
parse_pgm_token_string_t cmd_spickle_params_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params_result, arg0, str_spickle_params_arg0);
prog_char str_spickle_params_arg1[] = "left#right";
parse_pgm_token_string_t cmd_spickle_params_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params_result, arg1, str_spickle_params_arg1);
prog_char str_spickle_params_arg2[] = "pos";
parse_pgm_token_string_t cmd_spickle_params_arg2 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params_result, arg2, str_spickle_params_arg2);
parse_pgm_token_num_t cmd_spickle_params_arg3 =
	TOKEN_NUM_INITIALIZER(struct cmd_spickle_params_result, arg3, INT32);
parse_pgm_token_num_t cmd_spickle_params_arg4 =
	TOKEN_NUM_INITIALIZER(struct cmd_spickle_params_result, arg4, INT32);
parse_pgm_token_num_t cmd_spickle_params_arg5 =
	TOKEN_NUM_INITIALIZER(struct cmd_spickle_params_result, arg5, INT32);

prog_char help_spickle_params[] = "Set spickle pos values: left|right pos INTPACK INTMID INTDEPL";
parse_pgm_inst_t cmd_spickle_params = {
	.f = cmd_spickle_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spickle_params,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spickle_params_arg0,
		(prog_void *)&cmd_spickle_params_arg1,
		(prog_void *)&cmd_spickle_params_arg2,
		(prog_void *)&cmd_spickle_params_arg3,
		(prog_void *)&cmd_spickle_params_arg4,
		(prog_void *)&cmd_spickle_params_arg5,
		NULL,
	},
};

prog_char str_spickle_params_arg1_show[] = "show";
parse_pgm_token_string_t cmd_spickle_params_arg1_show =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params_result, arg1, str_spickle_params_arg1_show);

prog_char help_spickle_params_show[] = "show spickle params";
parse_pgm_inst_t cmd_spickle_params_show = {
	.f = cmd_spickle_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spickle_params_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spickle_params_arg0,
		(prog_void *)&cmd_spickle_params_arg1_show,
		NULL,
	},
};

/**********************************************************/
/* Set Spickle Params */

/* this structure is filled when cmd_spickle_params2 is parsed successfully */
struct cmd_spickle_params2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_spickle_params2 is parsed successfully */
static void cmd_spickle_params2_parsed(void *parsed_result,
				      __attribute__((unused)) void *data)
{
	struct cmd_spickle_params2_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("coef"))) {
		spickle_set_coefs(res->arg2, res->arg3);
	}

	/* else show */
	spickle_dump_params();
}

prog_char str_spickle_params2_arg0[] = "spickle_params2";
parse_pgm_token_string_t cmd_spickle_params2_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params2_result, arg0, str_spickle_params2_arg0);
prog_char str_spickle_params2_arg1[] = "coef";
parse_pgm_token_string_t cmd_spickle_params2_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params2_result, arg1, str_spickle_params2_arg1);
parse_pgm_token_num_t cmd_spickle_params2_arg2 =
	TOKEN_NUM_INITIALIZER(struct cmd_spickle_params2_result, arg2, INT32);
parse_pgm_token_num_t cmd_spickle_params2_arg3 =
	TOKEN_NUM_INITIALIZER(struct cmd_spickle_params2_result, arg3, INT32);

prog_char help_spickle_params2[] = "Set spickle_params2 values";
parse_pgm_inst_t cmd_spickle_params2 = {
	.f = cmd_spickle_params2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spickle_params2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spickle_params2_arg0,
		(prog_void *)&cmd_spickle_params2_arg1,
		(prog_void *)&cmd_spickle_params2_arg2,
		(prog_void *)&cmd_spickle_params2_arg3,
		NULL,
	},
};

prog_char str_spickle_params2_arg1_show[] = "show";
parse_pgm_token_string_t cmd_spickle_params2_arg1_show =
	TOKEN_STRING_INITIALIZER(struct cmd_spickle_params2_result, arg1, str_spickle_params2_arg1_show);

prog_char help_spickle_params2_show[] = "show spickle params";
parse_pgm_inst_t cmd_spickle_params2_show = {
	.f = cmd_spickle_params2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spickle_params2_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spickle_params2_arg0,
		(prog_void *)&cmd_spickle_params2_arg1_show,
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
static void cmd_test_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

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
