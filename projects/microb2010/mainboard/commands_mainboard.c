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
 *  Revision : $Id: commands_mainboard.c,v 1.9 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <hostsim.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <aversive/eeprom.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>
#include <i2c.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_core.h>
#include <trajectory_manager_utils.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "../common/eeprom_mapping.h"

#include "main.h"
#include "robotsim.h"
#include "sensor.h"
#include "cmdline.h"
#include "strat.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat_db.h"
#include "strat_corn.h"
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
			mainboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			mainboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"),
				 (DO_ENCODERS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"),
				 (DO_CS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("rs is %s\r\n"),
				 (DO_RS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("pos is %s\r\n"),
				 (DO_POS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"),
				 (DO_BD & mainboard.flags) ? "on":"off");
			printf_P(PSTR("timer is %s\r\n"),
				 (DO_TIMER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"),
				 (DO_POWER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("errblock is %s\r\n"),
				 (DO_ERRBLOCKING & mainboard.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		strat_hardstop();
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("rs")))
		bit = DO_RS;
	else if (!strcmp_P(res->arg1, PSTR("pos")))
		bit = DO_POS;
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("timer"))) {
		time_reset();
		bit = DO_TIMER;
	}
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;
	else if (!strcmp_P(res->arg1, PSTR("errblock")))
		bit = DO_ERRBLOCKING;

	if (!strcmp_P(res->arg2, PSTR("on")))
		mainboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
#ifdef HOST_VERSION
			robotsim_pwm(LEFT_PWM, 0);
			robotsim_pwm(RIGHT_PWM, 0);
#else
			pwm_ng_set(LEFT_PWM, 0);
			pwm_ng_set(RIGHT_PWM, 0);
#endif
		}
		mainboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1,
		      (bit & mainboard.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power#errblock";
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
/* Spi_Test */

/* this structure is filled when cmd_spi_test is parsed successfully */
struct cmd_spi_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_spi_test is parsed successfully */
static void cmd_spi_test_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	uint16_t i = 0, ret = 0, ret2 = 0;

	if (mainboard.flags & DO_ENCODERS) {
		printf_P(PSTR("Disable encoder event first\r\n"));
		return;
	}

	do {
		spi_slave_select(0);
		ret = spi_send_and_receive_byte(i);
		ret2 = spi_send_and_receive_byte(i);
		spi_slave_deselect(0);

		if ((i & 0x7ff) == 0)
			printf_P(PSTR("Sent %.4x twice, received %x %x\r\n"),
				 i, ret, ret2);

		i++;
	} while(!cmdline_keypressed());
#endif
}

prog_char str_spi_test_arg0[] = "spi_test";
parse_pgm_token_string_t cmd_spi_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_spi_test_result, arg0, str_spi_test_arg0);

prog_char help_spi_test[] = "Test the SPI";
parse_pgm_inst_t cmd_spi_test = {
	.f = cmd_spi_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_spi_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_spi_test_arg0,
		NULL,
	},
};



/**********************************************************/
/* Opponent tests */

/* this structure is filled when cmd_opponent is parsed successfully */
struct cmd_opponent_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_opponent is parsed successfully */
static void cmd_opponent_parsed(void *parsed_result, void *data)
{
	int16_t x,y,d,a;

	if (get_opponent_xyda(&x, &y, &d, &a))
		printf_P(PSTR("No opponent\r\n"));
	else
		printf_P(PSTR("x=%d y=%d, d=%d a=%d\r\n"), x, y, d, a);
}

prog_char str_opponent_arg0[] = "opponent";
parse_pgm_token_string_t cmd_opponent_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg0, str_opponent_arg0);
prog_char str_opponent_arg1[] = "show";
parse_pgm_token_string_t cmd_opponent_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1);

prog_char help_opponent[] = "Show (x,y) opponent";
parse_pgm_inst_t cmd_opponent = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0,
		(prog_void *)&cmd_opponent_arg1,
		NULL,
	},
};


prog_char str_opponent_arg1_set[] = "set";
parse_pgm_token_string_t cmd_opponent_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1_set);
parse_pgm_token_num_t cmd_opponent_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg2, INT32);
parse_pgm_token_num_t cmd_opponent_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg3, INT32);

prog_char help_opponent_set[] = "Set (x,y) opponent";
parse_pgm_inst_t cmd_opponent_set = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent_set,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0,
		(prog_void *)&cmd_opponent_arg1_set,
		(prog_void *)&cmd_opponent_arg2,
		(prog_void *)&cmd_opponent_arg3,
		NULL,
	},
};

/**********************************************************/
/* Beacon_Start */

/* this structure is filled when cmd_beacon_start is parsed successfully */
struct cmd_beacon_start_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_beacon_start is parsed successfully */
static void cmd_beacon_start_parsed(void *parsed_result, void *data)
{
	struct cmd_beacon_start_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("start")))
		i2c_ballboard_set_beacon(1);
	else
		i2c_ballboard_set_beacon(0);
}

prog_char str_beacon_start_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_start_result, arg0, str_beacon_start_arg0);
prog_char str_beacon_start_arg1[] = "start#stop";
parse_pgm_token_string_t cmd_beacon_start_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_start_result, arg1, str_beacon_start_arg1);

prog_char help_beacon_start[] = "Beacon enabled/disable";
parse_pgm_inst_t cmd_beacon_start = {
	.f = cmd_beacon_start_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon_start,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_start_arg0, 
		(prog_void *)&cmd_beacon_start_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result {
	fixed_string_t arg0;
	fixed_string_t color;
	fixed_string_t debug;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{
	struct cmd_start_result *res = parsed_result;
	uint8_t old_level = gen.log_level;

	gen.logs[NB_LOGS] = E_USER_STRAT;
	if (!strcmp_P(res->debug, PSTR("debug"))) {
		strat_db.dump_enabled = 1;
		gen.log_level = 5;
	}
	else {
		strat_db.dump_enabled = 0;
		gen.log_level = 0;
	}

	if (!strcmp_P(res->color, PSTR("yellow"))) {
		mainboard.our_color = I2C_COLOR_YELLOW;
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_YELLOW);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_YELLOW);
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		mainboard.our_color = I2C_COLOR_BLUE;
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_BLUE);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_BLUE);
	}

	printf_P(PSTR("Press a key when beacon ready\r\n"));
	i2c_ballboard_set_beacon(0);
	while(!cmdline_keypressed());
	i2c_ballboard_set_beacon(1);

	strat_start();

	gen.logs[NB_LOGS] = 0;
	gen.log_level = old_level;
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_color[] = "blue#yellow";
parse_pgm_token_string_t cmd_start_color = TOKEN_STRING_INITIALIZER(struct cmd_start_result, color, str_start_color);
prog_char str_start_debug[] = "debug#match";
parse_pgm_token_string_t cmd_start_debug = TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
	.f = cmd_start_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_start,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_start_arg0,
		(prog_void *)&cmd_start_color,
		(prog_void *)&cmd_start_debug,
		NULL,
	},
};



/**********************************************************/
/* Interact */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_interact_result {
	fixed_string_t arg0;
};

static void print_cs(void)
{
	printf_P(PSTR("cons_d=% .8"PRIi32" cons_a=% .8"PRIi32" fil_d=% .8"PRIi32" fil_a=% .8"PRIi32" "
		      "err_d=% .8"PRIi32" err_a=% .8"PRIi32" out_d=% .8"PRIi32" out_a=% .8"PRIi32"\r\n"),
		 cs_get_consign(&mainboard.distance.cs),
		 cs_get_consign(&mainboard.angle.cs),
		 cs_get_filtered_consign(&mainboard.distance.cs),
		 cs_get_filtered_consign(&mainboard.angle.cs),
		 cs_get_error(&mainboard.distance.cs),
		 cs_get_error(&mainboard.angle.cs),
		 cs_get_out(&mainboard.distance.cs),
		 cs_get_out(&mainboard.angle.cs));
}

static void print_pos(void)
{
	printf_P(PSTR("x=% .8d y=% .8d a=% .8d\r\n"),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
}

static void print_time(void)
{
	printf_P(PSTR("time %d\r\n"),time_get_s());
}


static void print_sensors(void)
{
#ifdef notyet
	if (sensor_start_switch())
		printf_P(PSTR("Start switch | "));
	else
		printf_P(PSTR("             | "));

	if (IR_DISP_SENSOR())
		printf_P(PSTR("IR disp | "));
	else
		printf_P(PSTR("        | "));

	printf_P(PSTR("\r\n"));
#endif
}

static void print_pid(void)
{
	printf_P(PSTR("P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32" | "
		      "P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32"\r\n"),
		 pid_get_value_in(&mainboard.distance.pid) * pid_get_gain_P(&mainboard.distance.pid),
		 pid_get_value_I(&mainboard.distance.pid) * pid_get_gain_I(&mainboard.distance.pid),
		 pid_get_value_D(&mainboard.distance.pid) * pid_get_gain_D(&mainboard.distance.pid),
		 pid_get_value_out(&mainboard.distance.pid),
		 pid_get_value_in(&mainboard.angle.pid) * pid_get_gain_P(&mainboard.angle.pid),
		 pid_get_value_I(&mainboard.angle.pid) * pid_get_gain_I(&mainboard.angle.pid),
		 pid_get_value_D(&mainboard.angle.pid) * pid_get_gain_D(&mainboard.angle.pid),
		 pid_get_value_out(&mainboard.angle.pid));
}

#define PRINT_POS       (1<<0)
#define PRINT_PID       (1<<1)
#define PRINT_CS        (1<<2)
#define PRINT_SENSORS   (1<<3)
#define PRINT_TIME      (1<<4)
#define PRINT_BLOCKING  (1<<5)

static void cmd_interact_parsed(void * parsed_result, void * data)
{
	int c;
	int8_t cmd;
	uint8_t print = 0;
	struct vt100 vt100;

	vt100_init(&vt100);

	printf_P(PSTR("Display debugs:\r\n"
		      "  1:pos\r\n"
		      "  2:pid\r\n"
		      "  3:cs\r\n"
		      "  4:sensors\r\n"
		      "  5:time\r\n"
		      /* "  6:blocking\r\n" */
		      "Commands:\r\n"
		      "  arrows:move\r\n"
		      "  space:stop\r\n"
		      "  q:quit\r\n"));

	/* stop motors */
	mainboard.flags &= (~DO_CS);
	pwm_set_and_save(LEFT_PWM, 0);
	pwm_set_and_save(RIGHT_PWM, 0);

	while(1) {
		if (print & PRINT_POS) {
			print_pos();
		}

		if (print & PRINT_PID) {
			print_pid();
		}

		if (print & PRINT_CS) {
			print_cs();
		}

		if (print & PRINT_SENSORS) {
			print_sensors();
		}

		if (print & PRINT_TIME) {
			print_time();
		}
/* 		if (print & PRINT_BLOCKING) { */
/* 			printf_P(PSTR("%s %s blocking=%d\r\n"),  */
/* 				 mainboard.blocking ? "BLOCK1":"      ", */
/* 				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ", */
/* 				 rs_get_blocking(&mainboard.rs)); */
/* 		} */

		c = cmdline_getchar();
		if (c == -1) {
			wait_ms(10);
			continue;
		}
		cmd = vt100_parser(&vt100, c);
		if (cmd == -2) {
			wait_ms(10);
			continue;
		}

		if (cmd == -1) {
			switch(c) {
			case '1': print ^= PRINT_POS; break;
			case '2': print ^= PRINT_PID; break;
			case '3': print ^= PRINT_CS; break;
			case '4': print ^= PRINT_SENSORS; break;
			case '5': print ^= PRINT_TIME; break;
			case '6': print ^= PRINT_BLOCKING; break;

			case 'q':
				if (mainboard.flags & DO_CS)
					strat_hardstop();
				pwm_set_and_save(LEFT_PWM, 0);
				pwm_set_and_save(RIGHT_PWM, 0);
				return;
			case ' ':
				pwm_set_and_save(LEFT_PWM, 0);
				pwm_set_and_save(RIGHT_PWM, 0);
				break;
			default:
				break;
			}
		}
		else {
#ifdef HOST_VERSION
#define PWM_INTERACT 1200
#else
#define PWM_INTERACT 1200
#endif
			switch(cmd) {
			case KEY_UP_ARR:
				pwm_set_and_save(LEFT_PWM, PWM_INTERACT);
				pwm_set_and_save(RIGHT_PWM, PWM_INTERACT);
				break;
			case KEY_LEFT_ARR:
				pwm_set_and_save(LEFT_PWM, -PWM_INTERACT);
				pwm_set_and_save(RIGHT_PWM, PWM_INTERACT);
				break;
			case KEY_DOWN_ARR:
				pwm_set_and_save(LEFT_PWM, -PWM_INTERACT);
				pwm_set_and_save(RIGHT_PWM, -PWM_INTERACT);
				break;
			case KEY_RIGHT_ARR:
				pwm_set_and_save(LEFT_PWM, PWM_INTERACT);
				pwm_set_and_save(RIGHT_PWM, -PWM_INTERACT);
				break;
			}
		}
		wait_ms(10);
	}
}

prog_char str_interact_arg0[] = "interact";
parse_pgm_token_string_t cmd_interact_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);

prog_char help_interact[] = "Interactive mode";
parse_pgm_inst_t cmd_interact = {
	.f = cmd_interact_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_interact,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_interact_arg0,
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
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		mainboard.our_color = I2C_COLOR_YELLOW;
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_YELLOW);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_YELLOW);
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		mainboard.our_color = I2C_COLOR_BLUE;
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_BLUE);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_BLUE);
	}
	printf_P(PSTR("Done\r\n"));
#endif
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
/* Rs tests */

/* this structure is filled when cmd_rs is parsed successfully */
struct cmd_rs_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_rs is parsed successfully */
static void cmd_rs_parsed(void *parsed_result, void *data)
{
	//	struct cmd_rs_result *res = parsed_result;
	do {
		printf_P(PSTR("angle cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
			 cs_get_consign(&mainboard.angle.cs),
			 cs_get_filtered_feedback(&mainboard.angle.cs),
			 cs_get_out(&mainboard.angle.cs));
		printf_P(PSTR("distance cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
			 cs_get_consign(&mainboard.distance.cs),
			 cs_get_filtered_feedback(&mainboard.distance.cs),
			 cs_get_out(&mainboard.distance.cs));
		printf_P(PSTR("l=% .4"PRIi32" r=% .4"PRIi32"\r\n"), mainboard.pwm_l,
			 mainboard.pwm_r);
		wait_ms(100);
	} while(!cmdline_keypressed());
}

prog_char str_rs_arg0[] = "rs";
parse_pgm_token_string_t cmd_rs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
prog_char str_rs_arg1[] = "show";
parse_pgm_token_string_t cmd_rs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);

prog_char help_rs[] = "Show rs (robot system) values";
parse_pgm_inst_t cmd_rs = {
	.f = cmd_rs_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_rs,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_rs_arg0,
		(prog_void *)&cmd_rs_arg1,
		NULL,
	},
};

/**********************************************************/
/* I2cdebug */

/* this structure is filled when cmd_i2cdebug is parsed successfully */
struct cmd_i2cdebug_result {
	fixed_string_t arg0;
};

/* function called when cmd_i2cdebug is parsed successfully */
static void cmd_i2cdebug_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	i2c_debug();
	i2c_protocol_debug();
#endif
}

prog_char str_i2cdebug_arg0[] = "i2cdebug";
parse_pgm_token_string_t cmd_i2cdebug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_i2cdebug_result, arg0, str_i2cdebug_arg0);

prog_char help_i2cdebug[] = "I2c debug infos";
parse_pgm_inst_t cmd_i2cdebug = {
	.f = cmd_i2cdebug_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_i2cdebug,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_i2cdebug_arg0,
		NULL,
	},
};

/**********************************************************/
/* Cobboard_Show */

/* this structure is filled when cmd_cobboard_show is parsed successfully */
struct cmd_cobboard_show_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_cobboard_show is parsed successfully */
static void cmd_cobboard_show_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	printf_P(PSTR("mode = %x\r\n"), cobboard.mode);
	printf_P(PSTR("status = %x\r\n"), cobboard.status);
	printf_P(PSTR("cob_count = %x\r\n"), cobboard.cob_count);
	printf_P(PSTR("left_cobroller_speed = %d\r\n"), cobboard.left_cobroller_speed);
	printf_P(PSTR("right_cobroller_speed = %d\r\n"), cobboard.right_cobroller_speed);
#endif
}

prog_char str_cobboard_show_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_cobboard_show_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_show_result, arg0, str_cobboard_show_arg0);
prog_char str_cobboard_show_arg1[] = "show";
parse_pgm_token_string_t cmd_cobboard_show_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_show_result, arg1, str_cobboard_show_arg1);

prog_char help_cobboard_show[] = "show cobboard status";
parse_pgm_inst_t cmd_cobboard_show = {
	.f = cmd_cobboard_show_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cobboard_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cobboard_show_arg0,
		(prog_void *)&cmd_cobboard_show_arg1,
		NULL,
	},
};

/**********************************************************/
/* Cobboard_Setmode1 */

/* this structure is filled when cmd_cobboard_setmode1 is parsed successfully */
struct cmd_cobboard_setmode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_cobboard_setmode1 is parsed successfully */
static void cmd_cobboard_setmode1_parsed(void *parsed_result, void *data)
{
	struct cmd_cobboard_setmode1_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("init")))
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_INIT);
	else if (!strcmp_P(res->arg1, PSTR("eject")))
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_EJECT);
	else if (!strcmp_P(res->arg1, PSTR("kickstand_up")))
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_KICKSTAND_UP);
	else if (!strcmp_P(res->arg1, PSTR("kickstand_down")))
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_KICKSTAND_DOWN);
}

prog_char str_cobboard_setmode1_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_cobboard_setmode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode1_result, arg0, str_cobboard_setmode1_arg0);
prog_char str_cobboard_setmode1_arg1[] = "init#eject#kickstand_up#kickstand_down";
parse_pgm_token_string_t cmd_cobboard_setmode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode1_result, arg1, str_cobboard_setmode1_arg1);

prog_char help_cobboard_setmode1[] = "set cobboard mode (mode)";
parse_pgm_inst_t cmd_cobboard_setmode1 = {
	.f = cmd_cobboard_setmode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cobboard_setmode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cobboard_setmode1_arg0,
		(prog_void *)&cmd_cobboard_setmode1_arg1,
		NULL,
	},
};

/**********************************************************/
/* Cobboard_Setmode2 */

/* this structure is filled when cmd_cobboard_setmode2 is parsed successfully */
struct cmd_cobboard_setmode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_cobboard_setmode2 is parsed successfully */
static void cmd_cobboard_setmode2_parsed(void *parsed_result, void *data)
{
	struct cmd_cobboard_setmode2_result *res = parsed_result;
	uint8_t side = I2C_LEFT_SIDE;

	if (!strcmp_P(res->arg2, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("right")))
		side = I2C_RIGHT_SIDE;

	if (!strcmp_P(res->arg1, PSTR("deploy"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_deploy(side);
	}
	else if (!strcmp_P(res->arg1, PSTR("harvest"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_autoharvest(side);
	}
	else if (!strcmp_P(res->arg1, PSTR("pack"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_pack(side);
	}
	else if (!strcmp_P(res->arg1, PSTR("weak_pack"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_pack_weak(side);
	}
	else if (!strcmp_P(res->arg1, PSTR("deploy_nomove"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_deploy_nomove(side);
	}
	else if (!strcmp_P(res->arg1, PSTR("harvest_nomove"))) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
		i2c_cobboard_autoharvest_nomove(side);
	}
}

prog_char str_cobboard_setmode2_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_cobboard_setmode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode2_result, arg0, str_cobboard_setmode2_arg0);
prog_char str_cobboard_setmode2_arg1[] = "harvest#deploy#pack#weak_pack#harvest_nomove#deploy_nomove";
parse_pgm_token_string_t cmd_cobboard_setmode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode2_result, arg1, str_cobboard_setmode2_arg1);
prog_char str_cobboard_setmode2_arg2[] = "left#right";
parse_pgm_token_string_t cmd_cobboard_setmode2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode2_result, arg2, str_cobboard_setmode2_arg2);

prog_char help_cobboard_setmode2[] = "set cobboard mode (mode, side)";
parse_pgm_inst_t cmd_cobboard_setmode2 = {
	.f = cmd_cobboard_setmode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cobboard_setmode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cobboard_setmode2_arg0,
		(prog_void *)&cmd_cobboard_setmode2_arg1,
		(prog_void *)&cmd_cobboard_setmode2_arg2,
		NULL,
	},
};

/**********************************************************/
/* Cobboard_Setmode3 */

/* this structure is filled when cmd_cobboard_setmode3 is parsed successfully */
struct cmd_cobboard_setmode3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t level;
};

/* function called when cmd_cobboard_setmode3 is parsed successfully */
static void cmd_cobboard_setmode3_parsed(void *parsed_result, void *data)
{
	struct cmd_cobboard_setmode3_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("xxx")))
		printf("faux\r\n");
}

prog_char str_cobboard_setmode3_arg0[] = "cobboard";
parse_pgm_token_string_t cmd_cobboard_setmode3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode3_result, arg0, str_cobboard_setmode3_arg0);
prog_char str_cobboard_setmode3_arg1[] = "xxx";
parse_pgm_token_string_t cmd_cobboard_setmode3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cobboard_setmode3_result, arg1, str_cobboard_setmode3_arg1);
parse_pgm_token_num_t cmd_cobboard_setmode3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_cobboard_setmode3_result, level, UINT8);

prog_char help_cobboard_setmode3[] = "set cobboard mode (mode, level)";
parse_pgm_inst_t cmd_cobboard_setmode3 = {
	.f = cmd_cobboard_setmode3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cobboard_setmode3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cobboard_setmode3_arg0,
		(prog_void *)&cmd_cobboard_setmode3_arg1,
		(prog_void *)&cmd_cobboard_setmode3_arg2,
		NULL,
	},
};

/**********************************************************/
/* Ballboard_Show */

/* this structure is filled when cmd_ballboard_show is parsed successfully */
struct cmd_ballboard_show_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_ballboard_show is parsed successfully */
static void cmd_ballboard_show_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	printf_P(PSTR("mode = %x\r\n"), ballboard.mode);
	printf_P(PSTR("status = %x\r\n"), ballboard.status);
	printf_P(PSTR("ball_count = %d\r\n"), ballboard.ball_count);
	printf_P(PSTR("lcob = %d\r\n"), ballboard.lcob);
	printf_P(PSTR("rcob = %d\r\n"), ballboard.rcob);
#endif
}

prog_char str_ballboard_show_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_ballboard_show_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_show_result, arg0, str_ballboard_show_arg0);
prog_char str_ballboard_show_arg1[] = "show";
parse_pgm_token_string_t cmd_ballboard_show_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_show_result, arg1, str_ballboard_show_arg1);

prog_char help_ballboard_show[] = "show ballboard status";
parse_pgm_inst_t cmd_ballboard_show = {
	.f = cmd_ballboard_show_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ballboard_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ballboard_show_arg0,
		(prog_void *)&cmd_ballboard_show_arg1,
		NULL,
	},
};

/**********************************************************/
/* Ballboard_Setmode1 */

/* this structure is filled when cmd_ballboard_setmode1 is parsed successfully */
struct cmd_ballboard_setmode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_ballboard_setmode1 is parsed successfully */
static void cmd_ballboard_setmode1_parsed(void *parsed_result, void *data)
{
	struct cmd_ballboard_setmode1_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("init")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_INIT);
	else if (!strcmp_P(res->arg1, PSTR("off")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_OFF);
	else if (!strcmp_P(res->arg1, PSTR("eject")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_EJECT);
	else if (!strcmp_P(res->arg1, PSTR("harvest")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_HARVEST);
	else if (!strcmp_P(res->arg1, PSTR("prepare")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_PREP_FORK);
	else if (!strcmp_P(res->arg1, PSTR("take")))
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_TAKE_FORK);

	/* other commands */
}

prog_char str_ballboard_setmode1_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_ballboard_setmode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode1_result, arg0, str_ballboard_setmode1_arg0);
prog_char str_ballboard_setmode1_arg1[] = "init#eject#harvest#off#take#prepare";
parse_pgm_token_string_t cmd_ballboard_setmode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode1_result, arg1, str_ballboard_setmode1_arg1);

prog_char help_ballboard_setmode1[] = "set ballboard mode (mode)";
parse_pgm_inst_t cmd_ballboard_setmode1 = {
	.f = cmd_ballboard_setmode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ballboard_setmode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ballboard_setmode1_arg0,
		(prog_void *)&cmd_ballboard_setmode1_arg1,
		NULL,
	},
};

/**********************************************************/
/* Ballboard_Setmode2 */

/* this structure is filled when cmd_ballboard_setmode2 is parsed successfully */
struct cmd_ballboard_setmode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_ballboard_setmode2 is parsed successfully */
static void cmd_ballboard_setmode2_parsed(void * parsed_result, void * data)
{
}

prog_char str_ballboard_setmode2_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_ballboard_setmode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode2_result, arg0, str_ballboard_setmode2_arg0);
prog_char str_ballboard_setmode2_arg1[] = "xxx";
parse_pgm_token_string_t cmd_ballboard_setmode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode2_result, arg1, str_ballboard_setmode2_arg1);
prog_char str_ballboard_setmode2_arg2[] = "left#right";
parse_pgm_token_string_t cmd_ballboard_setmode2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode2_result, arg2, str_ballboard_setmode2_arg2);

prog_char help_ballboard_setmode2[] = "set ballboard mode (mode, side)";
parse_pgm_inst_t cmd_ballboard_setmode2 = {
	.f = cmd_ballboard_setmode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ballboard_setmode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ballboard_setmode2_arg0,
		(prog_void *)&cmd_ballboard_setmode2_arg1,
		(prog_void *)&cmd_ballboard_setmode2_arg2,
		NULL,
	},
};

/**********************************************************/
/* Ballboard_Setmode3 */

/* this structure is filled when cmd_ballboard_setmode3 is parsed successfully */
struct cmd_ballboard_setmode3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t level;
};

/* function called when cmd_ballboard_setmode3 is parsed successfully */
static void cmd_ballboard_setmode3_parsed(void *parsed_result, void *data)
{
	struct cmd_ballboard_setmode3_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("xxx")))
		printf("faux\r\n");
}

prog_char str_ballboard_setmode3_arg0[] = "ballboard";
parse_pgm_token_string_t cmd_ballboard_setmode3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode3_result, arg0, str_ballboard_setmode3_arg0);
prog_char str_ballboard_setmode3_arg1[] = "xxx";
parse_pgm_token_string_t cmd_ballboard_setmode3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_ballboard_setmode3_result, arg1, str_ballboard_setmode3_arg1);
parse_pgm_token_num_t cmd_ballboard_setmode3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_ballboard_setmode3_result, level, UINT8);

prog_char help_ballboard_setmode3[] = "set ballboard mode (mode, level)";
parse_pgm_inst_t cmd_ballboard_setmode3 = {
	.f = cmd_ballboard_setmode3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ballboard_setmode3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ballboard_setmode3_arg0,
		(prog_void *)&cmd_ballboard_setmode3_arg1,
		(prog_void *)&cmd_ballboard_setmode3_arg2,
		NULL,
	},
};

/**********************************************************/
/* Servo_Balls */

/* this structure is filled when cmd_servo_balls is parsed successfully */
struct cmd_servo_balls_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_servo_balls is parsed successfully */
static void cmd_servo_balls_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	struct cmd_servo_balls_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("deploy")))
		support_balls_deploy();
	else if (!strcmp_P(res->arg1, PSTR("pack")))
		support_balls_pack();
}

prog_char str_servo_balls_arg0[] = "support_balls";
parse_pgm_token_string_t cmd_servo_balls_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_servo_balls_result, arg0, str_servo_balls_arg0);
prog_char str_servo_balls_arg1[] = "deploy#pack";
parse_pgm_token_string_t cmd_servo_balls_arg1 =
	TOKEN_STRING_INITIALIZER(struct cmd_servo_balls_result, arg1, str_servo_balls_arg1);

prog_char help_servo_balls[] = "control support balls";
parse_pgm_inst_t cmd_servo_balls = {
	.f = cmd_servo_balls_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_servo_balls,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_servo_balls_arg0,
		(prog_void *)&cmd_servo_balls_arg1,
		NULL,
	},
};

/**********************************************************/
/* Clitoid */

/* this structure is filled when cmd_clitoid is parsed successfully */
struct cmd_clitoid_result {
	fixed_string_t arg0;
	float alpha_deg;
	float beta_deg;
	float R_mm;
	float Vd;
	float Amax;
	float d_inter_mm;
};

/* function called when cmd_test is parsed successfully */
static void cmd_clitoid_parsed(void *parsed_result, void *data)
{
	struct cmd_clitoid_result *res = parsed_result;
/* 	clitoid(res->alpha_deg, res->beta_deg, res->R_mm, */
/* 		res->Vd, res->Amax, res->d_inter_mm); */
	double x = position_get_x_double(&mainboard.pos);
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);

	strat_set_speed(res->Vd, SPEED_ANGLE_FAST);
	trajectory_clitoid(&mainboard.traj, x, y, a, 150.,
			   res->alpha_deg, res->beta_deg, res->R_mm,
			   res->d_inter_mm);
}

prog_char str_clitoid_arg0[] = "clitoid";
parse_pgm_token_string_t cmd_clitoid_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_clitoid_result,
				 arg0, str_clitoid_arg0);
parse_pgm_token_num_t cmd_clitoid_alpha_deg =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      alpha_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_beta_deg =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      beta_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_R_mm =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      R_mm, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Vd =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      Vd, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Amax =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      Amax, FLOAT);
parse_pgm_token_num_t cmd_clitoid_d_inter_mm =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      d_inter_mm, FLOAT);

prog_char help_clitoid[] = "do a clitoid (alpha, beta, R, Vd, Amax, d_inter)";
parse_pgm_inst_t cmd_clitoid = {
	.f = cmd_clitoid_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_clitoid,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_clitoid_arg0,
		(prog_void *)&cmd_clitoid_alpha_deg,
		(prog_void *)&cmd_clitoid_beta_deg,
		(prog_void *)&cmd_clitoid_R_mm,
		(prog_void *)&cmd_clitoid_Vd,
		(prog_void *)&cmd_clitoid_Amax,
		(prog_void *)&cmd_clitoid_d_inter_mm,
		NULL,
	},
};

/**********************************************************/
/* Time_Monitor */

/* this structure is filled when cmd_time_monitor is parsed successfully */
struct cmd_time_monitor_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_time_monitor is parsed successfully */
static void cmd_time_monitor_parsed(void *parsed_result, void *data)
{
#ifndef HOST_VERSION
	struct cmd_time_monitor_result *res = parsed_result;
	uint16_t seconds;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		eeprom_write_word(EEPROM_TIME_ADDRESS, 0);
	}
	seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
	printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
#endif
}

prog_char str_time_monitor_arg0[] = "time_monitor";
parse_pgm_token_string_t cmd_time_monitor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg0, str_time_monitor_arg0);
prog_char str_time_monitor_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_time_monitor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg1, str_time_monitor_arg1);

prog_char help_time_monitor[] = "Show since how long we are running";
parse_pgm_inst_t cmd_time_monitor = {
	.f = cmd_time_monitor_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_time_monitor,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_time_monitor_arg0,
		(prog_void *)&cmd_time_monitor_arg1,
		NULL,
	},
};


/**********************************************************/
/* Strat_Event */

/* this structure is filled when cmd_strat_event is parsed successfully */
struct cmd_strat_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_strat_event is parsed successfully */
static void cmd_strat_event_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_event_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("on")))
		strat_event_enable();
	else
		strat_event_disable();
}

prog_char str_strat_event_arg0[] = "strat_event";
parse_pgm_token_string_t cmd_strat_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_event_result, arg0, str_strat_event_arg0);
prog_char str_strat_event_arg1[] = "on#off";
parse_pgm_token_string_t cmd_strat_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_event_result, arg1, str_strat_event_arg1);

prog_char help_strat_event[] = "Enable/disable strat_event callback";
parse_pgm_inst_t cmd_strat_event = {
	.f = cmd_strat_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_event_arg0,
		(prog_void *)&cmd_strat_event_arg1,
		NULL,
	},
};

/**********************************************************/
/* Sleep */

/* this structure is filled when cmd_sleep is parsed successfully */
struct cmd_sleep_result {
	fixed_string_t arg0;
	uint32_t ms;
};

/* function called when cmd_sleep is parsed successfully */
static void cmd_sleep_parsed(void *parsed_result, void *data)
{
	struct cmd_sleep_result *res = parsed_result;
	time_wait_ms(res->ms);
}

prog_char str_sleep_arg0[] = "sleep";
parse_pgm_token_string_t cmd_sleep_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sleep_result, arg0, str_sleep_arg0);
parse_pgm_token_num_t cmd_sleep_ms = TOKEN_NUM_INITIALIZER(struct cmd_sleep_result, ms, UINT32);

prog_char help_sleep[] = "Sleep during some miliseconds";
parse_pgm_inst_t cmd_sleep = {
	.f = cmd_sleep_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sleep,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sleep_arg0,
		(prog_void *)&cmd_sleep_ms,
		NULL,
	},
};


/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
	int32_t radius;
	int32_t dist;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(void *parsed_result, void *data)
{
	strat_db.dump_enabled = 1;
	strat_db_dump(__FUNCTION__);

	corn_set_color(strat_db.corn_table[0], I2C_COB_BLACK);
	strat_db_dump(__FUNCTION__);

	corn_set_color(strat_db.corn_table[3], I2C_COB_WHITE);
	strat_db_dump(__FUNCTION__);
	corn_set_color(strat_db.corn_table[4], I2C_COB_WHITE);
	strat_db_dump(__FUNCTION__);
	corn_set_color(strat_db.corn_table[5], I2C_COB_WHITE);
	strat_db_dump(__FUNCTION__);
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);
parse_pgm_token_num_t cmd_test_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_test_result, radius, INT32);
parse_pgm_token_num_t cmd_test_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_test_result, dist, INT32);

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
