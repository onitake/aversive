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

#include <hostsim.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <aversive/eeprom.h>

#include <uart.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
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

#include "main.h"
#include "actuator.h"
#include "robotsim.h"
#include "cmdline.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"

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
		bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS |
			DO_BD | DO_TIMER | DO_POWER;
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
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power";
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
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_start_result *res = parsed_result;
	uint8_t old_level = gen.log_level;

	gen.logs[NB_LOGS] = E_USER_STRAT;
	if (!strcmp_P(res->debug, PSTR("debug"))) {
		strat_infos.dump_enabled = 1;
		gen.log_level = 5;
	}
	else {
		strat_infos.dump_enabled = 0;
		gen.log_level = 0;
	}

	if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_RED);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_RED);
	}
	else if (!strcmp_P(res->color, PSTR("green"))) {
		mainboard.our_color = I2C_COLOR_GREEN;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_GREEN);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_GREEN);
	}

	printf_P(PSTR("Check that lintel is loaded\r\n"));
	while(!cmdline_keypressed());

	printf_P(PSTR("Press a key when beacon ready\r\n"));
	i2c_sensorboard_set_beacon(0);
	while(!cmdline_keypressed());
	i2c_sensorboard_set_beacon(1);

	strat_start();

	gen.logs[NB_LOGS] = 0;
	gen.log_level = old_level;
#endif
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_color[] = "green#red";
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
			switch(cmd) {
			case KEY_UP_ARR: 
				pwm_set_and_save(LEFT_PWM, 1200);
				pwm_set_and_save(RIGHT_PWM, 1200);
				break;
			case KEY_LEFT_ARR: 
				pwm_set_and_save(LEFT_PWM, -1200);
				pwm_set_and_save(RIGHT_PWM, 1200);
				break;
			case KEY_DOWN_ARR: 
				pwm_set_and_save(LEFT_PWM, -1200);
				pwm_set_and_save(RIGHT_PWM, -1200);
				break;
			case KEY_RIGHT_ARR:
				pwm_set_and_save(LEFT_PWM, 1200);
				pwm_set_and_save(RIGHT_PWM, -1200);
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
	if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_RED);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_RED);
	}
	else if (!strcmp_P(res->color, PSTR("green"))) {
		mainboard.our_color = I2C_COLOR_GREEN;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_GREEN);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_GREEN);
	}
	printf_P(PSTR("Done\r\n"));
#endif
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "green#red";
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
/* Mechboard_Show */

/* this structure is filled when cmd_mechboard_show is parsed successfully */
struct cmd_mechboard_show_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_mechboard_show is parsed successfully */
static void cmd_mechboard_show_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	printf_P(PSTR("mode = %x\r\n"), mechboard.mode);
	printf_P(PSTR("status = %x\r\n"), mechboard.status);
	printf_P(PSTR("lintel_count = %d\r\n"), mechboard.lintel_count);

	printf_P(PSTR("column_count = %d\r\n"), get_column_count());
	printf_P(PSTR("left1=%d left2=%d right1=%d right2=%d\r\n"),
		 pump_left1_is_full(), pump_left2_is_full(),
		 pump_right1_is_full(), pump_right2_is_full());
	
	printf_P(PSTR("pump_left1 = %d\r\n"), mechboard.pump_left1);
	printf_P(PSTR("pump_left2 = %d\r\n"), mechboard.pump_left2);
	printf_P(PSTR("pump_right1 = %d\r\n"), mechboard.pump_right1);
	printf_P(PSTR("pump_right2 = %d\r\n"), mechboard.pump_right2);

	printf_P(PSTR("servo_lintel_left = %d\r\n"), mechboard.servo_lintel_left);
	printf_P(PSTR("servo_lintel_right = %d\r\n"), mechboard.servo_lintel_right);

#endif
}

prog_char str_mechboard_show_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_show_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_show_result, arg0, str_mechboard_show_arg0);
prog_char str_mechboard_show_arg1[] = "show";
parse_pgm_token_string_t cmd_mechboard_show_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_show_result, arg1, str_mechboard_show_arg1);

prog_char help_mechboard_show[] = "show mechboard status";
parse_pgm_inst_t cmd_mechboard_show = {
	.f = cmd_mechboard_show_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_show_arg0, 
		(prog_void *)&cmd_mechboard_show_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Mechboard_Setmode1 */

/* this structure is filled when cmd_mechboard_setmode1 is parsed successfully */
struct cmd_mechboard_setmode1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_mechboard_setmode1 is parsed successfully */
static void cmd_mechboard_setmode1_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_mechboard_setmode1_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("init")))
		i2c_mechboard_mode_init();
	else if (!strcmp_P(res->arg1, PSTR("manual")))
		i2c_mechboard_mode_manual();
	else if (!strcmp_P(res->arg1, PSTR("pickup")))
		i2c_mechboard_mode_pickup();
	else if (!strcmp_P(res->arg1, PSTR("lazy_harvest")))
		i2c_mechboard_mode_lazy_harvest();
	else if (!strcmp_P(res->arg1, PSTR("harvest")))
		i2c_mechboard_mode_harvest();
	else if (!strcmp_P(res->arg1, PSTR("prepare_get_lintel")))
		i2c_mechboard_mode_prepare_get_lintel();
	else if (!strcmp_P(res->arg1, PSTR("get_lintel")))
		i2c_mechboard_mode_get_lintel();
	else if (!strcmp_P(res->arg1, PSTR("put_lintel")))
		i2c_mechboard_mode_put_lintel();
	else if (!strcmp_P(res->arg1, PSTR("init")))
		i2c_mechboard_mode_init();
	else if (!strcmp_P(res->arg1, PSTR("eject")))
		i2c_mechboard_mode_init();
	else if (!strcmp_P(res->arg1, PSTR("clear")))
		i2c_mechboard_mode_clear();
	else if (!strcmp_P(res->arg1, PSTR("loaded")))
		i2c_mechboard_mode_loaded();
	else if (!strcmp_P(res->arg1, PSTR("store")))
		i2c_mechboard_mode_store();
	else if (!strcmp_P(res->arg1, PSTR("manivelle")))
		i2c_mechboard_mode_manivelle();
	else if (!strcmp_P(res->arg1, PSTR("lazy_pickup")))
		i2c_mechboard_mode_lazy_pickup();
#endif
}

prog_char str_mechboard_setmode1_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_setmode1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode1_result, arg0, str_mechboard_setmode1_arg0);
prog_char str_mechboard_setmode1_arg1[] = "manivelle#init#manual#pickup#prepare_get_lintel#get_lintel#put_lintel1#eject#clear#harvest#lazy_harvest#store#lazy_pickup";
parse_pgm_token_string_t cmd_mechboard_setmode1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode1_result, arg1, str_mechboard_setmode1_arg1);

prog_char help_mechboard_setmode1[] = "set mechboard mode (mode)";
parse_pgm_inst_t cmd_mechboard_setmode1 = {
	.f = cmd_mechboard_setmode1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_setmode1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_setmode1_arg0, 
		(prog_void *)&cmd_mechboard_setmode1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Mechboard_Setmode2 */

/* this structure is filled when cmd_mechboard_setmode2 is parsed successfully */
struct cmd_mechboard_setmode2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_mechboard_setmode2 is parsed successfully */
static void cmd_mechboard_setmode2_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_mechboard_setmode2_result *res = parsed_result;
	uint8_t side = I2C_LEFT_SIDE;

	if (!strcmp_P(res->arg2, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("right")))
		side = I2C_RIGHT_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("center")))
		side = I2C_CENTER_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("auto")))
		side = I2C_AUTO_SIDE;

	if (!strcmp_P(res->arg1, PSTR("prepare_pickup")))
		i2c_mechboard_mode_prepare_pickup(side);
	else if (!strcmp_P(res->arg1, PSTR("push_temple_disc")))
		i2c_mechboard_mode_push_temple_disc(side);
#endif
}

prog_char str_mechboard_setmode2_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_setmode2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode2_result, arg0, str_mechboard_setmode2_arg0);
prog_char str_mechboard_setmode2_arg1[] = "prepare_pickup#push_temple_disc";
parse_pgm_token_string_t cmd_mechboard_setmode2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode2_result, arg1, str_mechboard_setmode2_arg1);
prog_char str_mechboard_setmode2_arg2[] = "left#right#auto#center";
parse_pgm_token_string_t cmd_mechboard_setmode2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode2_result, arg2, str_mechboard_setmode2_arg2);

prog_char help_mechboard_setmode2[] = "set mechboard mode (more, side)";
parse_pgm_inst_t cmd_mechboard_setmode2 = {
	.f = cmd_mechboard_setmode2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_setmode2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_setmode2_arg0, 
		(prog_void *)&cmd_mechboard_setmode2_arg1, 
		(prog_void *)&cmd_mechboard_setmode2_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Mechboard_Setmode3 */

/* this structure is filled when cmd_mechboard_setmode3 is parsed successfully */
struct cmd_mechboard_setmode3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t level;
};

/* function called when cmd_mechboard_setmode3 is parsed successfully */
static void cmd_mechboard_setmode3_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_mechboard_setmode3_result *res = parsed_result;
	if (!strcmp_P(res->arg1, PSTR("autobuild")))
		i2c_mechboard_mode_simple_autobuild(res->level);
	else if (!strcmp_P(res->arg1, PSTR("prepare_build")))
		i2c_mechboard_mode_prepare_build_both(res->level);
	else if (!strcmp_P(res->arg1, PSTR("prepare_inside")))
		i2c_mechboard_mode_prepare_inside_both(res->level);
	else if (!strcmp_P(res->arg1, PSTR("push_temple")))
		i2c_mechboard_mode_push_temple(res->level);
#endif
}

prog_char str_mechboard_setmode3_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_setmode3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode3_result, arg0, str_mechboard_setmode3_arg0);
prog_char str_mechboard_setmode3_arg1[] = "autobuild#prepare_build#prepare_inside#push_temple";
parse_pgm_token_string_t cmd_mechboard_setmode3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode3_result, arg1, str_mechboard_setmode3_arg1);
parse_pgm_token_num_t cmd_mechboard_setmode3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode3_result, level, UINT8);

prog_char help_mechboard_setmode3[] = "set mechboard mode (mode, level)";
parse_pgm_inst_t cmd_mechboard_setmode3 = {
	.f = cmd_mechboard_setmode3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_setmode3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_setmode3_arg0, 
		(prog_void *)&cmd_mechboard_setmode3_arg1, 
		(prog_void *)&cmd_mechboard_setmode3_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Mechboard_Setmode4 */

/* this structure is filled when cmd_mechboard_setmode4 is parsed successfully */
struct cmd_mechboard_setmode4_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t level_l;
	uint8_t count_l;
	uint8_t dist_l;
	uint8_t level_r;
	uint8_t count_r;
	uint8_t dist_r;
	uint8_t do_lintel;
};

/* function called when cmd_mechboard_setmode4 is parsed successfully */
static void cmd_mechboard_setmode4_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_mechboard_setmode4_result *res = parsed_result;
	i2c_mechboard_mode_autobuild(res->level_l, res->count_l, res->dist_l,
				     res->level_r, res->count_r, res->dist_r,
				     res->do_lintel);
#endif
}

prog_char str_mechboard_setmode4_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_setmode4_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode4_result, arg0, str_mechboard_setmode4_arg0);
prog_char str_mechboard_setmode4_arg1[] = "autobuild";
parse_pgm_token_string_t cmd_mechboard_setmode4_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode4_result, arg1, str_mechboard_setmode4_arg1);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, level_l, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, count_l, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, dist_l, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, level_r, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg6 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, count_r, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg7 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, dist_r, UINT8);
parse_pgm_token_num_t cmd_mechboard_setmode4_arg8 = TOKEN_NUM_INITIALIZER(struct cmd_mechboard_setmode4_result, do_lintel, UINT8);

prog_char help_mechboard_setmode4[] = "set mechboard mode (autobuild level_l count_l dist_l level_r count_r dist_r lintel)";
parse_pgm_inst_t cmd_mechboard_setmode4 = {
	.f = cmd_mechboard_setmode4_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_setmode4,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_setmode4_arg0, 
		(prog_void *)&cmd_mechboard_setmode4_arg1, 
		(prog_void *)&cmd_mechboard_setmode4_arg2, 
		(prog_void *)&cmd_mechboard_setmode4_arg3, 
		(prog_void *)&cmd_mechboard_setmode4_arg4, 
		(prog_void *)&cmd_mechboard_setmode4_arg5, 
		(prog_void *)&cmd_mechboard_setmode4_arg6, 
		(prog_void *)&cmd_mechboard_setmode4_arg7, 
		(prog_void *)&cmd_mechboard_setmode4_arg8, 
		NULL,
	},
};

/**********************************************************/
/* Mechboard_Setmode5 */

/* this structure is filled when cmd_mechboard_setmode5 is parsed successfully */
struct cmd_mechboard_setmode5_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	fixed_string_t arg3;
};

/* function called when cmd_mechboard_setmode5 is parsed successfully */
static void cmd_mechboard_setmode5_parsed(void *parsed_result, void * data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_mechboard_setmode5_result *res = parsed_result;
	uint8_t side = I2C_LEFT_SIDE, next_mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;

	if (!strcmp_P(res->arg2, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("right")))
		side = I2C_RIGHT_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("center")))
		side = I2C_CENTER_SIDE;
	else if (!strcmp_P(res->arg2, PSTR("auto")))
		side = I2C_AUTO_SIDE;

	if (!strcmp_P(res->arg3, PSTR("harvest")))
		next_mode = I2C_MECHBOARD_MODE_HARVEST;
	else if (!strcmp_P(res->arg3, PSTR("lazy_harvest")))
		next_mode = I2C_MECHBOARD_MODE_LAZY_HARVEST;
	else if (!strcmp_P(res->arg3, PSTR("pickup")))
		next_mode = I2C_MECHBOARD_MODE_PICKUP;
	else if (!strcmp_P(res->arg3, PSTR("clear")))
		next_mode = I2C_MECHBOARD_MODE_CLEAR;
	else if (!strcmp_P(res->arg3, PSTR("store")))
		next_mode = I2C_MECHBOARD_MODE_STORE;
	else if (!strcmp_P(res->arg3, PSTR("lazy_pickup")))
		next_mode = I2C_MECHBOARD_MODE_LAZY_PICKUP;

	if (!strcmp_P(res->arg1, PSTR("prepare_pickup")))
		i2c_mechboard_mode_prepare_pickup_next(side, next_mode);
#endif
}

prog_char str_mechboard_setmode5_arg0[] = "mechboard";
parse_pgm_token_string_t cmd_mechboard_setmode5_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode5_result, arg0, str_mechboard_setmode5_arg0);
prog_char str_mechboard_setmode5_arg1[] = "prepare_pickup";
parse_pgm_token_string_t cmd_mechboard_setmode5_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode5_result, arg1, str_mechboard_setmode5_arg1);
prog_char str_mechboard_setmode5_arg2[] = "left#right#auto#center";
parse_pgm_token_string_t cmd_mechboard_setmode5_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode5_result, arg2, str_mechboard_setmode5_arg2);
prog_char str_mechboard_setmode5_arg3[] = "harvest#pickup#store#lazy_harvest#lazy_pickup#clear";
parse_pgm_token_string_t cmd_mechboard_setmode5_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_mechboard_setmode5_result, arg3, str_mechboard_setmode5_arg3);

prog_char help_mechboard_setmode5[] = "set mechboard mode (more, side)";
parse_pgm_inst_t cmd_mechboard_setmode5 = {
	.f = cmd_mechboard_setmode5_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_mechboard_setmode5,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_mechboard_setmode5_arg0, 
		(prog_void *)&cmd_mechboard_setmode5_arg1, 
		(prog_void *)&cmd_mechboard_setmode5_arg2, 
		(prog_void *)&cmd_mechboard_setmode5_arg3, 
		NULL,
	},
};

/**********************************************************/
/* pickup wheels */

/* this structure is filled when cmd_pickup_wheels is parsed successfully */
struct cmd_pickup_wheels_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_pickup_wheels is parsed successfully */
static void cmd_pickup_wheels_parsed(void *parsed_result, void *data)
{
	struct cmd_pickup_wheels_result *res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("on")))
		pickup_wheels_on();	
	else
		pickup_wheels_off();
}

prog_char str_pickup_wheels_arg0[] = "pickup_wheels";
parse_pgm_token_string_t cmd_pickup_wheels_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pickup_wheels_result, arg0, str_pickup_wheels_arg0);
prog_char str_pickup_wheels_arg1[] = "on#off";
parse_pgm_token_string_t cmd_pickup_wheels_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pickup_wheels_result, arg1, str_pickup_wheels_arg1);

prog_char help_pickup_wheels[] = "Enable/disable pickup wheels";
parse_pgm_inst_t cmd_pickup_wheels = {
	.f = cmd_pickup_wheels_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pickup_wheels,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pickup_wheels_arg0,
		(prog_void *)&cmd_pickup_wheels_arg1,
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
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_beacon_start_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("start")))
		i2c_sensorboard_set_beacon(1);
	else
		i2c_sensorboard_set_beacon(0);
#endif
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
/* Pump_Current */

/* this structure is filled when cmd_pump_current is parsed successfully */
struct cmd_pump_current_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_pump_current is parsed successfully */
static void cmd_pump_current_parsed(__attribute__((unused)) void *parsed_result,
				    __attribute__((unused)) void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	printf_P(PSTR("l1=%d l2=%d r1=%d r2=%d\r\n"),
		 sensor_get_adc(ADC_CSENSE3), sensor_get_adc(ADC_CSENSE4),
		 mechboard.pump_right1_current, mechboard.pump_right2_current);
#endif
}

prog_char str_pump_current_arg0[] = "pump_current";
parse_pgm_token_string_t cmd_pump_current_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pump_current_result, arg0, str_pump_current_arg0);
prog_char str_pump_current_arg1[] = "show";
parse_pgm_token_string_t cmd_pump_current_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pump_current_result, arg1, str_pump_current_arg1);

prog_char help_pump_current[] = "dump pump current";
parse_pgm_inst_t cmd_pump_current = {
	.f = cmd_pump_current_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pump_current,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pump_current_arg0, 
		(prog_void *)&cmd_pump_current_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Build_Test */

/* this structure is filled when cmd_build_test is parsed successfully */
struct cmd_build_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_build_test is parsed successfully */
static void cmd_build_test_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	//struct cmd_build_test_result *res = parsed_result;

	printf_P(PSTR("lintel must be there\r\n"));
	i2c_mechboard_mode_prepare_pickup_next(I2C_LEFT_SIDE,
					       I2C_MECHBOARD_MODE_HARVEST);
	wait_ms(500);

	printf_P(PSTR("Insert 4 colums\r\n"));
	while (get_column_count() != 4);

	i2c_mechboard_mode_prepare_build_both(0);
	trajectory_d_rel(&mainboard.traj, 200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	wait_ms(500);

	i2c_mechboard_mode_simple_autobuild(0);
	wait_ms(100);
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

	trajectory_d_rel(&mainboard.traj, -200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	i2c_mechboard_mode_prepare_pickup_next(I2C_LEFT_SIDE,
					       I2C_MECHBOARD_MODE_HARVEST);

	while (get_column_count() != 3);

	i2c_mechboard_mode_prepare_build_both(3);
	trajectory_d_rel(&mainboard.traj, 200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	wait_ms(500);

	i2c_mechboard_mode_autobuild(3, 1, I2C_AUTOBUILD_DEFAULT_DIST,
				     3, 2,I2C_AUTOBUILD_DEFAULT_DIST, 0);
	i2cproto_wait_update();
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

	trajectory_d_rel(&mainboard.traj, -200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	i2c_mechboard_mode_prepare_pickup(I2C_RIGHT_SIDE);
	wait_ms(500);

	i2c_mechboard_mode_harvest();
	while (get_column_count() != 3);

	i2c_mechboard_mode_prepare_build_both(5);
	trajectory_d_rel(&mainboard.traj, 200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	wait_ms(1000);

	i2c_mechboard_mode_autobuild(4, 2, I2C_AUTOBUILD_DEFAULT_DIST,
				     5, 1, I2C_AUTOBUILD_DEFAULT_DIST, 0);
	i2cproto_wait_update();
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

	trajectory_d_rel(&mainboard.traj, -200);
#endif
}

prog_char str_build_test_arg0[] = "build_test";
parse_pgm_token_string_t cmd_build_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_build_test_result, arg0, str_build_test_arg0);

prog_char help_build_test[] = "Build_Test function";
parse_pgm_inst_t cmd_build_test = {
	.f = cmd_build_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_build_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_build_test_arg0, 
		NULL,
	},
};


/**********************************************************/
/* Column_Test */

/* this structure is filled when cmd_column_test is parsed successfully */
struct cmd_column_test_result {
	fixed_string_t arg0;
	uint8_t level;
	int16_t dist;
	int8_t a1;
	int8_t a2;
	int8_t a3;
	int16_t arm_dist;
	int8_t nb_col;
};

/* function called when cmd_column_test is parsed successfully */
static void cmd_column_test_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_column_test_result *res = parsed_result;
	uint8_t level = res->level, debug = 0;
	uint8_t c, push = 0;

	/* default conf */
	if (data) {
		res->dist = 70;
		res->a1 = -20;
		res->a2 =  40;
		res->a3 = -20;
		res->arm_dist = 220;
		res->nb_col = 2;
	}

	if (!strcmp_P(res->arg0, PSTR("column_test_debug")))
		debug = 1;
	if (!strcmp_P(res->arg0, PSTR("column_test_push")))
		push = 1;

	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	
	/* Go to disc */

	trajectory_d_rel(&mainboard.traj, 200);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* go back, insert colums */

	trajectory_d_rel(&mainboard.traj, -200);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	i2c_mechboard_mode_prepare_pickup_next(I2C_LEFT_SIDE,
					       I2C_MECHBOARD_MODE_HARVEST);
	printf_P(PSTR("Insert 4 colums\r\n"));
	while (get_column_count() != 4);

	/* build with left arm */

	i2c_mechboard_mode_prepare_inside_both(level);
	trajectory_d_rel(&mainboard.traj, 200-(res->dist));
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	if (debug)
		c = cmdline_getchar_wait();

	trajectory_a_rel(&mainboard.traj, res->a1);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	if (debug)
		c = cmdline_getchar_wait();

	i2c_mechboard_mode_prepare_build_select(level, -1);
	time_wait_ms(200);
	if (debug)
		c = cmdline_getchar_wait();
	i2c_mechboard_mode_autobuild(level, res->nb_col, res->arm_dist,
				     0, 0, res->arm_dist, 0);
	while (get_mechboard_mode() != I2C_MECHBOARD_MODE_AUTOBUILD);
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

	if (debug)
		c = cmdline_getchar_wait();
	i2c_mechboard_mode_prepare_inside_select(level+res->nb_col, -1);

	if (debug)
		c = cmdline_getchar_wait();
	/* build with right arm */

	trajectory_a_rel(&mainboard.traj, res->a2);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	if (debug)
		c = cmdline_getchar_wait();
	/* only ok for nb_col == 2 */
	if ((level + res->nb_col) >= 7)
		i2c_mechboard_mode_prepare_build_select(-1, level + res->nb_col + 1);
	else
		i2c_mechboard_mode_prepare_build_select(-1, level + res->nb_col);
	time_wait_ms(200);
	if (debug)
		c = cmdline_getchar_wait();
	i2c_mechboard_mode_autobuild(0, 0, res->arm_dist,
				     level + res->nb_col, res->nb_col,
				     res->arm_dist, 0);
	while (get_mechboard_mode() != I2C_MECHBOARD_MODE_AUTOBUILD);
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

		
	if (push) {
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
		trajectory_d_rel(&mainboard.traj, -100);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		i2c_mechboard_mode_push_temple_disc(I2C_RIGHT_SIDE);
		time_wait_ms(500);
		trajectory_d_rel(&mainboard.traj, 100);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}
	else if (level == 1 || level == 0) {
		trajectory_d_rel(&mainboard.traj, -100);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		i2c_mechboard_mode_push_temple(level);
		time_wait_ms(400);
		strat_set_speed(200, SPEED_ANGLE_SLOW);
		trajectory_d_rel(&mainboard.traj, 120);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	if (debug)
		c = cmdline_getchar_wait();
	i2c_mechboard_mode_prepare_inside_select(-1, level+res->nb_col*2);

	if (debug)
		c = cmdline_getchar_wait();

	trajectory_a_rel(&mainboard.traj, res->a3);
	wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	if (debug)
		c = cmdline_getchar_wait();
	/* go back, insert colums */

	trajectory_d_rel(&mainboard.traj, -100);

	return;
#endif
}

prog_char str_column_test_arg0[] = "column_test#column_test_debug#column_test_push";
parse_pgm_token_string_t cmd_column_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_column_test_result, arg0, str_column_test_arg0);
parse_pgm_token_num_t cmd_column_test_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, level, UINT8);

prog_char help_column_test[] = "Column_Test function (level)";
parse_pgm_inst_t cmd_column_test = {
	.f = cmd_column_test_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_column_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_column_test_arg0, 
		(prog_void *)&cmd_column_test_arg1, 
		NULL,
	},
};

parse_pgm_token_num_t cmd_column_test_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, dist, INT16);
parse_pgm_token_num_t cmd_column_test_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, a1, INT8);
parse_pgm_token_num_t cmd_column_test_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, a2, INT8);
parse_pgm_token_num_t cmd_column_test_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, a3, INT8);
parse_pgm_token_num_t cmd_column_test_arg6 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, arm_dist, INT16);
parse_pgm_token_num_t cmd_column_test_arg7 = TOKEN_NUM_INITIALIZER(struct cmd_column_test_result, nb_col, INT8);

prog_char help_column_test2[] = "Column_Test function (level, dist, a1, a2, a3, arm_dist, nb_col)";
parse_pgm_inst_t cmd_column_test2 = {
	.f = cmd_column_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_column_test2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_column_test_arg0, 
		(prog_void *)&cmd_column_test_arg1, 
		(prog_void *)&cmd_column_test_arg2, 
		(prog_void *)&cmd_column_test_arg3, 
		(prog_void *)&cmd_column_test_arg4, 
		(prog_void *)&cmd_column_test_arg5, 
		(prog_void *)&cmd_column_test_arg6, 
		(prog_void *)&cmd_column_test_arg7, 
		NULL,
	},
};


/**********************************************************/
/* Pickup_Test */

/* this structure is filled when cmd_pickup_test is parsed successfully */
struct cmd_pickup_test_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t dist;
};

/* return red or green sensor */
#define COLOR_IR_SENSOR()						\
	({								\
		uint8_t __ret = 0;					\
		if (side == I2C_RIGHT_SIDE)				\
			__ret = sensor_get(S_DISP_RIGHT);		\
		else							\
			__ret = sensor_get(S_DISP_LEFT);		\
		__ret;							\
	})								\
/* column dispensers */
#define COL_SCAN_MARGIN 200
/* distance between the wheel axis and the IR sensor */

/* function called when cmd_pickup_test is parsed successfully */
static void cmd_pickup_test_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	uint8_t err, side, first_try = 1;
	int8_t cols_count_before, cols_count_after, cols;
	struct cmd_pickup_test_result *res = parsed_result;
	int16_t pos1, pos2, pos;
        microseconds us;
	int16_t dist = res->dist;
	uint8_t timeout = 0;

	if (!strcmp_P(res->arg1, PSTR("left")))
		side = I2C_LEFT_SIDE;
	else
		side = I2C_RIGHT_SIDE;

	i2c_mechboard_mode_prepare_pickup(I2C_AUTO_SIDE);
	cols_count_before = get_column_count();
	position_set(&mainboard.pos, 0, 0, 0);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -1000);
	err = WAIT_COND_OR_TRAJ_END(!COLOR_IR_SENSOR(), TRAJ_FLAGS_NO_NEAR);
	if (err) /* we should not reach end */
		goto fail;
	pos1 = position_get_x_s16(&mainboard.pos);
	printf_P(PSTR("pos1 = %d\r\n"), pos1);

	err = WAIT_COND_OR_TRAJ_END(COLOR_IR_SENSOR(), TRAJ_FLAGS_NO_NEAR);
	if (err)
		goto fail;
	pos2 = position_get_x_s16(&mainboard.pos);
	printf_P(PSTR("pos2 = %d\r\n"), pos2);

	pos = ABS(pos1 - pos2);
	printf_P(PSTR("pos = %d\r\n"), pos);

	trajectory_d_rel(&mainboard.traj, -dist + pos/2);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	if (side == I2C_LEFT_SIDE)
		trajectory_a_rel(&mainboard.traj, 90);
	else
		trajectory_a_rel(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	pickup_wheels_on();	
 retry:
	if (first_try)
    		i2c_mechboard_mode_lazy_harvest();
	else
    		i2c_mechboard_mode_prepare_pickup(I2C_AUTO_SIDE);
	first_try = 0;

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, 300);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST | END_NEAR);
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	err = strat_calib(600, TRAJ_FLAGS_SMALL_DIST);

	trajectory_d_rel(&mainboard.traj, -DIST_BACK_DISPENSER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	position_set(&mainboard.pos, 0, 0, 0);
	if (get_mechboard_mode() == I2C_MECHBOARD_MODE_PREPARE_EJECT) {
		strat_eject_col(90, 0);
		goto retry;
	}

	/* start to pickup with finger / arms */

	printf_P(PSTR("%s pickup now\r\n"), __FUNCTION__);
	i2c_mechboard_mode_pickup();
	WAIT_COND_OR_TIMEOUT(get_mechboard_mode() == 
			     I2C_MECHBOARD_MODE_PICKUP, 100);
        us = time_get_us2();
	cols = get_column_count();
	while (get_mechboard_mode() == I2C_MECHBOARD_MODE_PICKUP) {
		if (get_column_count() != cols) {
			cols = get_column_count();
			us = time_get_us2();
		}
		if ((get_column_count() - cols_count_before) >= 4) {
			printf_P(PSTR("%s no more cols in disp\r\n"), __FUNCTION__);
			break;
		}
		/* 1 second timeout */
		if (time_get_us2() - us > 1500000L) {
			printf_P(PSTR("%s timeout\r\n"), __FUNCTION__);
			timeout = 1;
			break;
		}
	}

	/* eject if we found a bad color column */
	
	if (get_mechboard_mode() == I2C_MECHBOARD_MODE_PREPARE_EJECT) {
		strat_eject_col(90, 0);
		goto retry;
	}

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -250);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST | END_NEAR);

	cols_count_after = get_column_count();
	cols = cols_count_after - cols_count_before;
	DEBUG(E_USER_STRAT, "%s we got %d cols", __FUNCTION__, cols);

	pickup_wheels_off();
	i2c_mechboard_mode_clear();

	wait_ms(1000);
	return;
 fail:
	printf_P(PSTR("failed\r\n"));
	strat_hardstop();
#endif
}

prog_char str_pickup_test_arg0[] = "pickup_test";
parse_pgm_token_string_t cmd_pickup_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pickup_test_result, arg0, str_pickup_test_arg0);
prog_char str_pickup_test_arg1[] = "left#right";
parse_pgm_token_string_t cmd_pickup_test_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pickup_test_result, arg1, str_pickup_test_arg1);
parse_pgm_token_num_t cmd_pickup_test_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pickup_test_result, dist, INT16);

prog_char help_pickup_test[] = "Pickup_Test function";
parse_pgm_inst_t cmd_pickup_test = {
	.f = cmd_pickup_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pickup_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pickup_test_arg0,
		(prog_void *)&cmd_pickup_test_arg1,
		(prog_void *)&cmd_pickup_test_arg2,
		NULL,
	},
};

/**********************************************************/
/* Lintel_Test */

/* this structure is filled when cmd_lintel_test is parsed successfully */
struct cmd_lintel_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_lintel_test is parsed successfully */
static void cmd_lintel_test_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	uint8_t err, first_try = 1, right_ok, left_ok;
	int16_t left_cur, right_cur;
	
	i2c_mechboard_mode_prepare_get_lintel();
	time_wait_ms(500);
 retry:
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, 500);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err) && err != END_BLOCKING)
		goto fail;
	
	i2c_mechboard_mode_get_lintel();
	time_wait_ms(500);

	left_cur = sensor_get_adc(ADC_CSENSE3);
	left_ok = (left_cur > I2C_MECHBOARD_CURRENT_COLUMN);
	right_cur = mechboard.pump_right1_current;
	right_ok = (right_cur > I2C_MECHBOARD_CURRENT_COLUMN);

	printf_P(PSTR("left_ok=%d (%d), right_ok=%d (%d)\r\n"),
		 left_ok, left_cur, right_ok, right_cur);
	if (first_try) {
		if (!right_ok && !left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
		}
		else if (right_ok && !left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
			strat_set_speed(500, 500);
			trajectory_d_a_rel(&mainboard.traj, -150, 30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			trajectory_d_a_rel(&mainboard.traj, -140, -30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			first_try = 0;
			goto retry;
		}
		else if (!right_ok && left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
			strat_set_speed(500, 500);
			trajectory_d_a_rel(&mainboard.traj, -150, -30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			trajectory_d_a_rel(&mainboard.traj, -140, 30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			first_try = 0;
			goto retry;
		}
		/* else, lintel is ok */
		else {
			i2c_mechboard_mode_put_lintel();
		}
	}
	else {
		if (right_ok && left_ok) {
			/* lintel is ok */
			i2c_mechboard_mode_put_lintel();
		}
		else {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
		}
	}

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -250);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	return;
	
fail:
	printf_P(PSTR("fail\r\n"));
	return;
#endif
}

prog_char str_lintel_test_arg0[] = "lintel_test";
parse_pgm_token_string_t cmd_lintel_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_lintel_test_result, arg0, str_lintel_test_arg0);

prog_char help_lintel_test[] = "Lintel_Test function";
parse_pgm_inst_t cmd_lintel_test = {
	.f = cmd_lintel_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_lintel_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_lintel_test_arg0,
		NULL,
	},
};

/**********************************************************/
/* Scan_Test */

/* this structure is filled when cmd_scan_test is parsed successfully */
struct cmd_scan_test_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t start_dist;
	int16_t scan_dist;
	int16_t scan_speed;
	int16_t center_x;
	int16_t center_y;
	uint8_t level;
};

#define SCAN_MODE_CHECK_TEMPLE 0
#define SCAN_MODE_SCAN_COL     1
#define SCAN_MODE_SCAN_TEMPLE  2
#define SCAN_MODE_TRAJ_ONLY    3

/* function called when cmd_scan_test is parsed successfully */
static void cmd_scan_test_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	uint8_t err, mode=0, c;
	int16_t pos1x, pos1y, dist;
	struct cmd_scan_test_result *res = parsed_result;
	int16_t back_mm = 0;

	int16_t ckpt_rel_x = 0, ckpt_rel_y = 0;

	double center_abs_x, center_abs_y;
	double ckpt_rel_d, ckpt_rel_a;
	double ckpt_abs_x, ckpt_abs_y;

	if (!strcmp_P(res->arg1, PSTR("traj_only")))
		mode = SCAN_MODE_TRAJ_ONLY;
	else if (!strcmp_P(res->arg1, PSTR("check_temple")))
		mode = SCAN_MODE_CHECK_TEMPLE;
	else if (!strcmp_P(res->arg1, PSTR("scan_col")))
		mode = SCAN_MODE_SCAN_COL;
	else if (!strcmp_P(res->arg1, PSTR("scan_temple")))
		mode = SCAN_MODE_SCAN_TEMPLE;

	/* go to disc */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	trajectory_d_rel(&mainboard.traj, 400);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (err != END_BLOCKING) 
		return;

	/* save absolute position of disc */
	rel_da_to_abs_xy(265, 0, &center_abs_x, &center_abs_y);

	/* go back and prepare to scan */
	strat_set_speed(1000, 1000);
	trajectory_d_a_rel(&mainboard.traj, -140, 130);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		return;

	/* prepare scanner arm */
	if (mode != SCAN_MODE_TRAJ_ONLY)
		i2c_sensorboard_scanner_prepare();
	time_wait_ms(250);

	strat_set_speed(res->scan_speed, 1000);

	pos1x = position_get_x_s16(&mainboard.pos);
	pos1y = position_get_y_s16(&mainboard.pos);
	trajectory_d_rel(&mainboard.traj, -res->scan_dist);
	
	while (1) {
		err = test_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (err != 0)
			break;
		
		dist = distance_from_robot(pos1x, pos1y);

		if (dist > res->start_dist)
			break;

		if (get_scanner_status() & I2C_SCAN_MAX_COLUMN) {
			err = END_ERROR;
			break;
		}
	}
	
	if (err) {
		if (TRAJ_SUCCESS(err))
			err = END_ERROR; /* should not reach end */
		strat_hardstop();
		trajectory_goto_xy_abs(&mainboard.traj, pos1x, pos1y);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (mode != SCAN_MODE_TRAJ_ONLY)
			i2c_sensorboard_scanner_stop();
		return;
	}

	/* start the scanner */

	if (mode != SCAN_MODE_TRAJ_ONLY)
		i2c_sensorboard_scanner_start();

	err = WAIT_COND_OR_TRAJ_END(get_scanner_status() & I2C_SCAN_MAX_COLUMN,
				    TRAJ_FLAGS_NO_NEAR);
	if (err == 0)
		err = END_ERROR;
	if (!TRAJ_SUCCESS(err)) {
		strat_hardstop();
		trajectory_goto_xy_abs(&mainboard.traj, pos1x, pos1y);
		wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (mode != SCAN_MODE_TRAJ_ONLY)
			i2c_sensorboard_scanner_stop();
		return;
	}

	if (mode == SCAN_MODE_TRAJ_ONLY)
		return;

	wait_scan_done(10000);

	i2c_sensorboard_scanner_stop();

	if (mode == SCAN_MODE_CHECK_TEMPLE) {
		i2c_sensorboard_scanner_algo_check(res->level,
						   res->center_x, res->center_y);
		i2cproto_wait_update();
		wait_scan_done(10000);
		scanner_dump_state();

		if (sensorboard.dropzone_h == -1) {
			printf_P(PSTR("-- try to build a temple\r\n"));
			res->center_x = 15;
			res->center_y = 13;
			mode = SCAN_MODE_SCAN_TEMPLE;
		}
	}

	if (mode == SCAN_MODE_SCAN_TEMPLE) {
		i2c_sensorboard_scanner_algo_temple(I2C_SCANNER_ZONE_DISC,
						    res->center_x,
						    res->center_y);
		i2cproto_wait_update();
		wait_scan_done(10000);
		scanner_dump_state();
		
		if (sensorboard.dropzone_h == -1 ||
		    strat_scan_get_checkpoint(mode, &ckpt_rel_x,
					      &ckpt_rel_y, &back_mm)) {
			printf_P(PSTR("-- try to build a column\r\n"));
			mode = SCAN_MODE_SCAN_COL;
		}
	}

	if (mode == SCAN_MODE_SCAN_COL) {
		i2c_sensorboard_scanner_algo_column(I2C_SCANNER_ZONE_DISC,
						    res->center_x, res->center_y);
		i2cproto_wait_update();
		wait_scan_done(10000);
		scanner_dump_state();

		if (sensorboard.dropzone_h == -1 ||
		    strat_scan_get_checkpoint(mode, &ckpt_rel_x,
					      &ckpt_rel_y, &back_mm)) {
			return;
		}
	}

	if (sensorboard.dropzone_h == -1)
		return;

	if (mode == SCAN_MODE_CHECK_TEMPLE) {
		ckpt_rel_x = 220;
		ckpt_rel_y = 100;
	}


	printf_P(PSTR("rel xy for ckpt is %d,%d\r\n"), ckpt_rel_x, ckpt_rel_y);

	rel_xy_to_abs_xy(ckpt_rel_x, ckpt_rel_y, &ckpt_abs_x, &ckpt_abs_y);
	abs_xy_to_rel_da(ckpt_abs_x, ckpt_abs_y, &ckpt_rel_d, &ckpt_rel_a);

	printf_P(PSTR("abs ckpt is %2.2f,%2.2f\r\n"), ckpt_abs_x, ckpt_abs_y);

	printf_P(PSTR("ok ? (y/n)\r\n"));

	c = cmdline_getchar_wait();

	if (c != 'y')
		return;

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* intermediate checkpoint for some positions */
	if ( (DEG(ckpt_rel_a) < 0 && DEG(ckpt_rel_a) > -90) ) {
		trajectory_goto_xy_rel(&mainboard.traj, 200, 100);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			return;
	}

	trajectory_goto_xy_abs(&mainboard.traj, ckpt_abs_x, ckpt_abs_y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		return;

	trajectory_turnto_xy(&mainboard.traj, center_abs_x, center_abs_y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		return;

	c = cmdline_getchar_wait();

	pos1x = position_get_x_s16(&mainboard.pos);
	pos1y = position_get_y_s16(&mainboard.pos);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, 200);
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(pos1x, pos1y) > 200,
				    TRAJ_FLAGS_SMALL_DIST);
	if (err == 0) {
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, 400);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}
	if (err != END_BLOCKING) 
		return;

	if (back_mm) {
		trajectory_d_rel(&mainboard.traj, -back_mm);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}
#endif
}

prog_char str_scan_test_arg0[] = "scan_test";
parse_pgm_token_string_t cmd_scan_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scan_test_result, arg0, str_scan_test_arg0);
prog_char str_scan_test_arg1[] = "traj_only#scan_col#scan_temple";
parse_pgm_token_string_t cmd_scan_test_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_scan_test_result, arg1, str_scan_test_arg1);
parse_pgm_token_num_t cmd_scan_test_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, start_dist, INT16);
parse_pgm_token_num_t cmd_scan_test_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, scan_dist, INT16);
parse_pgm_token_num_t cmd_scan_test_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, scan_speed, INT16);
parse_pgm_token_num_t cmd_scan_test_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, center_x, INT16);
parse_pgm_token_num_t cmd_scan_test_arg6 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, center_y, INT16);

prog_char help_scan_test[] = "Scan_Test function (start_dist, scan_dist, speed_dist, centerx, centery)";
parse_pgm_inst_t cmd_scan_test = {
	.f = cmd_scan_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scan_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_test_arg0,
		(prog_void *)&cmd_scan_test_arg1,
		(prog_void *)&cmd_scan_test_arg2,
		(prog_void *)&cmd_scan_test_arg3,
		(prog_void *)&cmd_scan_test_arg4,
		(prog_void *)&cmd_scan_test_arg5,
		(prog_void *)&cmd_scan_test_arg6,
		NULL,
	},
};

prog_char str_scan_test_arg1b[] = "check_temple";
parse_pgm_token_string_t cmd_scan_test_arg1b = TOKEN_STRING_INITIALIZER(struct cmd_scan_test_result, arg1, str_scan_test_arg1b);
parse_pgm_token_num_t cmd_scan_test_arg7 = TOKEN_NUM_INITIALIZER(struct cmd_scan_test_result, level, UINT8);

prog_char help_scan_test2[] = "Scan_Test function (start_dist, scan_dist, speed_dist, templex, templey, level)";
parse_pgm_inst_t cmd_scan_test2 = {
	.f = cmd_scan_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scan_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scan_test_arg0,
		(prog_void *)&cmd_scan_test_arg1b,
		(prog_void *)&cmd_scan_test_arg2,
		(prog_void *)&cmd_scan_test_arg3,
		(prog_void *)&cmd_scan_test_arg4,
		(prog_void *)&cmd_scan_test_arg5,
		(prog_void *)&cmd_scan_test_arg6,
		(prog_void *)&cmd_scan_test_arg7,
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
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
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
/* Scanner */

/* this structure is filled when cmd_scanner is parsed successfully */
struct cmd_scanner_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_scanner is parsed successfully */
static void cmd_scanner_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_scanner_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("prepare"))) {
		i2c_sensorboard_scanner_prepare();
	}
	else if (!strcmp_P(res->arg1, PSTR("stop"))) {
		i2c_sensorboard_scanner_stop();
	}
	else if (!strcmp_P(res->arg1, PSTR("start"))) {
		i2c_sensorboard_scanner_start();
	}
	else if (!strcmp_P(res->arg1, PSTR("algo_col"))) {
		i2c_sensorboard_scanner_algo_column(I2C_SCANNER_ZONE_DISC,
						     15, 15);
	}
	else if (!strcmp_P(res->arg1, PSTR("algo_check"))) {
		i2c_sensorboard_scanner_algo_check(2, 15, 15); // XXX
	}
	else if (!strcmp_P(res->arg1, PSTR("calib"))) {
		i2c_sensorboard_scanner_calib();
	}
	else if (!strcmp_P(res->arg1, PSTR("show"))) {
		scanner_dump_state();
	}
#endif
}

prog_char str_scanner_arg0[] = "scanner";
parse_pgm_token_string_t cmd_scanner_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scanner_result, arg0, str_scanner_arg0);
prog_char str_scanner_arg1[] = "prepare#start#algo_col#algo_check#stop#show#calib";
parse_pgm_token_string_t cmd_scanner_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_scanner_result, arg1, str_scanner_arg1);

prog_char help_scanner[] = "send commands to scanner";
parse_pgm_inst_t cmd_scanner = {
	.f = cmd_scanner_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scanner,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scanner_arg0, 
		(prog_void *)&cmd_scanner_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Build_Z1 */

/* this structure is filled when cmd_build_z1 is parsed successfully */
struct cmd_build_z1_result {
	fixed_string_t arg0;
	uint8_t level;
	int16_t d1;
	int16_t d2;
	int16_t d3;
};

/* function called when cmd_build_z1 is parsed successfully */
static void cmd_build_z1_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_build_z1_result *res = parsed_result;

	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, 400);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	trajectory_d_rel(&mainboard.traj, -200);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	i2c_mechboard_mode_prepare_pickup_next(I2C_LEFT_SIDE,
					       I2C_MECHBOARD_MODE_HARVEST);

	while (get_column_count() != 4);

	i2c_mechboard_mode_prepare_build_both(res->level);
	time_wait_ms(500);

	trajectory_d_rel(&mainboard.traj, 400);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, -res->d1);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	i2c_mechboard_mode_autobuild(res->level, 2, I2C_AUTOBUILD_DEFAULT_DIST,
				     res->level, 2, I2C_AUTOBUILD_DEFAULT_DIST,
				     1);
	WAIT_COND_OR_TIMEOUT(get_mechboard_mode() == 
			     I2C_MECHBOARD_MODE_AUTOBUILD, 100);
	WAIT_COND_OR_TIMEOUT(get_mechboard_mode() != 
			     I2C_MECHBOARD_MODE_AUTOBUILD, 10000);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, -res->d2);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	i2c_mechboard_mode_push_temple(1);
	time_wait_ms(400);
	strat_set_speed(200, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, res->d3);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
#endif
}

prog_char str_build_z1_arg0[] = "build_z1";
parse_pgm_token_string_t cmd_build_z1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_build_z1_result, arg0, str_build_z1_arg0);
parse_pgm_token_num_t cmd_build_z1_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_build_z1_result, level, UINT8);
parse_pgm_token_num_t cmd_build_z1_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_build_z1_result, d1, INT16);
parse_pgm_token_num_t cmd_build_z1_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_build_z1_result, d2, INT16);
parse_pgm_token_num_t cmd_build_z1_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_build_z1_result, d3, INT16);

prog_char help_build_z1[] = "Build_Z1 function (level, d1, d2, d3)";
parse_pgm_inst_t cmd_build_z1 = {
	.f = cmd_build_z1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_build_z1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_build_z1_arg0, 
		(prog_void *)&cmd_build_z1_arg1, 
		(prog_void *)&cmd_build_z1_arg2, 
		(prog_void *)&cmd_build_z1_arg3, 
		(prog_void *)&cmd_build_z1_arg4, 
		NULL,
	},
};

#ifdef TEST_BEACON
/**********************************************************/
/* Beacon_Opp_Dump */

/* this structure is filled when cmd_beacon_opp_dump is parsed successfully */
struct cmd_beacon_opp_dump_result {
	fixed_string_t arg0;
};

void beacon_dump_samples(void);

/* function called when cmd_beacon_opp_dump is parsed successfully */
static void cmd_beacon_opp_dump_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	beacon_dump_samples();
#endif
}

prog_char str_beacon_opp_dump_arg0[] = "beacon_opp_dump";
parse_pgm_token_string_t cmd_beacon_opp_dump_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_opp_dump_result, arg0, str_beacon_opp_dump_arg0);

prog_char help_beacon_opp_dump[] = "Dump beacon samples";
parse_pgm_inst_t cmd_beacon_opp_dump = {
	.f = cmd_beacon_opp_dump_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon_opp_dump,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_opp_dump_arg0, 
		NULL,
	},
};
#endif

/**********************************************************/
/* Circle_Radius */

/* this structure is filled when cmd_circle_radius is parsed successfully */
struct cmd_circle_radius_result {
	fixed_string_t arg0;
	int32_t radius;
};
void circle_get_da_speed_from_radius(struct trajectory *traj,
				double radius_mm,
				double *speed_d,
				double *speed_a);
/* function called when cmd_circle_radius is parsed successfully */
static void cmd_circle_radius_parsed(void *parsed_result, void *data)
{
	struct cmd_circle_radius_result *res = parsed_result;
	double d,a;
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	circle_get_da_speed_from_radius(&mainboard.traj, res->radius, &d, &a);
	printf_P(PSTR("d=%2.2f a=%2.2f\r\n"), d, a);
}

prog_char str_circle_radius_arg0[] = "circle_radius";
parse_pgm_token_string_t cmd_circle_radius_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_circle_radius_result, arg0, str_circle_radius_arg0);
parse_pgm_token_num_t cmd_circle_radius_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_circle_radius_result, radius, INT32);

prog_char help_circle_radius[] = "Circle_Radius function";
parse_pgm_inst_t cmd_circle_radius = {
	.f = cmd_circle_radius_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_circle_radius,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_circle_radius_arg0,
		(prog_void *)&cmd_circle_radius_arg1,
		NULL,
	},
};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
	int32_t radius;
};
void circle_get_da_speed_from_radius(struct trajectory *traj,
				double radius_mm,
				double *speed_d,
				double *speed_a);
/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(void *parsed_result, void *data)
{
	struct cmd_test_result *res = parsed_result;
	double d,a;
	uint8_t err;

	strat_reset_pos(1000, 500, 0);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	circle_get_da_speed_from_radius(&mainboard.traj, res->radius, &d, &a);
	trajectory_d_rel(&mainboard.traj, 1000);
	err = WAIT_COND_OR_TRAJ_END(position_get_x_double(&mainboard.pos) > 1500, 0xFF);
	if (err)
		return;
	strat_set_speed(d, a);
	trajectory_d_a_rel(&mainboard.traj, 10000, 1000);
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);
parse_pgm_token_num_t cmd_test_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_test_result, radius, INT32);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0,
		(prog_void *)&cmd_test_arg1,
		NULL,
	},
};
