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
 *  Revision : $Id: commands_mainboard.c,v 1.7 2009-05-02 10:08:09 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <avr/eeprom.h>

#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "actuator.h"
#include "strat_base.h"

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
			pwm_ng_set(LEFT_PWM, 0);
			pwm_ng_set(RIGHT_PWM, 0);
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
/* Interact */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_interact_result {
	fixed_string_t arg0;
};

static void print_cs(void)
{
	printf_P(PSTR("cons_d=% .8ld cons_a=% .8ld fil_d=% .8ld fil_a=% .8ld "
		      "err_d=% .8ld err_a=% .8ld out_d=% .8ld out_a=% .8ld\r\n"), 
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
	printf_P(PSTR("P=% .8ld I=% .8ld D=% .8ld out=% .8ld | "
		      "P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
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
/* 				if (mainboard.flags & DO_CS) */
/* 					strat_hardstop(); */
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
		printf_P(PSTR("angle cons=% .6ld in=% .6ld out=% .6ld / "), 
			 cs_get_consign(&mainboard.angle.cs),
			 cs_get_filtered_feedback(&mainboard.angle.cs),
			 cs_get_out(&mainboard.angle.cs));
		printf_P(PSTR("distance cons=% .6ld in=% .6ld out=% .6ld / "), 
			 cs_get_consign(&mainboard.distance.cs),
			 cs_get_filtered_feedback(&mainboard.distance.cs),
			 cs_get_out(&mainboard.distance.cs));
		printf_P(PSTR("l=% .4ld r=% .4ld\r\n"), mainboard.pwm_l,
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
/* Fessors tests */

/* this structure is filled when cmd_fessor is parsed successfully */
struct cmd_fessor_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_fessor is parsed successfully */
static void cmd_fessor_parsed(void * parsed_result, void * data)
{
	struct cmd_fessor_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("up"))) {
		fessor_up();
	}
	else if (!strcmp_P(res->arg1, PSTR("down"))) {
		fessor_down();
	}
	else if (!strcmp_P(res->arg1, PSTR("stop"))) {
		fessor_stop();
	}
	else if (!strcmp_P(res->arg1, PSTR("auto"))) {
		fessor_auto();
	}

	printf_P(PSTR("done\r\n"));
}

prog_char str_fessor_arg0[] = "fessor";
parse_pgm_token_string_t cmd_fessor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_fessor_result, arg0, str_fessor_arg0);
prog_char str_fessor_arg1[] = "auto#up#down#stop";
parse_pgm_token_string_t cmd_fessor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_fessor_result, arg1, str_fessor_arg1);

prog_char help_fessor[] = "fessor auto mode: fessor auto delay_up delay_down";
parse_pgm_inst_t cmd_fessor = {
	.f = cmd_fessor_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fessor,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fessor_arg0, 
		(prog_void *)&cmd_fessor_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Fessor_Paramss tests */

/* this structure is filled when cmd_fessor_params is parsed successfully */
struct cmd_fessor_params_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_fessor_params is parsed successfully */
static void cmd_fessor_params_parsed(void * parsed_result, void * data)
{
	struct cmd_fessor_params_result * res = parsed_result;
	
	
	if (!strcmp_P(res->arg1, PSTR("delay"))) {
		fessor_set_delays(res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("coef"))) {
		fessor_set_coefs(res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("pos"))) {
		fessor_set_pos(res->arg2, res->arg3);
	}

	/* else show */
	fessor_dump_params();
}

prog_char str_fessor_params_arg0[] = "fessor_params";
parse_pgm_token_string_t cmd_fessor_params_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_fessor_params_result, arg0, str_fessor_params_arg0);
prog_char str_fessor_params_arg1[] = "delay#pos#coef";
parse_pgm_token_string_t cmd_fessor_params_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_fessor_params_result, arg1, str_fessor_params_arg1);
parse_pgm_token_num_t cmd_fessor_params_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_fessor_params_result, arg2, INT32);
parse_pgm_token_num_t cmd_fessor_params_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_fessor_params_result, arg3, INT32);

prog_char help_fessor_params[] = "Set fessor_params values";
parse_pgm_inst_t cmd_fessor_params = {
	.f = cmd_fessor_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fessor_params,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fessor_params_arg0, 
		(prog_void *)&cmd_fessor_params_arg1, 
		(prog_void *)&cmd_fessor_params_arg2, 
		(prog_void *)&cmd_fessor_params_arg3, 
		NULL,
	},
};

prog_char str_fessor_params_arg1_show[] = "show";
parse_pgm_token_string_t cmd_fessor_params_arg1_show = TOKEN_STRING_INITIALIZER(struct cmd_fessor_params_result, arg1, str_fessor_params_arg1_show);

prog_char help_fessor_params_show[] = "show fessor params";
parse_pgm_inst_t cmd_fessor_params_show = {
	.f = cmd_fessor_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fessor_params_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fessor_params_arg0, 
		(prog_void *)&cmd_fessor_params_arg1_show, 
		NULL,
	},
};




/**********************************************************/
/* Elevators tests */

/* this structure is filled when cmd_elevator is parsed successfully */
struct cmd_elevator_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_elevator is parsed successfully */
static void cmd_elevator_parsed(void * parsed_result, void * data)
{
	struct cmd_elevator_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("up"))) {
		elevator_up();
	}
	else if (!strcmp_P(res->arg1, PSTR("down"))) {
		elevator_down();
	}
	else if (!strcmp_P(res->arg1, PSTR("stop"))) {
		elevator_stop();
	}
	else if (!strcmp_P(res->arg1, PSTR("auto"))) {
		elevator_auto();
	}

	printf_P(PSTR("done\r\n"));
}

prog_char str_elevator_arg0[] = "elevator";
parse_pgm_token_string_t cmd_elevator_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_elevator_result, arg0, str_elevator_arg0);
prog_char str_elevator_arg1[] = "auto#up#down#stop";
parse_pgm_token_string_t cmd_elevator_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_elevator_result, arg1, str_elevator_arg1);

prog_char help_elevator[] = "elevator auto mode: elevator auto delay_up delay_down";
parse_pgm_inst_t cmd_elevator = {
	.f = cmd_elevator_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_elevator,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_elevator_arg0, 
		(prog_void *)&cmd_elevator_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Elevator_Paramss tests */

/* this structure is filled when cmd_elevator_params is parsed successfully */
struct cmd_elevator_params_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_elevator_params is parsed successfully */
static void cmd_elevator_params_parsed(void * parsed_result, void * data)
{
	struct cmd_elevator_params_result * res = parsed_result;
	
	
	if (!strcmp_P(res->arg1, PSTR("delay"))) {
		elevator_set_delays(res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("coef"))) {
		elevator_set_coefs(res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("pos"))) {
		elevator_set_pos(res->arg2, res->arg3);
	}

	/* else show */
	elevator_dump_params();
}

prog_char str_elevator_params_arg0[] = "elevator_params";
parse_pgm_token_string_t cmd_elevator_params_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_elevator_params_result, arg0, str_elevator_params_arg0);
prog_char str_elevator_params_arg1[] = "delay#pos#coef";
parse_pgm_token_string_t cmd_elevator_params_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_elevator_params_result, arg1, str_elevator_params_arg1);
parse_pgm_token_num_t cmd_elevator_params_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_elevator_params_result, arg2, INT32);
parse_pgm_token_num_t cmd_elevator_params_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_elevator_params_result, arg3, INT32);

prog_char help_elevator_params[] = "Set elevator_params values";
parse_pgm_inst_t cmd_elevator_params = {
	.f = cmd_elevator_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_elevator_params,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_elevator_params_arg0, 
		(prog_void *)&cmd_elevator_params_arg1, 
		(prog_void *)&cmd_elevator_params_arg2, 
		(prog_void *)&cmd_elevator_params_arg3, 
		NULL,
	},
};

prog_char str_elevator_params_arg1_show[] = "show";
parse_pgm_token_string_t cmd_elevator_params_arg1_show = TOKEN_STRING_INITIALIZER(struct cmd_elevator_params_result, arg1, str_elevator_params_arg1_show);

prog_char help_elevator_params_show[] = "show elevator params";
parse_pgm_inst_t cmd_elevator_params_show = {
	.f = cmd_elevator_params_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_elevator_params_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_elevator_params_arg0, 
		(prog_void *)&cmd_elevator_params_arg1_show, 
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
