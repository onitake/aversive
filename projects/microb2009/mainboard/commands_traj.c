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
 *  Revision : $Id: commands_traj.c,v 1.8 2009-11-08 17:24:33 zer0 Exp $
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
#include <time.h>
#include <spi.h>
#include <encoders_spi.h>

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
#include "cs.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat.h"
#include "../common/i2c_commands.h"
#include "i2c_protocol.h"

/**********************************************************/
/* Traj_Speeds for trajectory_manager */

/* this structure is filled when cmd_traj_speed is parsed successfully */
struct cmd_traj_speed_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t s;
};

/* function called when cmd_traj_speed is parsed successfully */
static void cmd_traj_speed_parsed(void *parsed_result, void *data)
{
	struct cmd_traj_speed_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("angle"))) {
		trajectory_set_speed(&mainboard.traj, mainboard.traj.d_speed, res->s);
	}
	else if (!strcmp_P(res->arg1, PSTR("distance"))) {
		trajectory_set_speed(&mainboard.traj, res->s, mainboard.traj.a_speed);
	}
	/* else it is a "show" */

	printf_P(PSTR("angle %u, distance %u\r\n"), 
		 mainboard.traj.a_speed,
		 mainboard.traj.d_speed);
}

prog_char str_traj_speed_arg0[] = "traj_speed";
parse_pgm_token_string_t cmd_traj_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg0, str_traj_speed_arg0);
prog_char str_traj_speed_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_speed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_arg1);
parse_pgm_token_num_t cmd_traj_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_speed_result, s, UINT16);

prog_char help_traj_speed[] = "Set traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed = {
	.f = cmd_traj_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_traj_speed,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_traj_speed_arg0, 
		(prog_void *)&cmd_traj_speed_arg1, 
		(prog_void *)&cmd_traj_speed_s, 
		NULL,
	},
};

/* show */

prog_char str_traj_speed_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_speed_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_show_arg);

prog_char help_traj_speed_show[] = "Show traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed_show = {
	.f = cmd_traj_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_traj_speed_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_traj_speed_arg0, 
		(prog_void *)&cmd_traj_speed_show_arg,
		NULL,
	},
};

/**********************************************************/
/* circle coef configuration */

/* this structure is filled when cmd_circle_coef is parsed successfully */
struct cmd_circle_coef_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float circle_coef;
};


/* function called when cmd_circle_coef is parsed successfully */
static void cmd_circle_coef_parsed(void *parsed_result, void *data)
{
	struct cmd_circle_coef_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		trajectory_set_circle_coef(&mainboard.traj, res->circle_coef);
	}

	printf_P(PSTR("circle_coef set %2.2f\r\n"), mainboard.traj.circle_coef);
}

prog_char str_circle_coef_arg0[] = "circle_coef";
parse_pgm_token_string_t cmd_circle_coef_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg0, str_circle_coef_arg0);
prog_char str_circle_coef_arg1[] = "set";
parse_pgm_token_string_t cmd_circle_coef_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_arg1);
parse_pgm_token_num_t cmd_circle_coef_val = TOKEN_NUM_INITIALIZER(struct cmd_circle_coef_result, circle_coef, FLOAT);

prog_char help_circle_coef[] = "Set circle coef";
parse_pgm_inst_t cmd_circle_coef = {
	.f = cmd_circle_coef_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_circle_coef,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_circle_coef_arg0,
		(prog_void *)&cmd_circle_coef_arg1,
		(prog_void *)&cmd_circle_coef_val,
		NULL,
	},
};

/* show */

prog_char str_circle_coef_show_arg[] = "show";
parse_pgm_token_string_t cmd_circle_coef_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_show_arg);

prog_char help_circle_coef_show[] = "Show circle coef";
parse_pgm_inst_t cmd_circle_coef_show = {
	.f = cmd_circle_coef_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_circle_coef_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_circle_coef_arg0, 
		(prog_void *)&cmd_circle_coef_show_arg,
		NULL,
	},
};

/**********************************************************/
/* trajectory window configuration */

/* this structure is filled when cmd_trajectory is parsed successfully */
struct cmd_trajectory_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float d_win;
	float a_win;
	float a_start;
};


/* function called when cmd_trajectory is parsed successfully */
static void cmd_trajectory_parsed(void * parsed_result, void * data)
{
	struct cmd_trajectory_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("set"))) {
		trajectory_set_windows(&mainboard.traj, res->d_win,
				       res->a_win, res->a_start);
	}

	printf_P(PSTR("trajectory %2.2f %2.2f %2.2f\r\n"), mainboard.traj.d_win,
		 DEG(mainboard.traj.a_win_rad), DEG(mainboard.traj.a_start_rad));
}

prog_char str_trajectory_arg0[] = "trajectory";
parse_pgm_token_string_t cmd_trajectory_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg0, str_trajectory_arg0);
prog_char str_trajectory_arg1[] = "set";
parse_pgm_token_string_t cmd_trajectory_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_arg1);
parse_pgm_token_num_t cmd_trajectory_d = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, d_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_a = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_as = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_start, FLOAT);

prog_char help_trajectory[] = "Set trajectory windows (distance, angle, angle_start)";
parse_pgm_inst_t cmd_trajectory = {
	.f = cmd_trajectory_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_trajectory,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_trajectory_arg0, 
		(prog_void *)&cmd_trajectory_arg1, 
		(prog_void *)&cmd_trajectory_d, 
		(prog_void *)&cmd_trajectory_a, 
		(prog_void *)&cmd_trajectory_as, 
		NULL,
	},
};

/* show */

prog_char str_trajectory_show_arg[] = "show";
parse_pgm_token_string_t cmd_trajectory_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_show_arg);

prog_char help_trajectory_show[] = "Show trajectory window configuration";
parse_pgm_inst_t cmd_trajectory_show = {
	.f = cmd_trajectory_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_trajectory_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_trajectory_arg0, 
		(prog_void *)&cmd_trajectory_show_arg,
		NULL,
	},
};

/**********************************************************/
/* rs_gains configuration */

/* this structure is filled when cmd_rs_gains is parsed successfully */
struct cmd_rs_gains_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float left;
	float right;
};

/* function called when cmd_rs_gains is parsed successfully */
static void cmd_rs_gains_parsed(void * parsed_result, void * data)
{
	struct cmd_rs_gains_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		rs_set_left_ext_encoder(&mainboard.rs, encoders_spi_get_value, 
					LEFT_ENCODER, res->left); // en augmentant on tourne à gauche
		rs_set_right_ext_encoder(&mainboard.rs, encoders_spi_get_value, 
					 RIGHT_ENCODER, res->right); //en augmentant on tourne à droite
	}
	printf_P(PSTR("rs_gains set "));
	//f64_print(mainboard.rs.left_ext_gain);
	printf_P(PSTR(" "));
	//f64_print(mainboard.rs.right_ext_gain);
	printf_P(PSTR("\r\n"));
}

prog_char str_rs_gains_arg0[] = "rs_gains";
parse_pgm_token_string_t cmd_rs_gains_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg0, str_rs_gains_arg0);
prog_char str_rs_gains_arg1[] = "set";
parse_pgm_token_string_t cmd_rs_gains_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_arg1);
parse_pgm_token_num_t cmd_rs_gains_l = TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, left, FLOAT);
parse_pgm_token_num_t cmd_rs_gains_r = TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, right, FLOAT);

prog_char help_rs_gains[] = "Set rs_gains (left, right)";
parse_pgm_inst_t cmd_rs_gains = {
	.f = cmd_rs_gains_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_rs_gains,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_rs_gains_arg0, 
		(prog_void *)&cmd_rs_gains_arg1, 
		(prog_void *)&cmd_rs_gains_l, 
		(prog_void *)&cmd_rs_gains_r, 
		NULL,
	},
};

/* show */

prog_char str_rs_gains_show_arg[] = "show";
parse_pgm_token_string_t cmd_rs_gains_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_show_arg);

prog_char help_rs_gains_show[] = "Show rs_gains";
parse_pgm_inst_t cmd_rs_gains_show = {
	.f = cmd_rs_gains_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_rs_gains_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_rs_gains_arg0, 
		(prog_void *)&cmd_rs_gains_show_arg,
		NULL,
	},
};

/**********************************************************/
/* track configuration */

/* this structure is filled when cmd_track is parsed successfully */
struct cmd_track_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float val;
};

/* function called when cmd_track is parsed successfully */
static void cmd_track_parsed(void * parsed_result, void * data)
{
	struct cmd_track_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		position_set_physical_params(&mainboard.pos, res->val, DIST_IMP_MM);
	}
	printf_P(PSTR("track set %f\r\n"), mainboard.pos.phys.track_mm);
}

prog_char str_track_arg0[] = "track";
parse_pgm_token_string_t cmd_track_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg0, str_track_arg0);
prog_char str_track_arg1[] = "set";
parse_pgm_token_string_t cmd_track_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_arg1);
parse_pgm_token_num_t cmd_track_val = TOKEN_NUM_INITIALIZER(struct cmd_track_result, val, FLOAT);

prog_char help_track[] = "Set track in mm";
parse_pgm_inst_t cmd_track = {
	.f = cmd_track_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_track,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_track_arg0, 
		(prog_void *)&cmd_track_arg1, 
		(prog_void *)&cmd_track_val, 
		NULL,
	},
};

/* show */

prog_char str_track_show_arg[] = "show";
parse_pgm_token_string_t cmd_track_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_show_arg);

prog_char help_track_show[] = "Show track";
parse_pgm_inst_t cmd_track_show = {
	.f = cmd_track_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_track_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_track_arg0, 
		(prog_void *)&cmd_track_show_arg,
		NULL,
	},
};



/**********************************************************/
/* Pt_Lists for testing traj */

#define PT_LIST_SIZE 10
static struct xy_point pt_list[PT_LIST_SIZE];
static uint16_t pt_list_len = 0;

/* this structure is filled when cmd_pt_list is parsed successfully */
struct cmd_pt_list_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t arg2;
	int16_t arg3;
	int16_t arg4;
};

/* function called when cmd_pt_list is parsed successfully */
static void cmd_pt_list_parsed(void * parsed_result, void * data)
{
	struct cmd_pt_list_result * res = parsed_result;
	uint8_t i, why=0;
	
	if (!strcmp_P(res->arg1, PSTR("append"))) {
		res->arg2 = pt_list_len;
	}

	if (!strcmp_P(res->arg1, PSTR("insert")) ||
	    !strcmp_P(res->arg1, PSTR("append"))) {
		if (res->arg2 > pt_list_len) {
			printf_P(PSTR("Index too large\r\n"));
			return;
		}
		if (pt_list_len >= PT_LIST_SIZE) {
			printf_P(PSTR("List is too large\r\n"));
			return;
		}
		memmove(&pt_list[res->arg2+1], &pt_list[res->arg2], 
		       PT_LIST_SIZE-1-res->arg2);
		pt_list[res->arg2].x = res->arg3;
		pt_list[res->arg2].y = res->arg4;
		pt_list_len++;
	}
	else if (!strcmp_P(res->arg1, PSTR("del"))) {
		if (pt_list_len <= 0) {
			printf_P(PSTR("Error: list empty\r\n"));
			return;
		}
		if (res->arg2 > pt_list_len) {
			printf_P(PSTR("Index too large\r\n"));
			return;
		}
		memmove(&pt_list[res->arg2], &pt_list[res->arg2+1], 
		       (PT_LIST_SIZE-1-res->arg2)*sizeof(struct xy_point));
		pt_list_len--;
	}
	else if (!strcmp_P(res->arg1, PSTR("reset"))) {
		pt_list_len = 0;
	}
	
	/* else it is a "show" or a "start" */
	if (pt_list_len == 0) {
		printf_P(PSTR("List empty\r\n"));
		return;
	}
	for (i=0 ; i<pt_list_len ; i++) {
		printf_P(PSTR("%d: x=%d y=%d\r\n"), i, pt_list[i].x, pt_list[i].y);
		if (!strcmp_P(res->arg1, PSTR("start"))) {
			trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x, pt_list[i].y);
			why = wait_traj_end(0xFF); /* all */
		}
		else if (!strcmp_P(res->arg1, PSTR("avoid_start"))) {
			while (1) {
				why = goto_and_avoid(pt_list[i].x, pt_list[i].y, 0xFF, 0xFF);
				printf("next point\r\n");
				if (why != END_OBSTACLE)
					break;
			}
		}
		if (why & (~(END_TRAJ | END_NEAR)))
			trajectory_stop(&mainboard.traj);
	}
}

prog_char str_pt_list_arg0[] = "pt_list";
parse_pgm_token_string_t cmd_pt_list_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg0, str_pt_list_arg0);
prog_char str_pt_list_arg1[] = "insert";
parse_pgm_token_string_t cmd_pt_list_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1);
parse_pgm_token_num_t cmd_pt_list_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg2, UINT16);
parse_pgm_token_num_t cmd_pt_list_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg3, INT16);
parse_pgm_token_num_t cmd_pt_list_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg4, INT16);

prog_char help_pt_list[] = "Insert point in pt_list (idx,x,y)";
parse_pgm_inst_t cmd_pt_list = {
	.f = cmd_pt_list_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pt_list,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pt_list_arg0, 
		(prog_void *)&cmd_pt_list_arg1, 
		(prog_void *)&cmd_pt_list_arg2, 
		(prog_void *)&cmd_pt_list_arg3, 
		(prog_void *)&cmd_pt_list_arg4, 
		NULL,
	},
};

/* append */

prog_char str_pt_list_arg1_append[] = "append";
parse_pgm_token_string_t cmd_pt_list_arg1_append = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1_append);

prog_char help_pt_list_append[] = "Append point in pt_list (x,y)";
parse_pgm_inst_t cmd_pt_list_append = {
	.f = cmd_pt_list_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pt_list_append,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pt_list_arg0, 
		(prog_void *)&cmd_pt_list_arg1_append, 
		(prog_void *)&cmd_pt_list_arg3, 
		(prog_void *)&cmd_pt_list_arg4, 
		NULL,
	},
};

/* del */

prog_char str_pt_list_del_arg[] = "del";
parse_pgm_token_string_t cmd_pt_list_del_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_del_arg);

prog_char help_pt_list_del[] = "Del or insert point in pt_list (num)";
parse_pgm_inst_t cmd_pt_list_del = {
	.f = cmd_pt_list_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pt_list_del,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pt_list_arg0, 
		(prog_void *)&cmd_pt_list_del_arg, 
		(prog_void *)&cmd_pt_list_arg2,
		NULL,
	},
};
/* show */

prog_char str_pt_list_show_arg[] = "show#reset#start#avoid_start";
parse_pgm_token_string_t cmd_pt_list_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_show_arg);

prog_char help_pt_list_show[] = "Show, start or reset pt_list";
parse_pgm_inst_t cmd_pt_list_show = {
	.f = cmd_pt_list_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pt_list_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pt_list_arg0, 
		(prog_void *)&cmd_pt_list_show_arg,
		NULL,
	},
};



/**********************************************************/
/* Goto function */

/* this structure is filled when cmd_goto is parsed successfully */
struct cmd_goto_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
	int32_t arg5;
};

/* function called when cmd_goto is parsed successfully */
static void cmd_goto_parsed(void * parsed_result, void * data)
{
	struct cmd_goto_result * res = parsed_result;
	uint8_t err;
	microseconds t1, t2;

	interrupt_traj_reset();
	if (!strcmp_P(res->arg1, PSTR("a_rel"))) {
		trajectory_a_rel(&mainboard.traj, res->arg2);
	}
	else if (!strcmp_P(res->arg1, PSTR("d_rel"))) {
		trajectory_d_rel(&mainboard.traj, res->arg2);
	}
	else if (!strcmp_P(res->arg1, PSTR("a_abs"))) {
		trajectory_a_abs(&mainboard.traj, res->arg2);
	}
	else if (!strcmp_P(res->arg1, PSTR("a_to_xy"))) {
		trajectory_turnto_xy(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("a_behind_xy"))) {
		trajectory_turnto_xy_behind(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("xy_rel"))) {
		trajectory_goto_xy_rel(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("xy_abs"))) {
		trajectory_goto_xy_abs(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("avoid"))) {
		err = goto_and_avoid_forward(res->arg2, res->arg3, 0xFF, 0xFF);
		if (err != END_TRAJ && err != END_NEAR)
			strat_hardstop();
	}
	else if (!strcmp_P(res->arg1, PSTR("avoid_bw"))) {
		err = goto_and_avoid_backward(res->arg2, res->arg3, 0xFF, 0xFF);
		if (err != END_TRAJ && err != END_NEAR)
			strat_hardstop();
	}
	else if (!strcmp_P(res->arg1, PSTR("xy_abs_fow"))) {
		trajectory_goto_forward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("xy_abs_back"))) {
		trajectory_goto_backward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("da_rel"))) {
		trajectory_d_a_rel(&mainboard.traj, res->arg2, res->arg3);
	}
	else if (!strcmp_P(res->arg1, PSTR("circle_rel"))) {
		trajectory_circle_rel(&mainboard.traj, res->arg2, res->arg3,
				      res->arg4, res->arg5, 0);
		return; /* XXX */
	}
	t1 = time_get_us2();
	while ((err = test_traj_end(0xFF)) == 0) {
		t2 = time_get_us2();
		if (t2 - t1 > 200000) {
			dump_cs_debug("angle", &mainboard.angle.cs);
			dump_cs_debug("distance", &mainboard.distance.cs);
			t1 = t2;
		}
	}
	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();
	printf_P(PSTR("returned %s\r\n"), get_err(err));
}

prog_char str_goto_arg0[] = "goto";
parse_pgm_token_string_t cmd_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg0, str_goto_arg0);
prog_char str_goto_arg1_a[] = "d_rel#a_rel#a_abs";
parse_pgm_token_string_t cmd_goto_arg1_a = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_a);
parse_pgm_token_num_t cmd_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg2, INT32);

/* 1 params */
prog_char help_goto1[] = "Change orientation of the mainboard";
parse_pgm_inst_t cmd_goto1 = {
	.f = cmd_goto_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_goto1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_goto_arg0, 
		(prog_void *)&cmd_goto_arg1_a, 
		(prog_void *)&cmd_goto_arg2, 
		NULL,
	},
};

prog_char str_goto_arg1_b[] = "xy_rel#xy_abs#xy_abs_fow#xy_abs_back#da_rel#a_to_xy#avoid#avoid_bw#a_behind_xy";
parse_pgm_token_string_t cmd_goto_arg1_b = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_b);
parse_pgm_token_num_t cmd_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg3, INT32);

/* 2 params */
prog_char help_goto2[] = "Go to a (x,y) or (d,a) position";
parse_pgm_inst_t cmd_goto2 = {
	.f = cmd_goto_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_goto2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_goto_arg0, 
		(prog_void *)&cmd_goto_arg1_b, 
		(prog_void *)&cmd_goto_arg2,
		(prog_void *)&cmd_goto_arg3, 
		NULL,
	},
};

prog_char str_goto_arg1_c[] = "circle_rel";
parse_pgm_token_string_t cmd_goto_arg1_c = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_c);
parse_pgm_token_num_t cmd_goto_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg4, INT32);
parse_pgm_token_num_t cmd_goto_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg5, INT32);

/* 4 params */
prog_char help_goto4[] = "Do a circle: (x,y, radius, angle)";
parse_pgm_inst_t cmd_goto4 = {
	.f = cmd_goto_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_goto4,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_goto_arg0,
		(prog_void *)&cmd_goto_arg1_c,
		(prog_void *)&cmd_goto_arg2,
		(prog_void *)&cmd_goto_arg3,
		(prog_void *)&cmd_goto_arg4,
		(prog_void *)&cmd_goto_arg5,
		NULL,
	},
};

/**********************************************************/
/* Position tests */

/* this structure is filled when cmd_position is parsed successfully */
struct cmd_position_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
};

#define AUTOPOS_SPEED_FAST 200
static void auto_position(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;

	interrupt_traj_reset();
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	strat_reset_pos(ROBOT_LENGTH/2, 0, 0);

	trajectory_d_rel(&mainboard.traj, 120);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	trajectory_a_rel(&mainboard.traj, COLOR_A(90));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	strat_reset_pos(DO_NOT_SET_POS, COLOR_Y(ROBOT_LENGTH/2),
			COLOR_A(90));

	trajectory_d_rel(&mainboard.traj, 120);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	
	trajectory_a_rel(&mainboard.traj, COLOR_A(-40));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	
	strat_set_speed(old_spdd, old_spda);
	return;

intr:
	strat_hardstop();
	strat_set_speed(old_spdd, old_spda);
}

/* function called when cmd_position is parsed successfully */
static void cmd_position_parsed(void * parsed_result, void * data)
{
	struct cmd_position_result * res = parsed_result;
	
	/* display raw position values */
	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		position_set(&mainboard.pos, 0, 0, 0);
	}
	else if (!strcmp_P(res->arg1, PSTR("set"))) {
		position_set(&mainboard.pos, res->arg2, res->arg3, res->arg4);
	}
	else if (!strcmp_P(res->arg1, PSTR("autoset_green"))) {
		mainboard.our_color = I2C_COLOR_GREEN;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_GREEN);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_GREEN);
		auto_position();
	}
	else if (!strcmp_P(res->arg1, PSTR("autoset_red"))) {
		mainboard.our_color = I2C_COLOR_RED;
		i2c_set_color(I2C_MECHBOARD_ADDR, I2C_COLOR_RED);
		i2c_set_color(I2C_SENSORBOARD_ADDR, I2C_COLOR_RED);
		auto_position();
	}

	/* else it's just a "show" */
	printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"), 
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));
}

prog_char str_position_arg0[] = "position";
parse_pgm_token_string_t cmd_position_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg0, str_position_arg0);
prog_char str_position_arg1[] = "show#reset#autoset_green#autoset_red";
parse_pgm_token_string_t cmd_position_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1);

prog_char help_position[] = "Show/reset (x,y,a) position";
parse_pgm_inst_t cmd_position = {
	.f = cmd_position_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_position,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_position_arg0, 
		(prog_void *)&cmd_position_arg1, 
		NULL,
	},
};


prog_char str_position_arg1_set[] = "set";
parse_pgm_token_string_t cmd_position_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1_set);
parse_pgm_token_num_t cmd_position_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg2, INT32);
parse_pgm_token_num_t cmd_position_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg3, INT32);
parse_pgm_token_num_t cmd_position_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg4, INT32);

prog_char help_position_set[] = "Set (x,y,a) position";
parse_pgm_inst_t cmd_position_set = {
	.f = cmd_position_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_position_set,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_position_arg0, 
		(prog_void *)&cmd_position_arg1_set, 
		(prog_void *)&cmd_position_arg2, 
		(prog_void *)&cmd_position_arg3, 
		(prog_void *)&cmd_position_arg4, 
		NULL,
	},
};


/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_infos is parsed successfully */
struct cmd_strat_infos_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_strat_infos is parsed successfully */
static void cmd_strat_infos_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_infos_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		strat_reset_infos();
	}
	strat_infos.dump_enabled = 1;
	strat_dump_infos(__FUNCTION__);
}

prog_char str_strat_infos_arg0[] = "strat_infos";
parse_pgm_token_string_t cmd_strat_infos_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg0, str_strat_infos_arg0);
prog_char str_strat_infos_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_strat_infos_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg1, str_strat_infos_arg1);

prog_char help_strat_infos[] = "reset/show strat_infos";
parse_pgm_inst_t cmd_strat_infos = {
	.f = cmd_strat_infos_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_infos,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_infos_arg0, 
		(prog_void *)&cmd_strat_infos_arg1, 
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf is parsed successfully */
struct cmd_strat_conf_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_strat_conf is parsed successfully */
static void cmd_strat_conf_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("base"))) {
		strat_infos.conf.flags = 0;
		strat_infos.conf.scan_our_min_time = 90;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 90;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	else if (!strcmp_P(res->arg1, PSTR("big3"))) {
		strat_infos.conf.flags = 
			STRAT_CONF_STORE_STATIC2 |
			STRAT_CONF_BIG_3_TEMPLE;
		strat_infos.conf.scan_our_min_time = 90;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 90;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	else if (!strcmp_P(res->arg1, PSTR("base_check"))) {
		strat_infos.conf.flags = 0;
		strat_infos.conf.scan_our_min_time = 35;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 90;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	else if (!strcmp_P(res->arg1, PSTR("big3_check"))) {
		strat_infos.conf.flags = 
			STRAT_CONF_STORE_STATIC2 |
			STRAT_CONF_BIG_3_TEMPLE;
		strat_infos.conf.scan_our_min_time = 35;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 90;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	else if (!strcmp_P(res->arg1, PSTR("offensive_early"))) {
		strat_infos.conf.flags = 
			STRAT_CONF_TAKE_ONE_LINTEL |
			STRAT_CONF_STORE_STATIC2 |
			STRAT_CONF_EARLY_SCAN |
			STRAT_CONF_PUSH_OPP_COLS;
		strat_infos.conf.scan_our_min_time = 50;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 15;
		strat_infos.conf.delay_between_opp_scan = 90;
		strat_infos.conf.wait_opponent = 5;
	}
	else if (!strcmp_P(res->arg1, PSTR("offensive_late"))) {
		strat_infos.conf.flags = STRAT_CONF_TAKE_ONE_LINTEL;
		strat_infos.conf.scan_our_min_time = 90;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 30;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	else if (!strcmp_P(res->arg1, PSTR("one_on_disc"))) {
		strat_infos.conf.flags = 
			STRAT_CONF_ONLY_ONE_ON_DISC;
		strat_infos.conf.scan_our_min_time = 90;
		strat_infos.conf.delay_between_our_scan = 90;
		strat_infos.conf.scan_opp_min_time = 90;
		strat_infos.conf.delay_between_opp_scan = 90;
	}
	strat_infos.dump_enabled = 1;
	strat_dump_conf();
}

prog_char str_strat_conf_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg0, str_strat_conf_arg0);
prog_char str_strat_conf_arg1[] = "show#base#big3#base_check#big3_check#offensive_early#offensive_late#one_on_disc";
parse_pgm_token_string_t cmd_strat_conf_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg1, str_strat_conf_arg1);

prog_char help_strat_conf[] = "configure strat options";
parse_pgm_inst_t cmd_strat_conf = {
	.f = cmd_strat_conf_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_conf,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_conf_arg0, 
		(prog_void *)&cmd_strat_conf_arg1, 
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf2 is parsed successfully */
struct cmd_strat_conf2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_strat_conf2 is parsed successfully */
static void cmd_strat_conf2_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf2_result *res = parsed_result;
	uint8_t on, bit = 0;

	if (!strcmp_P(res->arg2, PSTR("on")))
		on = 1;
	else
		on = 0;
	
	if (!strcmp_P(res->arg1, PSTR("one_temple_on_disc")))
		bit = STRAT_CONF_ONLY_ONE_ON_DISC;
	else if (!strcmp_P(res->arg1, PSTR("bypass_static2")))
		bit = STRAT_CONF_BYPASS_STATIC2;
	else if (!strcmp_P(res->arg1, PSTR("take_one_lintel")))
		bit = STRAT_CONF_TAKE_ONE_LINTEL;
	else if (!strcmp_P(res->arg1, PSTR("skip_when_check_fail")))
		bit = STRAT_CONF_TAKE_ONE_LINTEL;
	else if (!strcmp_P(res->arg1, PSTR("store_static2")))
		bit = STRAT_CONF_STORE_STATIC2;
	else if (!strcmp_P(res->arg1, PSTR("big3_temple")))
		bit = STRAT_CONF_BIG_3_TEMPLE;
	else if (!strcmp_P(res->arg1, PSTR("early_opp_scan")))
		bit = STRAT_CONF_EARLY_SCAN;
	else if (!strcmp_P(res->arg1, PSTR("push_opp_cols")))
		bit = STRAT_CONF_PUSH_OPP_COLS;

	if (on)
		strat_infos.conf.flags |= bit;
	else
		strat_infos.conf.flags &= (~bit);

	strat_infos.dump_enabled = 1;
	strat_dump_conf();
}

prog_char str_strat_conf2_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg0, str_strat_conf2_arg0);
prog_char str_strat_conf2_arg1[] = "push_opp_cols#one_temple_on_disc#bypass_static2#take_one_lintel#skip_when_check_fail#store_static2#big3_temple#early_opp_scan";
parse_pgm_token_string_t cmd_strat_conf2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg1, str_strat_conf2_arg1);
prog_char str_strat_conf2_arg2[] = "on#off";
parse_pgm_token_string_t cmd_strat_conf2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg2, str_strat_conf2_arg2);


prog_char help_strat_conf2[] = "configure strat options";
parse_pgm_inst_t cmd_strat_conf2 = {
	.f = cmd_strat_conf2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_conf2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_conf2_arg0, 
		(prog_void *)&cmd_strat_conf2_arg1, 
		(prog_void *)&cmd_strat_conf2_arg2, 
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf3 is parsed successfully */
struct cmd_strat_conf3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
};

/* function called when cmd_strat_conf3 is parsed successfully */
static void cmd_strat_conf3_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf3_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("scan_opponent_min_time"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_infos.conf.scan_opp_min_time = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("delay_between_opponent_scan"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_infos.conf.delay_between_opp_scan = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("scan_our_min_time"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_infos.conf.scan_our_min_time = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("delay_between_our_scan"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_infos.conf.delay_between_our_scan = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("wait_opponent"))) {
		strat_infos.conf.wait_opponent = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("lintel_min_time"))) {
		strat_infos.conf.lintel_min_time = res->arg2;
	}
	strat_infos.dump_enabled = 1;
	strat_dump_conf();
}

prog_char str_strat_conf3_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg0, str_strat_conf3_arg0);
prog_char str_strat_conf3_arg1[] = "lintel_min_time#scan_opponent_min_time#delay_between_opponent_scan#scan_our_min_time#delay_between_our_scan#wait_opponent";
parse_pgm_token_string_t cmd_strat_conf3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg1, str_strat_conf3_arg1);
parse_pgm_token_num_t cmd_strat_conf3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_strat_conf3_result, arg2, UINT16);

prog_char help_strat_conf3[] = "configure strat options";
parse_pgm_inst_t cmd_strat_conf3 = {
	.f = cmd_strat_conf3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_conf3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_conf3_arg0, 
		(prog_void *)&cmd_strat_conf3_arg1, 
		(prog_void *)&cmd_strat_conf3_arg2, 
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf4 is parsed successfully */
struct cmd_strat_conf4_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

/* function called when cmd_strat_conf4 is parsed successfully */
static void cmd_strat_conf4_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf4_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("scan_opponent_angle"))) {
		strat_infos.conf.scan_opp_angle = res->arg2;
	}
	strat_infos.dump_enabled = 1;
	strat_dump_conf();
}

prog_char str_strat_conf4_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf4_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf4_result, arg0, str_strat_conf4_arg0);
prog_char str_strat_conf4_arg1[] = "scan_opponent_angle";
parse_pgm_token_string_t cmd_strat_conf4_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf4_result, arg1, str_strat_conf4_arg1);
parse_pgm_token_num_t cmd_strat_conf4_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_strat_conf4_result, arg2, UINT16);

prog_char help_strat_conf4[] = "configure strat options";
parse_pgm_inst_t cmd_strat_conf4 = {
	.f = cmd_strat_conf4_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_conf4,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_conf4_arg0, 
		(prog_void *)&cmd_strat_conf4_arg1, 
		(prog_void *)&cmd_strat_conf4_arg2, 
		NULL,
	},
};


/**********************************************************/
/* Subtraj */

/* this structure is filled when cmd_subtraj is parsed successfully */
struct cmd_subtraj_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
	int32_t arg5;
};

/* function called when cmd_subtraj is parsed successfully */
static void cmd_subtraj_parsed(void *parsed_result, void *data)
{
	struct cmd_subtraj_result *res = parsed_result;
	uint8_t err = 0;
	struct column_dispenser *disp;

	if (strcmp_P(res->arg1, PSTR("static")) == 0) {
		err = strat_static_columns(res->arg2);
	}
	else if (strcmp_P(res->arg1, PSTR("static2")) == 0) {
		strat_infos.s_cols.configuration = res->arg2;
		switch (res->arg2) {
		case 1:
			position_set(&mainboard.pos, 1398, 
				     COLOR_Y(1297), COLOR_A(-66));
			break;
		case 2:
			position_set(&mainboard.pos, 1232, 
				     COLOR_Y(1051), COLOR_A(4));
			break;
		case 3:
			position_set(&mainboard.pos, 1232, 
				     COLOR_Y(1043), COLOR_A(5));
			break;
		case 4:
			position_set(&mainboard.pos, 1346,
				     COLOR_Y(852), COLOR_A(57));
			break;
		default:
			return;
		}
		if (res->arg2 == 1 && res->arg3 == 1) {
			strat_infos.s_cols.flags = STATIC_COL_LINE1_DONE;
		}
		if (res->arg2 == 1 && res->arg3 == 2) {
			strat_infos.s_cols.flags = STATIC_COL_LINE2_DONE;
		}
		err = strat_static_columns_pass2();
	}
	else if (strcmp_P(res->arg1, PSTR("lintel1")) == 0) {
		err = strat_goto_lintel_disp(&strat_infos.l1);
	}
	else if (strcmp_P(res->arg1, PSTR("lintel2")) == 0) {
		err = strat_goto_lintel_disp(&strat_infos.l2);
	}
	else if (strcmp_P(res->arg1, PSTR("coldisp1")) == 0) {
		disp = &strat_infos.c1;
		err = strat_goto_col_disp(&disp);
	}
	else if (strcmp_P(res->arg1, PSTR("coldisp2")) == 0) {
		disp = &strat_infos.c2;
		err = strat_goto_col_disp(&disp);
	}
	else if (strcmp_P(res->arg1, PSTR("coldisp3")) == 0) {
		disp = &strat_infos.c3;
		err = strat_goto_col_disp(&disp);
	}
	else if (strcmp_P(res->arg1, PSTR("disc")) == 0) {
		if (res->arg2 == 0) {
			printf_P(PSTR("bad level\r\n"));
			return;
		}
		err = strat_goto_disc(res->arg2);
	}

	printf_P(PSTR("substrat returned %s\r\n"), get_err(err));
	trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj_result, arg0, str_subtraj_arg0);
prog_char str_subtraj_arg1[] = "static#disc#lintel1#lintel2#coldisp1#coldisp2#coldisp3#static2";
parse_pgm_token_string_t cmd_subtraj_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj_result, arg1, str_subtraj_arg1);
parse_pgm_token_num_t cmd_subtraj_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj_result, arg2, INT32);
parse_pgm_token_num_t cmd_subtraj_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj_result, arg3, INT32);
parse_pgm_token_num_t cmd_subtraj_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj_result, arg4, INT32);
parse_pgm_token_num_t cmd_subtraj_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj_result, arg5, INT32);

prog_char help_subtraj[] = "Test sub-trajectories (a,b,c,d: specific params)";
parse_pgm_inst_t cmd_subtraj = {
	.f = cmd_subtraj_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_subtraj,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_subtraj_arg0, 
		(prog_void *)&cmd_subtraj_arg1, 
		(prog_void *)&cmd_subtraj_arg2, 
		(prog_void *)&cmd_subtraj_arg3, 
		(prog_void *)&cmd_subtraj_arg4, 
		(prog_void *)&cmd_subtraj_arg5, 
		NULL,
	},
};
