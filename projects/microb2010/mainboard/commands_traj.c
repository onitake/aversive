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

#include <hostsim.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>
#include <encoders_spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
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

#include "main.h"
#include "cs.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat.h"
#include "strat_db.h"
#include "../common/i2c_commands.h"
#include "i2c_protocol.h"

/**********************************************************/
/* Traj_Speeds for trajectory_manager */

/* this structure is filled when cmd_traj_speed is parsed successfully */
struct cmd_traj_speed_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float s;
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

	printf_P(PSTR("angle %2.2f, distance %2.2f\r\n"),
		 mainboard.traj.a_speed,
		 mainboard.traj.d_speed);
}

prog_char str_traj_speed_arg0[] = "traj_speed";
parse_pgm_token_string_t cmd_traj_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg0, str_traj_speed_arg0);
prog_char str_traj_speed_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_speed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_arg1);
parse_pgm_token_num_t cmd_traj_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_speed_result, s, FLOAT);

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
/* Traj_Accs for trajectory_manager */

/* this structure is filled when cmd_traj_acc is parsed successfully */
struct cmd_traj_acc_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float s;
};

/* function called when cmd_traj_acc is parsed successfully */
static void cmd_traj_acc_parsed(void *parsed_result, void *data)
{
	struct cmd_traj_acc_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("angle"))) {
		trajectory_set_acc(&mainboard.traj, mainboard.traj.d_acc, res->s);
	}
	else if (!strcmp_P(res->arg1, PSTR("distance"))) {
		trajectory_set_acc(&mainboard.traj, res->s, mainboard.traj.a_acc);
	}
	/* else it is a "show" */

	printf_P(PSTR("angle %2.2f, distance %2.2f\r\n"),
		 mainboard.traj.a_acc,
		 mainboard.traj.d_acc);
}

prog_char str_traj_acc_arg0[] = "traj_acc";
parse_pgm_token_string_t cmd_traj_acc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg0, str_traj_acc_arg0);
prog_char str_traj_acc_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_acc_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_arg1);
parse_pgm_token_num_t cmd_traj_acc_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_acc_result, s, FLOAT);

prog_char help_traj_acc[] = "Set traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc = {
	.f = cmd_traj_acc_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_traj_acc,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_traj_acc_arg0,
		(prog_void *)&cmd_traj_acc_arg1,
		(prog_void *)&cmd_traj_acc_s,
		NULL,
	},
};

/* show */

prog_char str_traj_acc_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_acc_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_show_arg);

prog_char help_traj_acc_show[] = "Show traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc_show = {
	.f = cmd_traj_acc_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_traj_acc_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_traj_acc_arg0,
		(prog_void *)&cmd_traj_acc_show_arg,
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
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_rs_gains_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		rs_set_left_ext_encoder(&mainboard.rs, encoders_spi_get_value,
					LEFT_ENCODER, res->left); // en augmentant on tourne à gauche
		rs_set_right_ext_encoder(&mainboard.rs, encoders_spi_get_value,
					 RIGHT_ENCODER, res->right); //en augmentant on tourne à droite
	}
	printf_P(PSTR("rs_gains set %2.2f %2.2f\r\n"),
		 mainboard.rs.left_ext_gain, mainboard.rs.right_ext_gain);
#endif
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
/* centrifugal configuration */

/* this structure is filled when cmd_centrifugal is parsed successfully */
struct cmd_centrifugal_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float val;
};

/* function called when cmd_centrifugal is parsed successfully */
static void cmd_centrifugal_parsed(void * parsed_result, void * data)
{
	struct cmd_centrifugal_result * res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		position_set_centrifugal_coef(&mainboard.pos, res->val);
	}
	printf_P(PSTR("centrifugal set %f\r\n"), mainboard.pos.centrifugal_coef);
}

prog_char str_centrifugal_arg0[] = "centrifugal";
parse_pgm_token_string_t cmd_centrifugal_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg0, str_centrifugal_arg0);
prog_char str_centrifugal_arg1[] = "set";
parse_pgm_token_string_t cmd_centrifugal_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_arg1);
parse_pgm_token_num_t cmd_centrifugal_val = TOKEN_NUM_INITIALIZER(struct cmd_centrifugal_result, val, FLOAT);

prog_char help_centrifugal[] = "Set centrifugal coef";
parse_pgm_inst_t cmd_centrifugal = {
	.f = cmd_centrifugal_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_centrifugal,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_centrifugal_arg0,
		(prog_void *)&cmd_centrifugal_arg1,
		(prog_void *)&cmd_centrifugal_val,
		NULL,
	},
};

/* show */

prog_char str_centrifugal_show_arg[] = "show";
parse_pgm_token_string_t cmd_centrifugal_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_show_arg);

prog_char help_centrifugal_show[] = "Show centrifugal";
parse_pgm_inst_t cmd_centrifugal_show = {
	.f = cmd_centrifugal_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_centrifugal_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_centrifugal_arg0,
		(prog_void *)&cmd_centrifugal_show_arg,
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
	if (!strcmp_P(res->arg1, PSTR("avoid_start"))) {
		printf_P(PSTR("removed\r\n"));
		return;
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
 restart:
	for (i=0 ; i<pt_list_len ; i++) {
		printf_P(PSTR("%d: x=%d y=%d\r\n"), i, pt_list[i].x, pt_list[i].y);
		if (!strcmp_P(res->arg1, PSTR("start"))) {
			trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x, pt_list[i].y);
			why = wait_traj_end(0xFF); /* all */
		}
		else if (!strcmp_P(res->arg1, PSTR("loop_start"))) {
			trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x, pt_list[i].y);
			why = wait_traj_end(0xFF); /* all */
		}
#if 0
		else if (!strcmp_P(res->arg1, PSTR("avoid_start"))) {
			while (1) {
				why = goto_and_avoid(pt_list[i].x, pt_list[i].y, 0xFF, 0xFF);
				printf("next point\r\n");
				if (why != END_OBSTACLE)
					break;
			}
		}
#endif
		if (why & (~(END_TRAJ | END_NEAR)))
			trajectory_stop(&mainboard.traj);
		if (why & END_INTR)
			break;
	}
	if (why & END_INTR)
		return;
	if (!strcmp_P(res->arg1, PSTR("loop_start")))
		goto restart;
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

prog_char str_pt_list_show_arg[] = "show#reset#start#avoid_start#loop_start";
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
#if 0
		err = goto_and_avoid_forward(res->arg2, res->arg3, 0xFF, 0xFF);
		if (err != END_TRAJ && err != END_NEAR)
			strat_hardstop();
#else
		printf_P(PSTR("not implemented\r\n"));
		return;
#endif
	}
	else if (!strcmp_P(res->arg1, PSTR("avoid_bw"))) {
#if 0
		err = goto_and_avoid_backward(res->arg2, res->arg3, 0xFF, 0xFF);
		if (err != END_TRAJ && err != END_NEAR)
			strat_hardstop();
#else
		printf_P(PSTR("not implemented\r\n"));
		return;
#endif
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
	strat_set_acc(3, 3);

	err = strat_calib(300, END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	strat_reset_pos(ROBOT_WIDTH/2 + 100,
			COLOR_Y(ROBOT_HALF_LENGTH_FRONT),
			COLOR_A(-90));
	strat_hardstop();

	trajectory_d_rel(&mainboard.traj, -180);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	time_wait_ms(250);
	trajectory_a_rel(&mainboard.traj, COLOR_A(-90));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	time_wait_ms(250);
	err = strat_calib(300, END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	strat_reset_pos(ROBOT_HALF_LENGTH_FRONT,
			DO_NOT_SET_POS,
			180);
	strat_hardstop();

	trajectory_d_rel(&mainboard.traj, -170);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	time_wait_ms(250);

	trajectory_a_rel(&mainboard.traj, COLOR_A(-110));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	time_wait_ms(250);

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
	else if (!strcmp_P(res->arg1, PSTR("autoset_blue"))) {
		mainboard.our_color = I2C_COLOR_BLUE;
#ifndef HOST_VERSION
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_BLUE);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_BLUE);
#endif
		auto_position();
	}
	else if (!strcmp_P(res->arg1, PSTR("autoset_yellow"))) {
		mainboard.our_color = I2C_COLOR_YELLOW;
#ifndef HOST_VERSION
		i2c_set_color(I2C_COBBOARD_ADDR, I2C_COLOR_YELLOW);
		i2c_set_color(I2C_BALLBOARD_ADDR, I2C_COLOR_YELLOW);
#endif
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
prog_char str_position_arg1[] = "show#reset#autoset_blue#autoset_yellow";
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

/* this structure is filled when cmd_strat_db is parsed successfully */
struct cmd_strat_db_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_strat_db is parsed successfully */
static void cmd_strat_db_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_db_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		strat_db_init();
	}
	strat_db.dump_enabled = 1;
	strat_db_dump(__FUNCTION__);
}

prog_char str_strat_db_arg0[] = "strat_db";
parse_pgm_token_string_t cmd_strat_db_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_db_result, arg0, str_strat_db_arg0);
prog_char str_strat_db_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_strat_db_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_db_result, arg1, str_strat_db_arg1);

prog_char help_strat_db[] = "reset/show strat_db";
parse_pgm_inst_t cmd_strat_db = {
	.f = cmd_strat_db_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_db,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_db_arg0,
		(prog_void *)&cmd_strat_db_arg1,
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
	//	struct cmd_strat_conf_result *res = parsed_result;
	strat_conf.dump_enabled = 1;
	strat_conf_dump(__FUNCTION__);
}

prog_char str_strat_conf_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg0, str_strat_conf_arg0);
prog_char str_strat_conf_arg1[] = "show#base";
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

#if 0
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
#endif

	if (on)
		strat_conf.flags |= bit;
	else
		strat_conf.flags &= (~bit);

	strat_conf.dump_enabled = 1;
	strat_conf_dump(__FUNCTION__);
}

prog_char str_strat_conf2_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg0, str_strat_conf2_arg0);
prog_char str_strat_conf2_arg1[] = "faux";
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
#if 0
	struct cmd_strat_conf3_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("scan_opponent_min_time"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_conf.scan_opp_min_time = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("delay_between_opponent_scan"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_conf.delay_between_opp_scan = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("scan_our_min_time"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_conf.scan_our_min_time = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("delay_between_our_scan"))) {
		if (res->arg2 > 90)
			res->arg2 = 90;
		strat_conf.delay_between_our_scan = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("wait_opponent"))) {
		strat_conf.wait_opponent = res->arg2;
	}
	else if (!strcmp_P(res->arg1, PSTR("lintel_min_time"))) {
		strat_conf.lintel_min_time = res->arg2;
	}
#endif
	strat_conf.dump_enabled = 1;
	strat_conf_dump(__FUNCTION__);
}

prog_char str_strat_conf3_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg0, str_strat_conf3_arg0);
prog_char str_strat_conf3_arg1[] = "faux2";
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
/* Subtraj */

//////////////////////

// 500 -- 5
// 400 -- 3
#define TEST_SPEED 400
#define TEST_ACC 3

static void line2line(double line1x1, double line1y1,
		      double line1x2, double line1y2,
		      double line2x1, double line2y1,
		      double line2x2, double line2y2,
		      double radius, double dist)
{
	uint8_t err;
	double speed_d, speed_a;
	double distance, angle;
	double line1_angle = atan2(line1y2-line1y1, line1x2-line1x1);
	double line2_angle = atan2(line2y2-line2y1, line2x2-line2x1);

	printf_P(PSTR("%s()\r\n"), __FUNCTION__);

	strat_set_speed(TEST_SPEED, TEST_SPEED);
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, TEST_ACC, TEST_ACC);

	circle_get_da_speed_from_radius(&mainboard.traj, radius,
					&speed_d, &speed_a);
	trajectory_line_abs(&mainboard.traj,
			    line1x1, line1y1,
			    line1x2, line1y2, 150.);
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(line1x2, line1y2) <
				    dist, TRAJ_FLAGS_NO_NEAR);
	/* circle */
	strat_set_speed(speed_d, speed_a);
	angle = line2_angle - line1_angle;
	distance = angle * radius;
	if (distance < 0)
		distance = -distance;
	angle = simple_modulo_2pi(angle);
	angle = DEG(angle);
	printf_P(PSTR("(%d,%d,%d) "),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
	printf_P(PSTR("circle distance=%2.2f angle=%2.2f\r\n"),
		 distance, angle);

	/* take some margin on dist to avoid deceleration */
	trajectory_d_a_rel(&mainboard.traj, distance + 250, angle);

	/* circle exit condition */
	err = WAIT_COND_OR_TRAJ_END(trajectory_angle_finished(&mainboard.traj),
				    TRAJ_FLAGS_NO_NEAR);

	strat_set_speed(500, 500);
	printf_P(PSTR("(%d,%d,%d) "),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
	printf_P(PSTR("line\r\n"));
	trajectory_line_abs(&mainboard.traj,
			    line2x1, line2y1,
			    line2x2, line2y2, 150.);
}

static void halfturn(double line1x1, double line1y1,
		     double line1x2, double line1y2,
		     double line2x1, double line2y1,
		     double line2x2, double line2y2,
		     double radius, double dist, double dir)
{
	uint8_t err;
	double speed_d, speed_a;
	double distance, angle;

	printf_P(PSTR("%s()\r\n"), __FUNCTION__);

	strat_set_speed(TEST_SPEED, TEST_SPEED);
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, TEST_ACC, TEST_ACC);

	circle_get_da_speed_from_radius(&mainboard.traj, radius,
					&speed_d, &speed_a);
	trajectory_line_abs(&mainboard.traj,
			    line1x1, line1y1,
			    line1x2, line1y2, 150.);
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(line1x2, line1y2) <
				    dist, TRAJ_FLAGS_NO_NEAR);
	/* circle */
	strat_set_speed(speed_d, speed_a);
	angle = dir * M_PI/2.;
	distance = angle * radius;
	if (distance < 0)
		distance = -distance;
	angle = simple_modulo_2pi(angle);
	angle = DEG(angle);

	/* take some margin on dist to avoid deceleration */
	DEBUG(E_USER_STRAT, "circle1 distance=%2.2f angle=%2.2f",
	      distance, angle);
	trajectory_d_a_rel(&mainboard.traj, distance + 500, angle);

	/* circle exit condition */
	err = WAIT_COND_OR_TRAJ_END(trajectory_angle_finished(&mainboard.traj),
				    TRAJ_FLAGS_NO_NEAR);

	DEBUG(E_USER_STRAT, "miniline");
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(line2x1, line2y1) <
				    dist, TRAJ_FLAGS_NO_NEAR);
	DEBUG(E_USER_STRAT, "circle2");
	/* take some margin on dist to avoid deceleration */
	trajectory_d_a_rel(&mainboard.traj, distance + 500, angle);

	err = WAIT_COND_OR_TRAJ_END(trajectory_angle_finished(&mainboard.traj),
				    TRAJ_FLAGS_NO_NEAR);

	strat_set_speed(500, 500);
	DEBUG(E_USER_STRAT, "line");
	trajectory_line_abs(&mainboard.traj,
			    line2x1, line2y1,
			    line2x2, line2y2, 150.);
}


/* function called when cmd_test is parsed successfully */
static void subtraj_test(void)
{
#ifdef HOST_VERSION
	strat_reset_pos(400, 400, 90);
	mainboard.angle.on = 1;
	mainboard.distance.on = 1;
#endif
	printf_P(PSTR("%s()\r\n"), __FUNCTION__);
	while (!cmdline_keypressed()) {
		/****** PASS1 */

#define DIST_HARD_TURN   260
#define RADIUS_HARD_TURN 100
#define DIST_EASY_TURN   190
#define RADIUS_EASY_TURN 190
#define DIST_HALF_TURN   225
#define RADIUS_HALF_TURN 130

		/* hard turn */
		line2line(375, 597, 375, 1847,
			  375, 1847, 1050, 1472,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

		/* easy left and easy right !*/
		line2line(825, 1596, 1050, 1472,
			  1050, 1472, 1500, 1722,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);
		line2line(1050, 1472, 1500, 1722,
			  1500, 1722, 2175, 1347,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);
		line2line(1500, 1722, 2175, 1347,
			  2175, 1347, 2175, 847,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);

		/* half turns */
		halfturn(2175, 1347, 2175, 722,
			 2625, 722, 2625, 1597,
			 RADIUS_HALF_TURN, DIST_HALF_TURN, 1.);
		halfturn(2625, 847, 2625, 1722,
			  2175, 1722, 2175, 1097,
			  RADIUS_HALF_TURN, DIST_HALF_TURN, 1.);

		/* easy turns */
		line2line(2175, 1597, 2175, 1097,
			  2175, 1097, 1500, 722,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);
		line2line(2175, 1097, 1500, 722,
			  1500, 722, 1050, 972,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);
		line2line(1500, 722, 1050, 972,
			  1050, 972, 375, 597,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);

		/* hard turn */
		line2line(1050, 972, 375, 597,
			  375, 597, 375, 1097,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

		/****** PASS2 */

		/* easy turn */
		line2line(375, 597, 375, 1097,
			  375, 1097, 1050, 1472,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);

		/* hard turn */
		line2line(375, 1097, 1050, 1472,
			  1050, 1472, 375, 1847,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

		/* hard turn */
		line2line(1050, 1472, 375, 1847,
			  375, 1847, 375, 1347,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

		/* easy turn */
		line2line(375, 1847, 375, 1347,
			  375, 1347, 1050, 972,
			  RADIUS_EASY_TURN, DIST_EASY_TURN);

		/* hard turn */
		line2line(375, 1347, 1050, 972,
			  1050, 972, 375, 597,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

		/* hard turn */
		line2line(1050, 972, 375, 597,
			  375, 597, 375, 1847,
			  RADIUS_HARD_TURN, DIST_HARD_TURN);

	}
	trajectory_hardstop(&mainboard.traj);
}


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

	if (!strcmp_P(res->arg1, PSTR("test")))
		subtraj_test();

	trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj_result, arg0, str_subtraj_arg0);
prog_char str_subtraj_arg1[] = "faux";
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

