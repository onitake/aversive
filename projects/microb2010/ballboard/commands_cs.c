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
 *  Revision : $Id: commands_cs.c,v 1.1 2009-03-29 18:44:54 zer0 Exp $
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

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cs.h"
#include "cmdline.h"

struct csb_list {
	const prog_char *name;
	struct cs_block *csb;
};

prog_char csb_roller_str[] = "roller";
prog_char csb_forktrans_str[] = "forktrans";
prog_char csb_forkrot_str[] = "forkrot";
struct csb_list csb_list[] = {
	{ .name = csb_roller_str, .csb = &ballboard.roller },
	{ .name = csb_forktrans_str, .csb = &ballboard.forktrans },
	{ .name = csb_forkrot_str, .csb = &ballboard.forkrot },
};

struct cmd_cs_result {
	fixed_string_t cmdname;
	fixed_string_t csname;
};

/* token to be used for all cs-related commands */
prog_char str_csb_name[] = "roller#forktrans#forkrot";
parse_pgm_token_string_t cmd_csb_name_tok = TOKEN_STRING_INITIALIZER(struct cmd_cs_result, csname, str_csb_name);

struct cs_block *cs_from_name(const char *name)
{
	int i;

	for (i=0; i<(sizeof(csb_list)/sizeof(*csb_list)); i++) {
		if (!strcmp_P(name, csb_list[i].name))
			return csb_list[i].csb;
	}
	return NULL;
}
		
/**********************************************************/
/* Gains for control system */

/* this structure is filled when cmd_gain is parsed successfully */
struct cmd_gain_result {
	struct cmd_cs_result cs;
	int16_t p;
	int16_t i;
	int16_t d;
};

/* function called when cmd_gain is parsed successfully */
static void cmd_gain_parsed(void * parsed_result, void *show)
{
	struct cmd_gain_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show) 
		pid_set_gains(&csb->pid, res->p, res->i, res->d);

	printf_P(PSTR("%s %s %d %d %d\r\n"),
		 res->cs.cmdname,
		 res->cs.csname,
		 pid_get_gain_P(&csb->pid),
		 pid_get_gain_I(&csb->pid),
		 pid_get_gain_D(&csb->pid));
}

prog_char str_gain_arg0[] = "gain";
parse_pgm_token_string_t cmd_gain_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_gain_result, cs.cmdname, str_gain_arg0);
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
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_gain_p, 
		(prog_void *)&cmd_gain_i, 
		(prog_void *)&cmd_gain_d, 
		NULL,
	},
};

/* show */
/* this structure is filled when cmd_gain is parsed successfully */
struct cmd_gain_show_result {
	struct cmd_cs_result cs;
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
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_gain_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Speeds for control system */

/* this structure is filled when cmd_speed is parsed successfully */
struct cmd_speed_result {
	struct cmd_cs_result cs;
	uint16_t s;
};

/* function called when cmd_speed is parsed successfully */
static void cmd_speed_parsed(void *parsed_result, void *show)
{
	struct cmd_speed_result * res = parsed_result;
	
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

#if notyet
	if (!show) 
		ramp_set_vars(&csb->ramp, res->s, res->s); /* set speed */

	printf_P(PSTR("%s %lu\r\n"), 
		 res->cs.csname,
		 ext.r_b.var_pos);
#else
	printf_P(PSTR("TODO\r\n"));
#endif
}

prog_char str_speed_arg0[] = "speed";
parse_pgm_token_string_t cmd_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_speed_result, cs.cmdname, str_speed_arg0);
parse_pgm_token_num_t cmd_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_speed_result, s, UINT16);

prog_char help_speed[] = "Set speed values for ramp filter";
parse_pgm_inst_t cmd_speed = {
	.f = cmd_speed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_speed,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_speed_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_speed_s, 
		NULL,
	},
};

/* show */
struct cmd_speed_show_result {
	struct cmd_cs_result cs;
	fixed_string_t show;
};

prog_char str_speed_show_arg[] = "show";
parse_pgm_token_string_t cmd_speed_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_speed_show_result, show, str_speed_show_arg);

prog_char help_speed_show[] = "Show speed values for ramp filter";
parse_pgm_inst_t cmd_speed_show = {
	.f = cmd_speed_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_speed_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_speed_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_speed_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Derivate_Filters for control system */

/* this structure is filled when cmd_derivate_filter is parsed successfully */
struct cmd_derivate_filter_result {
	struct cmd_cs_result cs;
	uint8_t size;
};

/* function called when cmd_derivate_filter is parsed successfully */
static void cmd_derivate_filter_parsed(void *parsed_result, void *show)
{
	struct cmd_derivate_filter_result * res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show) 
		pid_set_derivate_filter(&csb->pid, res->size);

	printf_P(PSTR("%s %s %u\r\n"), 
		 res->cs.cmdname,
		 res->cs.csname,
		 pid_get_derivate_filter(&csb->pid));
}

prog_char str_derivate_filter_arg0[] = "derivate_filter";
parse_pgm_token_string_t cmd_derivate_filter_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_derivate_filter_result, cs.cmdname, str_derivate_filter_arg0);
parse_pgm_token_num_t cmd_derivate_filter_size = TOKEN_NUM_INITIALIZER(struct cmd_derivate_filter_result, size, UINT32);

prog_char help_derivate_filter[] = "Set derivate_filter values for PID (in, I, out)";
parse_pgm_inst_t cmd_derivate_filter = {
	.f = cmd_derivate_filter_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_derivate_filter,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_derivate_filter_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_derivate_filter_size, 
		NULL,
	},
};

/* show */

struct cmd_derivate_filter_show_result {
	struct cmd_cs_result cs;
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
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_derivate_filter_show_arg,
		NULL,
	},
};


/**********************************************************/
/* Consign for control system */

/* this structure is filled when cmd_consign is parsed successfully */
struct cmd_consign_result {
	struct cmd_cs_result cs;
	int32_t p;
};

/* function called when cmd_consign is parsed successfully */
static void cmd_consign_parsed(void * parsed_result, void *data)
{
	struct cmd_consign_result * res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	cs_set_consign(&csb->cs, res->p);
}

prog_char str_consign_arg0[] = "consign";
parse_pgm_token_string_t cmd_consign_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_consign_result, cs.cmdname, str_consign_arg0);
parse_pgm_token_num_t cmd_consign_p = TOKEN_NUM_INITIALIZER(struct cmd_consign_result, p, INT32);

prog_char help_consign[] = "Set consign value";
parse_pgm_inst_t cmd_consign = {
	.f = cmd_consign_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_consign,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_consign_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_consign_p, 
		NULL,
	},
};


/**********************************************************/
/* Maximums for control system */

/* this structure is filled when cmd_maximum is parsed successfully */
struct cmd_maximum_result {
	struct cmd_cs_result cs;
	uint32_t in;
	uint32_t i;
	uint32_t out;
};

/* function called when cmd_maximum is parsed successfully */
static void cmd_maximum_parsed(void *parsed_result, void *show)
{
	struct cmd_maximum_result * res = parsed_result;
	
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		pid_set_maximums(&csb->pid, res->in, res->i, res->out);

	printf_P(PSTR("maximum %s %lu %lu %lu\r\n"), 
		 res->cs.csname,
		 pid_get_max_in(&csb->pid),
		 pid_get_max_I(&csb->pid),
		 pid_get_max_out(&csb->pid));
}

prog_char str_maximum_arg0[] = "maximum";
parse_pgm_token_string_t cmd_maximum_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_maximum_result, cs.cmdname, str_maximum_arg0);
parse_pgm_token_num_t cmd_maximum_in = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, in, UINT32);
parse_pgm_token_num_t cmd_maximum_i = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, i, UINT32);
parse_pgm_token_num_t cmd_maximum_out = TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, out, UINT32);

prog_char help_maximum[] = "Set maximum values for PID (in, I, out)";
parse_pgm_inst_t cmd_maximum = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_maximum,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_maximum_in, 
		(prog_void *)&cmd_maximum_i, 
		(prog_void *)&cmd_maximum_out, 
		NULL,
	},
};

/* show */

/* this structure is filled when cmd_maximum is parsed successfully */
struct cmd_maximum_show_result {
	struct cmd_cs_result cs;
	fixed_string_t show;
};
prog_char str_maximum_show_arg[] = "show";
parse_pgm_token_string_t cmd_maximum_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_maximum_show_result, show, str_maximum_show_arg);

prog_char help_maximum_show[] = "Show maximum values for PID";
parse_pgm_inst_t cmd_maximum_show = {
	.f = cmd_maximum_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_maximum_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_maximum_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_maximum_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Quadramp for control system */

/* this structure is filled when cmd_quadramp is parsed successfully */
struct cmd_quadramp_result {
	struct cmd_cs_result cs;
	double ap;
	double an;
	double sp;
	double sn;
};

/* function called when cmd_quadramp is parsed successfully */
static void cmd_quadramp_parsed(void *parsed_result, void *show)
{
	struct cmd_quadramp_result * res = parsed_result;

	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)  {
		quadramp_set_1st_order_vars(&csb->qr, res->sp, res->sn);
		quadramp_set_2nd_order_vars(&csb->qr, res->ap, res->an);
	}

	printf_P(PSTR("quadramp %s %2.2f %2.2f %2.2f %2.2f\r\n"),
		 res->cs.csname,
		 csb->qr.var_2nd_ord_pos,
		 csb->qr.var_2nd_ord_neg,
		 csb->qr.var_1st_ord_pos,
		 csb->qr.var_1st_ord_neg);
}

prog_char str_quadramp_arg0[] = "quadramp";
parse_pgm_token_string_t cmd_quadramp_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_quadramp_result, cs.cmdname, str_quadramp_arg0);
parse_pgm_token_num_t cmd_quadramp_ap = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, ap, FLOAT);
parse_pgm_token_num_t cmd_quadramp_an = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, an, FLOAT);
parse_pgm_token_num_t cmd_quadramp_sp = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sp, FLOAT);
parse_pgm_token_num_t cmd_quadramp_sn = TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sn, FLOAT);

prog_char help_quadramp[] = "Set quadramp values (acc+, acc-, speed+, speed-)";
parse_pgm_inst_t cmd_quadramp = {
	.f = cmd_quadramp_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_quadramp,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_quadramp_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_quadramp_ap,
		(prog_void *)&cmd_quadramp_an,
		(prog_void *)&cmd_quadramp_sp,
		(prog_void *)&cmd_quadramp_sn,

		NULL,
	},
};

/* show */

struct cmd_quadramp_show_result {
	struct cmd_cs_result cs;
	fixed_string_t show;
};

prog_char str_quadramp_show_arg[] = "show";
parse_pgm_token_string_t cmd_quadramp_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_quadramp_show_result, show, str_quadramp_show_arg);

prog_char help_quadramp_show[] = "Get quadramp values for control system";
parse_pgm_inst_t cmd_quadramp_show = {
	.f = cmd_quadramp_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_quadramp_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_quadramp_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_quadramp_show_arg, 
		NULL,
	},
};



/**********************************************************/
/* cs_status show for control system */

/* this structure is filled when cmd_cs_status is parsed successfully */
struct cmd_cs_status_result {
	struct cmd_cs_result cs;
	fixed_string_t arg;
};

/* function called when cmd_cs_status is parsed successfully */
static void cmd_cs_status_parsed(void *parsed_result, void *data)
{
	struct cmd_cs_status_result *res = parsed_result;
	struct cs_block *csb;
	uint8_t loop = 0;
	uint8_t print_pid = 0, print_cs = 0;
	
	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}
	if (strcmp_P(res->arg, PSTR("on")) == 0) {
		csb->on = 1;
		printf_P(PSTR("%s is on\r\n"), res->cs.csname);
		return;
	}
	else if (strcmp_P(res->arg, PSTR("off")) == 0) {
		csb->on = 0;
		printf_P(PSTR("%s is off\r\n"), res->cs.csname);
		return;
	}
	else if (strcmp_P(res->arg, PSTR("show")) == 0) {
		print_cs = 1;
	}
	else if (strcmp_P(res->arg, PSTR("loop_show")) == 0) {
		loop = 1;
		print_cs = 1;
	}
	else if (strcmp_P(res->arg, PSTR("pid_show")) == 0) {
		print_pid = 1;
	}
	else if (strcmp_P(res->arg, PSTR("pid_loop_show")) == 0) {
		print_pid = 1;
		loop = 1;
	}

	printf_P(PSTR("%s cs is %s\r\n"), res->cs.csname, csb->on ? "on":"off");
	do {
		if (print_cs)
			dump_cs(res->cs.csname, &csb->cs);
		if (print_pid)
			dump_pid(res->cs.csname, &csb->pid);
		wait_ms(100);
	} while(loop && !cmdline_keypressed());
}

prog_char str_cs_status_arg0[] = "cs_status";
parse_pgm_token_string_t cmd_cs_status_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cs_status_result, cs.cmdname, str_cs_status_arg0);
prog_char str_cs_status_arg[] = "pid_show#pid_loop_show#show#loop_show#on#off";
parse_pgm_token_string_t cmd_cs_status_arg = TOKEN_STRING_INITIALIZER(struct cmd_cs_status_result, arg, str_cs_status_arg);

prog_char help_cs_status[] = "Show cs status";
parse_pgm_inst_t cmd_cs_status = {
	.f = cmd_cs_status_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cs_status,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cs_status_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_cs_status_arg, 
		NULL,
	},
};


/**********************************************************/
/* Blocking_I for control system */

/* this structure is filled when cmd_blocking_i is parsed successfully */
struct cmd_blocking_i_result {
	struct cmd_cs_result cs;
	int32_t k1;
	int32_t k2;
	uint32_t i;
	uint16_t cpt;
};

/* function called when cmd_blocking_i is parsed successfully */
static void cmd_blocking_i_parsed(void *parsed_result, void *show)
{
	struct cmd_blocking_i_result * res = parsed_result;
	
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		bd_set_current_thresholds(&csb->bd, res->k1, res->k2,
					  res->i, res->cpt);

	printf_P(PSTR("%s %s %ld %ld %ld %d\r\n"), 
		 res->cs.cmdname,
		 res->cs.csname,
		 csb->bd.k1,
		 csb->bd.k2,
		 csb->bd.i_thres,
		 csb->bd.cpt_thres);
}

prog_char str_blocking_i_arg0[] = "blocking";
parse_pgm_token_string_t cmd_blocking_i_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_blocking_i_result, cs.cmdname, str_blocking_i_arg0);
parse_pgm_token_num_t cmd_blocking_i_k1 = TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, k1, INT32);
parse_pgm_token_num_t cmd_blocking_i_k2 = TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, k2, INT32);
parse_pgm_token_num_t cmd_blocking_i_i = TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, i, UINT32);
parse_pgm_token_num_t cmd_blocking_i_cpt = TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, cpt, UINT16);

prog_char help_blocking_i[] = "Set blocking detection values (k1, k2, i, cpt)";
parse_pgm_inst_t cmd_blocking_i = {
	.f = cmd_blocking_i_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_blocking_i,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_blocking_i_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_blocking_i_k1, 
		(prog_void *)&cmd_blocking_i_k2, 
		(prog_void *)&cmd_blocking_i_i, 
		(prog_void *)&cmd_blocking_i_cpt,
		NULL,
	},
};

/* show */

struct cmd_blocking_i_show_result {
	struct cmd_cs_result cs;
	fixed_string_t show;
};

prog_char str_blocking_i_show_arg[] = "show";
parse_pgm_token_string_t cmd_blocking_i_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_blocking_i_show_result, show, str_blocking_i_show_arg);

prog_char help_blocking_i_show[] = "Show blocking detection values";
parse_pgm_inst_t cmd_blocking_i_show = {
	.f = cmd_blocking_i_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_blocking_i_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_blocking_i_arg0, 
		(prog_void *)&cmd_csb_name_tok, 
		(prog_void *)&cmd_blocking_i_show_arg, 
		NULL,
	},
};


