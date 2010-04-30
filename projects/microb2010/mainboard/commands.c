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
 *  Revision : $Id: commands.c,v 1.9 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdlib.h>
#include <aversive/pgmspace.h>
#include <parse.h>

/* commands_gen.c */
extern parse_pgm_inst_t cmd_reset;
extern parse_pgm_inst_t cmd_bootloader;
extern parse_pgm_inst_t cmd_encoders;
extern parse_pgm_inst_t cmd_pwm;
extern parse_pgm_inst_t cmd_adc;
extern parse_pgm_inst_t cmd_sensor;
extern parse_pgm_inst_t cmd_log;
extern parse_pgm_inst_t cmd_log_show;
extern parse_pgm_inst_t cmd_log_type;
extern parse_pgm_inst_t cmd_stack_space;
extern parse_pgm_inst_t cmd_scheduler;

/* commands_ax12.c */
extern parse_pgm_inst_t cmd_baudrate;
extern parse_pgm_inst_t cmd_uint16_read;
extern parse_pgm_inst_t cmd_uint16_write;
extern parse_pgm_inst_t cmd_uint8_read;
extern parse_pgm_inst_t cmd_uint8_write;

/* commands_cs.c */
extern parse_pgm_inst_t cmd_gain;
extern parse_pgm_inst_t cmd_gain_show;
extern parse_pgm_inst_t cmd_speed;
extern parse_pgm_inst_t cmd_speed_show;
extern parse_pgm_inst_t cmd_derivate_filter;
extern parse_pgm_inst_t cmd_derivate_filter_show;
extern parse_pgm_inst_t cmd_consign;
extern parse_pgm_inst_t cmd_maximum;
extern parse_pgm_inst_t cmd_maximum_show;
extern parse_pgm_inst_t cmd_quadramp;
extern parse_pgm_inst_t cmd_quadramp_show;
extern parse_pgm_inst_t cmd_cs_status;
extern parse_pgm_inst_t cmd_blocking_i;
extern parse_pgm_inst_t cmd_blocking_i_show;

/* commands_mainboard.c */
extern parse_pgm_inst_t cmd_event;
extern parse_pgm_inst_t cmd_spi_test;
extern parse_pgm_inst_t cmd_opponent;
extern parse_pgm_inst_t cmd_opponent_set;
extern parse_pgm_inst_t cmd_start;
extern parse_pgm_inst_t cmd_interact;
extern parse_pgm_inst_t cmd_color;
extern parse_pgm_inst_t cmd_rs;
extern parse_pgm_inst_t cmd_i2cdebug;
extern parse_pgm_inst_t cmd_cobboard_show;
extern parse_pgm_inst_t cmd_cobboard_setmode1;
extern parse_pgm_inst_t cmd_cobboard_setmode2;
extern parse_pgm_inst_t cmd_cobboard_setmode3;
extern parse_pgm_inst_t cmd_ballboard_show;
extern parse_pgm_inst_t cmd_ballboard_setmode1;
extern parse_pgm_inst_t cmd_ballboard_setmode2;
extern parse_pgm_inst_t cmd_ballboard_setmode3;
extern parse_pgm_inst_t cmd_beacon_start;
extern parse_pgm_inst_t cmd_servo_balls;
extern parse_pgm_inst_t cmd_clitoid;
extern parse_pgm_inst_t cmd_time_monitor;
extern parse_pgm_inst_t cmd_strat_event;
extern parse_pgm_inst_t cmd_test;
extern parse_pgm_inst_t cmd_climb;

/* commands_traj.c */
extern parse_pgm_inst_t cmd_traj_speed;
extern parse_pgm_inst_t cmd_traj_speed_show;
extern parse_pgm_inst_t cmd_traj_acc;
extern parse_pgm_inst_t cmd_traj_acc_show;
extern parse_pgm_inst_t cmd_trajectory;
extern parse_pgm_inst_t cmd_trajectory_show;
extern parse_pgm_inst_t cmd_circle_coef;
extern parse_pgm_inst_t cmd_circle_coef_show;
extern parse_pgm_inst_t cmd_rs_gains;
extern parse_pgm_inst_t cmd_rs_gains_show;
extern parse_pgm_inst_t cmd_track;
extern parse_pgm_inst_t cmd_track_show;
extern parse_pgm_inst_t cmd_pt_list;
extern parse_pgm_inst_t cmd_pt_list_append;
extern parse_pgm_inst_t cmd_pt_list_del;
extern parse_pgm_inst_t cmd_pt_list_show;
extern parse_pgm_inst_t cmd_goto1;
extern parse_pgm_inst_t cmd_goto2;
extern parse_pgm_inst_t cmd_goto3;
extern parse_pgm_inst_t cmd_position;
extern parse_pgm_inst_t cmd_position_set;
extern parse_pgm_inst_t cmd_strat_db;
extern parse_pgm_inst_t cmd_strat_conf;
extern parse_pgm_inst_t cmd_strat_conf2;
extern parse_pgm_inst_t cmd_strat_conf3;
extern parse_pgm_inst_t cmd_strat_conf4;
extern parse_pgm_inst_t cmd_subtraj;

/* in progmem */
parse_pgm_ctx_t main_ctx[] = {

	/* commands_gen.c */
	(parse_pgm_inst_t *)&cmd_reset,
	(parse_pgm_inst_t *)&cmd_bootloader,
	(parse_pgm_inst_t *)&cmd_encoders,
	(parse_pgm_inst_t *)&cmd_pwm,
	(parse_pgm_inst_t *)&cmd_adc,
	(parse_pgm_inst_t *)&cmd_sensor,
	(parse_pgm_inst_t *)&cmd_log,
	(parse_pgm_inst_t *)&cmd_log_show,
	(parse_pgm_inst_t *)&cmd_log_type,
	(parse_pgm_inst_t *)&cmd_stack_space,
	(parse_pgm_inst_t *)&cmd_scheduler,

	/* commands_ax12.c */
	(parse_pgm_inst_t *)&cmd_baudrate,
	(parse_pgm_inst_t *)&cmd_uint16_read,
	(parse_pgm_inst_t *)&cmd_uint16_write,
	(parse_pgm_inst_t *)&cmd_uint8_read,
	(parse_pgm_inst_t *)&cmd_uint8_write,

	/* commands_cs.c */
	(parse_pgm_inst_t *)&cmd_gain,
	(parse_pgm_inst_t *)&cmd_gain_show,
	(parse_pgm_inst_t *)&cmd_speed,
	(parse_pgm_inst_t *)&cmd_speed_show,
	(parse_pgm_inst_t *)&cmd_consign,
	(parse_pgm_inst_t *)&cmd_derivate_filter,
	(parse_pgm_inst_t *)&cmd_derivate_filter_show,
	(parse_pgm_inst_t *)&cmd_maximum,
	(parse_pgm_inst_t *)&cmd_maximum_show,
	(parse_pgm_inst_t *)&cmd_quadramp,
	(parse_pgm_inst_t *)&cmd_quadramp_show,
	(parse_pgm_inst_t *)&cmd_cs_status,
	(parse_pgm_inst_t *)&cmd_blocking_i,
	(parse_pgm_inst_t *)&cmd_blocking_i_show,

	/* commands_mainboard.c */
	(parse_pgm_inst_t *)&cmd_event,
	(parse_pgm_inst_t *)&cmd_spi_test,
	(parse_pgm_inst_t *)&cmd_opponent,
	(parse_pgm_inst_t *)&cmd_opponent_set,
	(parse_pgm_inst_t *)&cmd_start,
	(parse_pgm_inst_t *)&cmd_interact,
	(parse_pgm_inst_t *)&cmd_color,
	(parse_pgm_inst_t *)&cmd_rs,
	(parse_pgm_inst_t *)&cmd_i2cdebug,
	(parse_pgm_inst_t *)&cmd_cobboard_show,
	(parse_pgm_inst_t *)&cmd_cobboard_setmode1,
	(parse_pgm_inst_t *)&cmd_cobboard_setmode2,
	(parse_pgm_inst_t *)&cmd_cobboard_setmode3,
	(parse_pgm_inst_t *)&cmd_ballboard_show,
	(parse_pgm_inst_t *)&cmd_ballboard_setmode1,
	(parse_pgm_inst_t *)&cmd_ballboard_setmode2,
	(parse_pgm_inst_t *)&cmd_ballboard_setmode3,
	(parse_pgm_inst_t *)&cmd_servo_balls,
	(parse_pgm_inst_t *)&cmd_clitoid,
	(parse_pgm_inst_t *)&cmd_time_monitor,
	(parse_pgm_inst_t *)&cmd_strat_event,
	(parse_pgm_inst_t *)&cmd_test,
	(parse_pgm_inst_t *)&cmd_climb,

	/* commands_traj.c */
	(parse_pgm_inst_t *)&cmd_traj_speed,
	(parse_pgm_inst_t *)&cmd_traj_speed_show,
	(parse_pgm_inst_t *)&cmd_traj_acc,
	(parse_pgm_inst_t *)&cmd_traj_acc_show,
	(parse_pgm_inst_t *)&cmd_trajectory,
	(parse_pgm_inst_t *)&cmd_trajectory_show,
	(parse_pgm_inst_t *)&cmd_circle_coef,
	(parse_pgm_inst_t *)&cmd_circle_coef_show,
	(parse_pgm_inst_t *)&cmd_rs_gains,
	(parse_pgm_inst_t *)&cmd_rs_gains_show,
	(parse_pgm_inst_t *)&cmd_track,
	(parse_pgm_inst_t *)&cmd_track_show,
	(parse_pgm_inst_t *)&cmd_pt_list,
	(parse_pgm_inst_t *)&cmd_pt_list_append,
	(parse_pgm_inst_t *)&cmd_pt_list_del,
	(parse_pgm_inst_t *)&cmd_pt_list_show,
	(parse_pgm_inst_t *)&cmd_goto1,
	(parse_pgm_inst_t *)&cmd_goto2,
	(parse_pgm_inst_t *)&cmd_position,
	(parse_pgm_inst_t *)&cmd_position_set,
	(parse_pgm_inst_t *)&cmd_strat_db,
	(parse_pgm_inst_t *)&cmd_strat_conf,
	(parse_pgm_inst_t *)&cmd_strat_conf2,
	(parse_pgm_inst_t *)&cmd_strat_conf3,
	(parse_pgm_inst_t *)&cmd_subtraj,
	NULL,
};
