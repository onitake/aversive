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
 *  Revision : $Id: commands.c,v 1.6 2009-11-08 17:25:00 zer0 Exp $
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
extern parse_pgm_inst_t cmd_ax12_stress;
extern parse_pgm_inst_t cmd_ax12_dump_stats;

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

/* commands_mechboard.c */
extern parse_pgm_inst_t cmd_event;
extern parse_pgm_inst_t cmd_color;
extern parse_pgm_inst_t cmd_arm_show;
extern parse_pgm_inst_t cmd_arm_goto;
extern parse_pgm_inst_t cmd_arm_goto_fixed;
extern parse_pgm_inst_t cmd_arm_simulate;
extern parse_pgm_inst_t cmd_finger;
extern parse_pgm_inst_t cmd_pump;
extern parse_pgm_inst_t cmd_state1;
extern parse_pgm_inst_t cmd_state2;
extern parse_pgm_inst_t cmd_state3;
extern parse_pgm_inst_t cmd_state4;
extern parse_pgm_inst_t cmd_state5;
extern parse_pgm_inst_t cmd_state_debug;
extern parse_pgm_inst_t cmd_state_machine;
extern parse_pgm_inst_t cmd_servo_lintel;
extern parse_pgm_inst_t cmd_pump_current;
extern parse_pgm_inst_t cmd_manivelle;
extern parse_pgm_inst_t cmd_test;


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
	(parse_pgm_inst_t *)&cmd_ax12_stress,
	(parse_pgm_inst_t *)&cmd_ax12_dump_stats,

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

	/* commands_mechboard.c */
	(parse_pgm_inst_t *)&cmd_event,
	(parse_pgm_inst_t *)&cmd_color,
	(parse_pgm_inst_t *)&cmd_arm_show,
	(parse_pgm_inst_t *)&cmd_arm_goto,
	(parse_pgm_inst_t *)&cmd_arm_goto_fixed,
	(parse_pgm_inst_t *)&cmd_arm_simulate,
	(parse_pgm_inst_t *)&cmd_finger,
	(parse_pgm_inst_t *)&cmd_pump,
	(parse_pgm_inst_t *)&cmd_state1,
	(parse_pgm_inst_t *)&cmd_state2,
	(parse_pgm_inst_t *)&cmd_state3,
	(parse_pgm_inst_t *)&cmd_state4,
	(parse_pgm_inst_t *)&cmd_state5,
	(parse_pgm_inst_t *)&cmd_state_debug,
	(parse_pgm_inst_t *)&cmd_state_machine,
	(parse_pgm_inst_t *)&cmd_servo_lintel,
	(parse_pgm_inst_t *)&cmd_pump_current,
	(parse_pgm_inst_t *)&cmd_manivelle,
	(parse_pgm_inst_t *)&cmd_test,

	NULL,
};
