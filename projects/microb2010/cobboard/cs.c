/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: cs.c,v 1.4 2009-04-24 19:30:42 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <ax12.h>
#include <spi.h>
#include <encoders_spi.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>
#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "actuator.h"
#include "spickle.h"

/* called every 5 ms */
static void do_cs(__attribute__((unused)) void *dummy)
{
	/* read encoders */
	if (cobboard.flags & DO_ENCODERS) {
		encoders_spi_manage(NULL);
	}

	/* control system */
	if (cobboard.flags & DO_CS) {
		if (cobboard.left_spickle.on)
			cs_manage(&cobboard.left_spickle.cs);
		if (cobboard.right_spickle.on)
			cs_manage(&cobboard.right_spickle.cs);
		if (cobboard.shovel.on)
			cs_manage(&cobboard.shovel.cs);
	}

	if ((cobboard.flags & DO_BD) && (cobboard.flags & DO_POWER)) {
		bd_manage_from_cs(&cobboard.left_spickle.bd, &cobboard.left_spickle.cs);
		bd_manage_from_cs(&cobboard.right_spickle.bd, &cobboard.right_spickle.cs);
		bd_manage_from_cs(&cobboard.shovel.bd, &cobboard.shovel.cs);

		/* urgent case: stop power on blocking */
		if (cobboard.flags & DO_ERRBLOCKING) {
/* 			if (bd_get(&cobboard.left_spickle.bd) || */
/* 			    bd_get(&cobboard.right_spickle.bd) || */
/* 			    bd_get(&cobboard.shovel.bd)) { */
/* 				printf_P(PSTR("MOTOR BLOCKED STOP ALL\r\n")); */
/* 				cobboard.flags &= ~(DO_POWER | DO_ERRBLOCKING); */
/* 			} */
		}
	}
	if (cobboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
}

void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons=% .5ld fcons=% .5ld err=% .5ld "
	      "in=% .5ld out=% .5ld",
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs),
	      cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"),
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void microb_cs_init(void)
{
	/* ---- CS left_spickle */
	/* PID */
	pid_init(&cobboard.left_spickle.pid);
	pid_set_gains(&cobboard.left_spickle.pid, 400, 10, 1500);
	pid_set_maximums(&cobboard.left_spickle.pid, 0, 25000, 4095);
	pid_set_out_shift(&cobboard.left_spickle.pid, 10);
	pid_set_derivate_filter(&cobboard.left_spickle.pid, 4);

	/* CS */
	cs_init(&cobboard.left_spickle.cs);
	cs_set_correct_filter(&cobboard.left_spickle.cs, pid_do_filter, &cobboard.left_spickle.pid);
	cs_set_process_in(&cobboard.left_spickle.cs, spickle_set, LEFT_SPICKLE_PWM);
	cs_set_process_out(&cobboard.left_spickle.cs, encoders_spi_get_value, LEFT_SPICKLE_ENCODER);
	cs_set_consign(&cobboard.left_spickle.cs, 0);

	/* Blocking detection */
	bd_init(&cobboard.left_spickle.bd);
	bd_set_speed_threshold(&cobboard.left_spickle.bd, 150);
	bd_set_current_thresholds(&cobboard.left_spickle.bd, 500, 8000, 1000000, 200);

	/* ---- CS right_spickle */
	/* PID */
	pid_init(&cobboard.right_spickle.pid);
	pid_set_gains(&cobboard.right_spickle.pid, 400, 10, 1500);
	pid_set_maximums(&cobboard.right_spickle.pid, 0, 25000, 4095);
	pid_set_out_shift(&cobboard.right_spickle.pid, 10);
	pid_set_derivate_filter(&cobboard.right_spickle.pid, 4);

	/* CS */
	cs_init(&cobboard.right_spickle.cs);
	cs_set_correct_filter(&cobboard.right_spickle.cs, pid_do_filter, &cobboard.right_spickle.pid);
	cs_set_process_in(&cobboard.right_spickle.cs, spickle_set, RIGHT_SPICKLE_PWM);
	cs_set_process_out(&cobboard.right_spickle.cs, encoders_spi_get_value, RIGHT_SPICKLE_ENCODER);
	cs_set_consign(&cobboard.right_spickle.cs, 0);

	/* Blocking detection */
	bd_init(&cobboard.right_spickle.bd);
	bd_set_speed_threshold(&cobboard.right_spickle.bd, 150);
	bd_set_current_thresholds(&cobboard.right_spickle.bd, 500, 8000, 1000000, 200);

	/* ---- CS shovel */
	/* PID */
	pid_init(&cobboard.shovel.pid);
	pid_set_gains(&cobboard.shovel.pid, 1000, 10, 1400);
	pid_set_maximums(&cobboard.shovel.pid, 0, 10000, 3200); /* max is 18 V */
	pid_set_out_shift(&cobboard.shovel.pid, 10);
	pid_set_derivate_filter(&cobboard.shovel.pid, 4);

	/* quadramp */
	quadramp_init(&cobboard.shovel.qr);
	quadramp_set_1st_order_vars(&cobboard.shovel.qr, 2500, 2500); /* set speed */
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 50, 20); /* set accel */

	/* CS */
	cs_init(&cobboard.shovel.cs);
	cs_set_consign_filter(&cobboard.shovel.cs, quadramp_do_filter, &cobboard.shovel.qr);
	cs_set_correct_filter(&cobboard.shovel.cs, pid_do_filter, &cobboard.shovel.pid);
	cs_set_process_in(&cobboard.shovel.cs, pwm_ng_set, SHOVEL_PWM);
	cs_set_process_out(&cobboard.shovel.cs, encoders_spi_get_value, SHOVEL_ENCODER);
	cs_set_consign(&cobboard.shovel.cs, 0);

	/* Blocking detection */
	bd_init(&cobboard.shovel.bd);
	bd_set_speed_threshold(&cobboard.shovel.bd, 150);
	bd_set_current_thresholds(&cobboard.shovel.bd, 500, 8000, 1000000, 200);

	/* set them on (or not) !! */
	cobboard.left_spickle.on = 0;
	cobboard.right_spickle.on = 0;
	cobboard.shovel.on = 0;

	scheduler_add_periodical_event_priority(do_cs, NULL,
						CS_PERIOD / SCHEDULER_UNIT,
						CS_PRIO);
}
