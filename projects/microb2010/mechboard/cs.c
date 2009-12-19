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
#include <time.h>
#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "actuator.h"

/* called every 5 ms */
static void do_cs(__attribute__((unused)) void *dummy) 
{
	/* read encoders */
	if (mechboard.flags & DO_ENCODERS) {
		encoders_spi_manage(NULL);
	}
	/* control system */
	if (mechboard.flags & DO_CS) {
		if (mechboard.left_arm.on)
			cs_manage(&mechboard.left_arm.cs);
		if (mechboard.right_arm.on)
			cs_manage(&mechboard.right_arm.cs);
	}
	if (mechboard.flags & DO_BD) {
		bd_manage_from_cs(&mechboard.left_arm.bd, &mechboard.left_arm.cs);
		bd_manage_from_cs(&mechboard.right_arm.bd, &mechboard.right_arm.cs);
	}
	if (mechboard.flags & DO_POWER)
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
	/* ---- CS left_arm */
	/* PID */
	pid_init(&mechboard.left_arm.pid);
	pid_set_gains(&mechboard.left_arm.pid, 500, 40, 5000);
	pid_set_maximums(&mechboard.left_arm.pid, 0, 5000, 2400); /* max is 12 V */
	pid_set_out_shift(&mechboard.left_arm.pid, 10);
	pid_set_derivate_filter(&mechboard.left_arm.pid, 4);

	/* QUADRAMP */
	quadramp_init(&mechboard.left_arm.qr);
	quadramp_set_1st_order_vars(&mechboard.left_arm.qr, 2000, 2000); /* set speed */
	quadramp_set_2nd_order_vars(&mechboard.left_arm.qr, 20, 20); /* set accel */

	/* CS */
	cs_init(&mechboard.left_arm.cs);
	cs_set_consign_filter(&mechboard.left_arm.cs, quadramp_do_filter, &mechboard.left_arm.qr);
	cs_set_correct_filter(&mechboard.left_arm.cs, pid_do_filter, &mechboard.left_arm.pid);
	cs_set_process_in(&mechboard.left_arm.cs, pwm_ng_set, LEFT_ARM_PWM);
	cs_set_process_out(&mechboard.left_arm.cs, encoders_spi_get_value, LEFT_ARM_ENCODER);
	cs_set_consign(&mechboard.left_arm.cs, 0);

	/* Blocking detection */
	bd_init(&mechboard.left_arm.bd);
	bd_set_speed_threshold(&mechboard.left_arm.bd, 150);
	bd_set_current_thresholds(&mechboard.left_arm.bd, 500, 8000, 1000000, 40);

	/* ---- CS right_arm */
	/* PID */
	pid_init(&mechboard.right_arm.pid);
	pid_set_gains(&mechboard.right_arm.pid, 500, 40, 5000);
	pid_set_maximums(&mechboard.right_arm.pid, 0, 5000, 2400); /* max is 12 V */
	pid_set_out_shift(&mechboard.right_arm.pid, 10);
	pid_set_derivate_filter(&mechboard.right_arm.pid, 6);

	/* QUADRAMP */
	quadramp_init(&mechboard.right_arm.qr);
	quadramp_set_1st_order_vars(&mechboard.right_arm.qr, 1000, 1000); /* set speed */
	quadramp_set_2nd_order_vars(&mechboard.right_arm.qr, 20, 20); /* set accel */

	/* CS */
	cs_init(&mechboard.right_arm.cs);
	cs_set_consign_filter(&mechboard.right_arm.cs, quadramp_do_filter, &mechboard.right_arm.qr);
	cs_set_correct_filter(&mechboard.right_arm.cs, pid_do_filter, &mechboard.right_arm.pid);
	cs_set_process_in(&mechboard.right_arm.cs, pwm_ng_set, RIGHT_ARM_PWM);
	cs_set_process_out(&mechboard.right_arm.cs, encoders_spi_get_value, RIGHT_ARM_ENCODER);
	cs_set_consign(&mechboard.right_arm.cs, 0);

	/* Blocking detection */
	bd_init(&mechboard.right_arm.bd);
	bd_set_speed_threshold(&mechboard.right_arm.bd, 150);
	bd_set_current_thresholds(&mechboard.right_arm.bd, 500, 8000, 1000000, 40);

	/* set them on !! */
	mechboard.left_arm.on = 1;
	mechboard.right_arm.on = 1;


	scheduler_add_periodical_event_priority(do_cs, NULL, 
						CS_PERIOD / SCHEDULER_UNIT, 
						CS_PRIO);
}
