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
 *  Revision : $Id: cs.c,v 1.4 2009-05-27 20:04:07 zer0 Exp $
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

int32_t encoders_spi_update_roller_speed(void *number)
{
	static volatile int32_t roller_pos;
	int32_t tmp, speed;
	tmp = encoders_spi_get_value(number);
	speed = tmp - roller_pos;
	roller_pos = tmp;
	return speed;
}

/* called every 5 ms */
static void do_cs(void *dummy)
{
	/* read encoders */
	if (ballboard.flags & DO_ENCODERS) {
		encoders_spi_manage(NULL);
	}
	/* control system */
	if (ballboard.flags & DO_CS) {
		if (ballboard.roller.on)
			cs_manage(&ballboard.roller.cs);
		if (ballboard.forktrans.on)
			cs_manage(&ballboard.forktrans.cs);
		if (ballboard.forkrot.on)
			cs_manage(&ballboard.forkrot.cs);
	}
	if (ballboard.flags & DO_BD) {
		bd_manage_from_cs(&ballboard.roller.bd, &ballboard.roller.cs);
		bd_manage_from_cs(&ballboard.forktrans.bd, &ballboard.forktrans.cs);
		bd_manage_from_cs(&ballboard.forkrot.bd, &ballboard.forkrot.cs);
	}
	if (ballboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
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
	/* ---- CS roller */
	/* PID */
	pid_init(&ballboard.roller.pid);
	pid_set_gains(&ballboard.roller.pid, 80, 80, 250);
	pid_set_maximums(&ballboard.roller.pid, 0, 10000, 2000);
	pid_set_out_shift(&ballboard.roller.pid, 6);
	pid_set_derivate_filter(&ballboard.roller.pid, 6);

	/* CS */
	cs_init(&ballboard.roller.cs);
	cs_set_correct_filter(&ballboard.roller.cs, pid_do_filter, &ballboard.roller.pid);
	cs_set_process_in(&ballboard.roller.cs, pwm_ng_set, ROLLER_PWM);
	cs_set_process_out(&ballboard.roller.cs, encoders_spi_update_roller_speed, ROLLER_ENCODER);
	cs_set_consign(&ballboard.roller.cs, 0);

	/* ---- CS forktrans */
	/* PID */
	pid_init(&ballboard.forktrans.pid);
	pid_set_gains(&ballboard.forktrans.pid, 200, 5, 250);
	pid_set_maximums(&ballboard.forktrans.pid, 0, 10000, 2047);
	pid_set_out_shift(&ballboard.forktrans.pid, 6);
	pid_set_derivate_filter(&ballboard.forktrans.pid, 6);

	/* QUADRAMP */
	quadramp_init(&ballboard.forktrans.qr);
	quadramp_set_1st_order_vars(&ballboard.forktrans.qr, 200, 200); /* set speed */
	quadramp_set_2nd_order_vars(&ballboard.forktrans.qr, 20, 20); /* set accel */

	/* CS */
	cs_init(&ballboard.forktrans.cs);
	cs_set_consign_filter(&ballboard.forktrans.cs, quadramp_do_filter, &ballboard.forktrans.qr);
	cs_set_correct_filter(&ballboard.forktrans.cs, pid_do_filter, &ballboard.forktrans.pid);
	cs_set_process_in(&ballboard.forktrans.cs, pwm_ng_set, FORKTRANS_PWM);
	cs_set_process_out(&ballboard.forktrans.cs, encoders_spi_get_value, FORKTRANS_ENCODER);
	cs_set_consign(&ballboard.forktrans.cs, 0);

	/* Blocking detection */
	bd_init(&ballboard.forktrans.bd);
	bd_set_speed_threshold(&ballboard.forktrans.bd, 150);
	bd_set_current_thresholds(&ballboard.forktrans.bd, 500, 8000, 1000000, 40);

	/* ---- CS forkrot */
	/* PID */
	pid_init(&ballboard.forkrot.pid);
	pid_set_gains(&ballboard.forkrot.pid, 200, 5, 250);
	pid_set_maximums(&ballboard.forkrot.pid, 0, 10000, 2047);
	pid_set_out_shift(&ballboard.forkrot.pid, 6);
	pid_set_derivate_filter(&ballboard.forkrot.pid, 6);

	/* QUADRAMP */
	quadramp_init(&ballboard.forkrot.qr);
	quadramp_set_1st_order_vars(&ballboard.forkrot.qr, 200, 200); /* set speed */
	quadramp_set_2nd_order_vars(&ballboard.forkrot.qr, 20, 20); /* set accel */

	/* CS */
	cs_init(&ballboard.forkrot.cs);
	cs_set_consign_filter(&ballboard.forkrot.cs, quadramp_do_filter, &ballboard.forkrot.qr);
	cs_set_correct_filter(&ballboard.forkrot.cs, pid_do_filter, &ballboard.forkrot.pid);
	cs_set_process_in(&ballboard.forkrot.cs, pwm_ng_set, FORKROT_PWM);
	cs_set_process_out(&ballboard.forkrot.cs, encoders_spi_get_value, FORKROT_ENCODER);
	cs_set_consign(&ballboard.forkrot.cs, 0);

	/* Blocking detection */
	bd_init(&ballboard.forkrot.bd);
	bd_set_speed_threshold(&ballboard.forkrot.bd, 150);
	bd_set_current_thresholds(&ballboard.forkrot.bd, 500, 8000, 1000000, 40);

	/* set them on !! */
	ballboard.roller.on = 0;
	ballboard.forktrans.on = 1;
	ballboard.forkrot.on = 1;


	scheduler_add_periodical_event_priority(do_cs, NULL,
						5000L / SCHEDULER_UNIT, 
						CS_PRIO);
}
