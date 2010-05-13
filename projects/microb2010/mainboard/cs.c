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
 *  Revision : $Id: cs.c,v 1.9 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

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
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "robotsim.h"
#include "strat.h"
#include "actuator.h"

void dump_cs(const char *name, struct cs *cs);

#ifndef HOST_VERSION
int32_t encoders_left_cobroller_speed(void *number)
{
	static volatile int32_t roller_pos;
	int32_t tmp, speed;
	tmp = encoders_spi_get_value(number);
	speed = tmp - roller_pos;
	roller_pos = tmp;
	return speed;
}

int32_t encoders_right_cobroller_speed(void *number)
{
	static volatile int32_t roller_pos;
	int32_t tmp, speed;
	tmp = encoders_spi_get_value(number);
	speed = tmp - roller_pos;
	roller_pos = tmp;
	return speed;
}
#endif

/* called every 5 ms */
static void do_cs(void *dummy)
{
	static uint16_t cpt = 0;
	static int32_t old_a = 0, old_d = 0;

#ifdef HOST_VERSION
	robotsim_update();
#else
	/* read encoders */
	if (mainboard.flags & DO_ENCODERS) {
		encoders_spi_manage(NULL);
	}
#endif

	/* robot system, conversion to angle,distance */
	if (mainboard.flags & DO_RS) {
		int16_t a,d;
		rs_update(&mainboard.rs); /* takes about 0.5 ms */
		/* process and store current speed */
		a = rs_get_angle(&mainboard.rs);
		d = rs_get_distance(&mainboard.rs);
		mainboard.speed_a = a - old_a;
		mainboard.speed_d = d - old_d;
		old_a = a;
		old_d = d;
	}

	/* control system */
	if (mainboard.flags & DO_CS) {
		if (mainboard.angle.on)
			cs_manage(&mainboard.angle.cs);
		if (mainboard.distance.on)
			cs_manage(&mainboard.distance.cs);
#ifndef HOST_VERSION
		if (mainboard.left_cobroller.on)
			cs_manage(&mainboard.left_cobroller.cs);
		if (mainboard.right_cobroller.on)
			cs_manage(&mainboard.right_cobroller.cs);
#endif
	}
	if ((cpt & 1) && (mainboard.flags & DO_POS)) {
		/* about 1.5ms (worst case without centrifugal force
		 * compensation) */
		position_manage(&mainboard.pos);
	}
	if ((mainboard.flags & DO_BD) && (mainboard.flags & DO_POWER)) {
		bd_manage_from_cs(&mainboard.angle.bd, &mainboard.angle.cs);
		bd_manage_from_cs(&mainboard.distance.bd, &mainboard.distance.cs);
#ifndef HOST_VERSION
		bd_manage_from_cs(&mainboard.left_cobroller.bd, &mainboard.left_cobroller.cs);
		bd_manage_from_cs(&mainboard.right_cobroller.bd, &mainboard.right_cobroller.cs);
		if (mainboard.flags & DO_ERRBLOCKING) {
/* 			if (bd_get(&mainboard.left_cobroller.bd) || */
/* 			    bd_get(&mainboard.left_cobroller.bd)) { */
/* 				printf_P(PSTR("MOTOR BLOCKED STOP ALL\r\n")); */
/* 				mainboard.flags &= ~(DO_POWER | DO_ERRBLOCKING); */
/* 			} */
		}
#endif
	}
#ifndef HOST_VERSION
	if (mainboard.flags & DO_TIMER) {
		uint8_t second;
		/* the robot should stop correctly in the strat, but
		 * in some cases, we must force the stop from an
		 * interrupt */
		second = time_get_s();
		if (second >= MATCH_TIME + 2) {
			pwm_ng_set(LEFT_PWM, 0);
			pwm_ng_set(RIGHT_PWM, 0);
			printf_P(PSTR("END OF TIME\r\n"));
			while(1);
		}
	}
#endif
	/* brakes */
	if (mainboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
	cpt++;

#ifdef HOST_VERSION
	if ((cpt & 7) == 0) {
		robotsim_dump();
	}
#endif
}

void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons=% .5"PRIi32" fcons=% .5"PRIi32" err=% .5"PRIi32" "
	      "in=% .5"PRIi32" out=% .5"PRIi32"",
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs),
	      cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5"PRIi32" fcons=% .5"PRIi32" err=% .5"PRIi32" "
		      "in=% .5"PRIi32" out=% .5"PRIi32"\r\n"),
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32"\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void microb_cs_init(void)
{
	/* ROBOT_SYSTEM */
	rs_init(&mainboard.rs);
	rs_set_left_pwm(&mainboard.rs, pwm_set_and_save, LEFT_PWM);
	rs_set_right_pwm(&mainboard.rs,  pwm_set_and_save, RIGHT_PWM);
	/* increase gain to decrease dist, increase left and it will turn more left */
#ifdef HOST_VERSION
	rs_set_left_ext_encoder(&mainboard.rs, robotsim_encoder_get,
				LEFT_ENCODER, IMP_COEF * 1.);
	rs_set_right_ext_encoder(&mainboard.rs, robotsim_encoder_get,
				 RIGHT_ENCODER, IMP_COEF * 1.);
#else
	rs_set_left_ext_encoder(&mainboard.rs, encoders_spi_get_value,
				LEFT_ENCODER, IMP_COEF * -1.011718);
	rs_set_right_ext_encoder(&mainboard.rs, encoders_spi_get_value,
				 RIGHT_ENCODER, IMP_COEF * 1.012695);
#endif
	/* rs will use external encoders */
	rs_set_flags(&mainboard.rs, RS_USE_EXT);

	/* POSITION MANAGER */
	position_init(&mainboard.pos);
	position_set_physical_params(&mainboard.pos, VIRTUAL_TRACK_MM, DIST_IMP_MM);
	position_set_related_robot_system(&mainboard.pos, &mainboard.rs);
	position_set_centrifugal_coef(&mainboard.pos, 0.000016);
	position_use_ext(&mainboard.pos);

	/* TRAJECTORY MANAGER */
	trajectory_init(&mainboard.traj, CS_HZ);
	trajectory_set_cs(&mainboard.traj, &mainboard.distance.cs,
			  &mainboard.angle.cs);
	trajectory_set_robot_params(&mainboard.traj, &mainboard.rs, &mainboard.pos);
	trajectory_set_speed(&mainboard.traj, SPEED_DIST_FAST, SPEED_ANGLE_FAST); /* d, a */
	trajectory_set_acc(&mainboard.traj, ACC_DIST, ACC_ANGLE); /* d, a */
	/* distance window, angle window, angle start */
	trajectory_set_windows(&mainboard.traj, 200., 5.0, 30.);

	/* ---- CS angle */
	/* PID */
	pid_init(&mainboard.angle.pid);
	pid_set_gains(&mainboard.angle.pid, 850, 30, 15000);
	pid_set_maximums(&mainboard.angle.pid, 0, 10000, 4095);
	pid_set_out_shift(&mainboard.angle.pid, 10);
	pid_set_derivate_filter(&mainboard.angle.pid, 4);

	/* QUADRAMP */
	quadramp_init(&mainboard.angle.qr);
	quadramp_set_1st_order_vars(&mainboard.angle.qr, 500, 500); /* set speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, 5, 5); /* set accel */

	/* CS */
	cs_init(&mainboard.angle.cs);
	cs_set_consign_filter(&mainboard.angle.cs, quadramp_do_filter, &mainboard.angle.qr);
	cs_set_correct_filter(&mainboard.angle.cs, pid_do_filter, &mainboard.angle.pid);
	cs_set_process_in(&mainboard.angle.cs, rs_set_angle, &mainboard.rs);
	cs_set_process_out(&mainboard.angle.cs, rs_get_angle, &mainboard.rs);
	cs_set_consign(&mainboard.angle.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.angle.bd);
	bd_set_speed_threshold(&mainboard.angle.bd, 80);
	bd_set_current_thresholds(&mainboard.angle.bd, 500, 8000, 1000000, 20);

	/* ---- CS distance */
	/* PID */
	pid_init(&mainboard.distance.pid);
	pid_set_gains(&mainboard.distance.pid, 850, 30, 15000);
	pid_set_maximums(&mainboard.distance.pid, 0, 10000, 4095);
	pid_set_out_shift(&mainboard.distance.pid, 10);
	pid_set_derivate_filter(&mainboard.distance.pid, 6);

	/* QUADRAMP */
	quadramp_init(&mainboard.distance.qr);
	quadramp_set_1st_order_vars(&mainboard.distance.qr, 500, 500); /* set speed */
	quadramp_set_2nd_order_vars(&mainboard.distance.qr, 5., 5.); /* set accel */

	/* CS */
	cs_init(&mainboard.distance.cs);
	cs_set_consign_filter(&mainboard.distance.cs, quadramp_do_filter, &mainboard.distance.qr);
	cs_set_correct_filter(&mainboard.distance.cs, pid_do_filter, &mainboard.distance.pid);
	cs_set_process_in(&mainboard.distance.cs, rs_set_distance, &mainboard.rs);
	cs_set_process_out(&mainboard.distance.cs, rs_get_distance, &mainboard.rs);
	cs_set_consign(&mainboard.distance.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.distance.bd);
	bd_set_speed_threshold(&mainboard.distance.bd, 60);
	bd_set_current_thresholds(&mainboard.distance.bd, 500, 8000, 1000000, 20);

#ifndef HOST_VERSION
	/* ---- CS left_cobroller */
	/* PID */
	pid_init(&mainboard.left_cobroller.pid);
	pid_set_gains(&mainboard.left_cobroller.pid, 80, 10, 10);
	pid_set_maximums(&mainboard.left_cobroller.pid, 0, 30000, 4095);
	pid_set_out_shift(&mainboard.left_cobroller.pid, 5);
	pid_set_derivate_filter(&mainboard.left_cobroller.pid, 6);

	/* CS */
	cs_init(&mainboard.left_cobroller.cs);
	cs_set_correct_filter(&mainboard.left_cobroller.cs, pid_do_filter, &mainboard.left_cobroller.pid);
	cs_set_process_in(&mainboard.left_cobroller.cs, pwm_ng_set, LEFT_COBROLLER_PWM);
	cs_set_process_out(&mainboard.left_cobroller.cs, encoders_left_cobroller_speed, LEFT_COBROLLER_ENCODER);
	cs_set_consign(&mainboard.left_cobroller.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.left_cobroller.bd);
	bd_set_speed_threshold(&mainboard.left_cobroller.bd, 60);
	bd_set_current_thresholds(&mainboard.left_cobroller.bd, 500, 8000, 1000000, 50);

	/* ---- CS right_cobroller */
	/* PID */
	pid_init(&mainboard.right_cobroller.pid);
	pid_set_gains(&mainboard.right_cobroller.pid, 80, 10, 10);
	pid_set_maximums(&mainboard.right_cobroller.pid, 0, 30000, 4095);
	pid_set_out_shift(&mainboard.right_cobroller.pid, 5);
	pid_set_derivate_filter(&mainboard.right_cobroller.pid, 6);

	/* CS */
	cs_init(&mainboard.right_cobroller.cs);
	cs_set_correct_filter(&mainboard.right_cobroller.cs, pid_do_filter, &mainboard.right_cobroller.pid);
	cs_set_process_in(&mainboard.right_cobroller.cs, pwm_ng_set, RIGHT_COBROLLER_PWM);
	cs_set_process_out(&mainboard.right_cobroller.cs, encoders_right_cobroller_speed, RIGHT_COBROLLER_ENCODER);
	cs_set_consign(&mainboard.right_cobroller.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.right_cobroller.bd);
	bd_set_speed_threshold(&mainboard.right_cobroller.bd, 60);
	bd_set_current_thresholds(&mainboard.right_cobroller.bd, 500, 8000, 1000000, 50);
#endif /* !HOST_VERSION */

	/* set them on !! */
	mainboard.angle.on = 1;
	mainboard.distance.on = 1;
	mainboard.left_cobroller.on = 1;
	mainboard.right_cobroller.on = 1;

	scheduler_add_periodical_event_priority(do_cs, NULL,
						5000L / SCHEDULER_UNIT,
						CS_PRIO);

}
