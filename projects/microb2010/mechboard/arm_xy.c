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
 *  Revision : $Id: arm_xy.c,v 1.5 2009-11-08 17:25:00 zer0 Exp $
 *
 * Fabrice DESCLAUX <serpilliere@droids-corp.org>
 * Olivier MATZ <zer0@droids-corp.org>
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <spi.h>
#include <encoders_spi.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>

#include "main.h"
#include "cmdline.h"
#include "arm_xy.h"
#include "ax12_user.h"

#define ARM_DEBUG(args...) DEBUG(E_USER_ARM, args)
#define ARM_NOTICE(args...) NOTICE(E_USER_ARM, args)
#define ARM_ERROR(args...) ERROR(E_USER_ARM, args)

#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

/* physical location/dimensions of arm */
#define ARM_S_LEN 124.
#define ARM_E_LEN 130.

/* timeout after 1 second if position is not reached */
#define ARM_GLOBAL_TIMEOUT 1000000L

/* timeout 100ms after position is reached if not in window */
#define ARM_WINDOW_TIMEOUT 200000L

/* default (template) period, but real one is variable */
#define ARM_PERIOD 50000L
#define ARM_MAX_DIST 40L

/* we pos reached, check arm in window every period */
#define ARM_SURVEY_PERIOD 25000UL /* in us */

/* number of steps/s */
#define ARM_AX12_MAX_SPEED (800L)

/* Maximum number of steps in one ARM_PERIOD */
#define ARM_MAX_E (((ARM_AX12_MAX_SPEED*ARM_PERIOD)/1000000L))
/* 4000 steps/CS => 800step/ms */
#define ARM_MAX_S ((800L*ARM_PERIOD)/1000L)


/* window limits in ax12/cs unit */
#define ARM_SHOULDER_WINDOW_POS  250
#define ARM_ELBOW_WINDOW_POS     8
#define ARM_ELBOW_WINDOW_SPEED   100
#define ARM_WRIST_WINDOW_POS     8
#define ARM_WRIST_WINDOW_SPEED   100

/* default and max speeds */
#define SHOULDER_DEFAULT_SPEED   800
#define ELBOW_DEFAULT_SPEED      0x3ff
#define SHOULDER_MAX_SPEED       10000
#define ELBOW_MAX_SPEED          0x3ff

/* window status flags */
#define SHOULDER_NOT_IN_WIN 1
#define ELBOW_NOT_IN_WIN    2
#define WRIST_NOT_IN_WIN    4

static void wrist_angle_deg2robot_l(double wrist_deg, double *wrist_out);
static void angle_rad2robot_l(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot);
static void angle_robot2rad_l(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad);
static void wrist_angle_deg2robot_r(double wrist_deg, double *wrist_out);
static void angle_rad2robot_r(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot);
static void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad);

static void arm_schedule_event(struct arm *arm, uint32_t time);

struct arm left_arm = {
	.config = {
		.wrist_angle_deg2robot = wrist_angle_deg2robot_l,
		.angle_rad2robot = angle_rad2robot_l,
		.angle_robot2rad = angle_robot2rad_l,
		.elbow_ax12 = L_ELBOW_AX12,
		.wrist_ax12 = L_WRIST_AX12,
	},
};

struct arm right_arm = {
	.config = {
		.wrist_angle_deg2robot = wrist_angle_deg2robot_r,
		.angle_rad2robot = angle_rad2robot_r,
		.angle_robot2rad = angle_robot2rad_r,
		.elbow_ax12 = R_ELBOW_AX12,
		.wrist_ax12 = R_WRIST_AX12,
	},
};

/* process shoulder + elbow angles from height and distance */
int8_t cart2angle(int32_t h_int, int32_t d_int, double *alpha, double *beta)
{
	double h, d, l;
	double elbow, shoulder;

	d = d_int;
	h = h_int;
	l = sqrt(d*d + h*h);
	if  (l > (ARM_S_LEN + ARM_E_LEN))
		return -1;
	
	elbow = -acos((d*d + h*h - ARM_E_LEN*ARM_E_LEN - 
		       ARM_S_LEN*ARM_S_LEN) / (2*ARM_S_LEN*ARM_E_LEN));
	shoulder = atan2(h,d) - atan2(ARM_E_LEN*sin(elbow), 
				      ARM_S_LEN+ARM_E_LEN*cos(elbow));
	
	*alpha = shoulder;
	*beta = elbow;

	return 0;
}


/* process height and distance from shoulder + elbow angles */
void angle2cart(double alpha, double beta, int32_t *h, int32_t *d)
{
	double tmp_a;
	int32_t tmp_h, tmp_d;

	tmp_h = ARM_S_LEN * sin(alpha);
	tmp_d = ARM_S_LEN * cos(alpha);
	
	tmp_a = alpha+beta;
	*h = tmp_h + ARM_E_LEN * sin(tmp_a);
	*d = tmp_d + ARM_E_LEN * cos(tmp_a);
}

/*** left arm */

#define ARM_LEFT_S_OFFSET -1150.
#define ARM_LEFT_E_OFFSET 476.
#define ARM_LEFT_W_OFFSET 90.

static void wrist_angle_deg2robot_l(double wrist_deg, double *wrist_out)
{
	*wrist_out = -wrist_deg * 3.41  + ARM_LEFT_W_OFFSET;
}

/* convert an angle in radian into a robot-specific unit 
 * for shoulder and elbow for LEFT ARM*/
static void angle_rad2robot_l(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot)
{
	*shoulder_robot = shoulder_rad * 4 * 66 * 512. / (2*M_PI) + ARM_LEFT_S_OFFSET;
	*elbow_robot = -elbow_rad * 3.41 * 360. / (2*M_PI) + ARM_LEFT_E_OFFSET;
}

/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow for LEFT ARM */
static void angle_robot2rad_l(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad)
{
	*shoulder_rad = ((shoulder_robot - ARM_LEFT_S_OFFSET) * (2*M_PI))/(4 * 66 * 512.);
	*elbow_rad = -((elbow_robot - ARM_LEFT_E_OFFSET) * (2*M_PI))/(3.41 * 360.);		
}

/*** right arm */

#define ARM_RIGHT_S_OFFSET 1150.
#define ARM_RIGHT_E_OFFSET 673.
#define ARM_RIGHT_W_OFFSET 935.

static void wrist_angle_deg2robot_r(double wrist_deg, double *wrist_out)
{
	*wrist_out = wrist_deg * 3.41  + ARM_RIGHT_W_OFFSET;
}

/* convert an angle in radian into a robot-specific unit 
 * for shoulder and elbow */
static void angle_rad2robot_r(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot)
{
	*shoulder_robot = -shoulder_rad * 4 * 66 * 512. / (2*M_PI) + ARM_RIGHT_S_OFFSET;
	*elbow_robot = elbow_rad * 3.41 * 360. / (2*M_PI) + ARM_RIGHT_E_OFFSET;
}

/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow */
static void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad)
{
	*shoulder_rad = -((shoulder_robot - ARM_RIGHT_S_OFFSET) * (2*M_PI))/(4 * 66 * 512.);
	*elbow_rad = ((elbow_robot - ARM_RIGHT_E_OFFSET) * (2*M_PI))/(3.41 * 360.);
}


/*
 * Fill the arm_status structure according to request.
 *
 * return:
 *     0  => success
 *   < 0  => error
 */
static int8_t arm_do_step(struct arm *arm)
{
	const struct arm_config *conf = &arm->config;
	const struct arm_request *req = &arm->req;
	struct arm_status *status = &arm->status;

	int8_t ret;
	int32_t diff_h, diff_d; /* position delta in steps */
	int32_t next_h, next_d; /* next position in steps */
	int32_t l; /* distance between cur pos and next pos */

	double as_cur_rad, ae_cur_rad; /* current angle in rad */
	double as_next_rad, ae_next_rad; /* next angle in rad */
	double as_cur, ae_cur; /* current angle in angle_steps */
	double as_next, ae_next; /* next angle in angle_steps */

	int32_t as_diff, ae_diff; /* angle delta in angle_steps */
	int32_t s_speed, e_speed; /* elbow/shoulder speed in angle_steps */
	
	double as_coef, ae_coef;
	
	/* process diff between final request and current pos */
	diff_h = req->h_mm - status->h_mm;
	diff_d = req->d_mm - status->d_mm;
	ARM_NOTICE("goal:d=%ld,h=%ld cur:d=%ld,h=%ld diff:d=%ld,h=%ld",
		  req->d_mm, req->h_mm, status->d_mm, status->h_mm,
		  diff_d, diff_h);

	/* if distance to next point is too large, saturate it */
	l = sqrt(diff_h*diff_h + diff_d*diff_d);
	if (l > ARM_MAX_DIST) {
		diff_h = diff_h * ARM_MAX_DIST / l;
		diff_d = diff_d * ARM_MAX_DIST / l;
	}
	ARM_NOTICE("l=%ld ; after max dist: diff:d=%ld,h=%ld", l, diff_d, diff_h);
	
	/* process next position */
	next_h = status->h_mm + diff_h;
	next_d = status->d_mm + diff_d;
	ARM_DEBUG("next:d=%ld,h=%ld", next_d, next_h);

	/* calculate the current angle of arm in radian */
	ret = cart2angle(status->h_mm, status->d_mm, &as_cur_rad, &ae_cur_rad);
	if (ret)
		return ret;
	ARM_DEBUG("as_cur_rad=%f ae_cur_rad=%f", as_cur_rad, ae_cur_rad);

	/* calculate the next angle of arm in radian */
	ret = cart2angle(next_h, next_d, &as_next_rad, &ae_next_rad);
	if (ret)
		return ret;
	ARM_DEBUG("as_next_rad=%f ae_next_rad=%f", as_next_rad, ae_next_rad);

	/* convert radian in angle_steps */
	conf->angle_rad2robot(as_cur_rad, ae_cur_rad,
			     &as_cur, &ae_cur);
	ARM_DEBUG("as_cur=%f ae_cur=%f", as_cur, ae_cur);
	conf->angle_rad2robot(as_next_rad, ae_next_rad,
			     &as_next, &ae_next);
	ARM_DEBUG("as_next=%f ae_next=%f", as_next, ae_next);

	/* process angle delta in angle_steps */
	as_diff = as_next - as_cur;
	ae_diff = ae_next - ae_cur;
	ARM_DEBUG("as_diff=%ld ae_diff=%ld", as_diff, ae_diff);

	/* update position status */
	status->h_mm = next_h;
	status->d_mm = next_d;
	status->shoulder_angle_steps = as_next;
	status->elbow_angle_steps = ae_next;
	status->shoulder_angle_rad = as_next_rad;
	status->elbow_angle_rad = ae_next_rad;

	/* we reached destination, nothing to do */
	if (as_diff == 0 && ae_diff == 0) {
		status->shoulder_speed = SHOULDER_DEFAULT_SPEED;
		status->elbow_speed = ELBOW_DEFAULT_SPEED;
		status->next_update_time = 0;
		ARM_NOTICE("reaching end");
		return 0;
	}

	/* test if one actuator is already in position */
	if (as_diff == 0) {
		ARM_DEBUG("shoulder reached destination");
		ae_coef = (double)ARM_MAX_E / (double)ae_diff;
		status->next_update_time = ARM_PERIOD * ABS(ae_coef);
		e_speed = ABS(ae_coef) * ABS(ae_diff);
		s_speed = ARM_MAX_S;
	}
	else if (ae_diff == 0) {
		ARM_DEBUG("elbow reached destination");
		as_coef = (double)ARM_MAX_S / (double)as_diff;
		status->next_update_time = ARM_PERIOD / ABS(as_coef);
		e_speed = ARM_MAX_E;
		s_speed = ABS(as_coef) * ABS(as_diff);
	}

	else {
		as_coef = (double)ARM_MAX_S / (double)as_diff;
		ae_coef = (double)ARM_MAX_E / (double)ae_diff;
	    
		ARM_DEBUG("as_coef=%f ae_coef=%f", as_coef, ae_coef);
	    
		/* if elbow is limitating */
		if (ABS(as_coef) >= ABS(ae_coef)) {
			ARM_DEBUG("elbow limit");
			status->next_update_time = ARM_PERIOD / ABS(ae_coef);
			s_speed = ABS(ae_coef) * ABS(as_diff);
			e_speed = ABS(ae_coef) * ABS(ae_diff);
		}
		/* else, shoulder is limitating */
		else {
			ARM_DEBUG("shoulder limit");
			status->next_update_time = ARM_PERIOD / ABS(as_coef);
			s_speed = ABS(as_coef) * ABS(as_diff);
			e_speed = ABS(as_coef) * ABS(ae_diff);
		}
	}

	ARM_NOTICE("next update: %ld", status->next_update_time);

	/* convert speed in specific unit */
	status->shoulder_speed = (s_speed * CS_PERIOD) / ARM_PERIOD;
	status->elbow_speed = (e_speed * 0x3ff) / ARM_MAX_E;

	ARM_DEBUG("speeds: s=%ld e=%ld", status->shoulder_speed, status->elbow_speed);

	/* avoid limits */
	if (status->shoulder_speed == 0)
		status->shoulder_speed = 1;
	if (status->elbow_speed == 0)
		status->elbow_speed = 1;
	if (status->shoulder_speed >= SHOULDER_MAX_SPEED)
		status->shoulder_speed = SHOULDER_MAX_SPEED;
	if (status->elbow_speed >= ELBOW_MAX_SPEED)
		status->elbow_speed = ELBOW_MAX_SPEED;

	ARM_DEBUG("speeds (sat): s=%ld e=%ld", status->shoulder_speed, status->elbow_speed);

	return 0;
}

static void arm_delete_event(struct arm *arm)
{
	if (arm->status.event == -1)
		return;
	ARM_DEBUG("Delete arm event");
	scheduler_del_event(arm->status.event);
	arm->status.event = -1;
}

/* write values to ax12 + cs */
static void arm_apply(struct arm *arm)
{
	struct cs_block *csb = arm->config.csb;
	const struct arm_status *st = &arm->status;

	ARM_DEBUG("arm_apply");

	if (arm->config.simulate)
		return;

	/* set speed and pos of shoulder */
	quadramp_set_1st_order_vars(&csb->qr, 
				    st->shoulder_speed,
				    st->shoulder_speed);
	cs_set_consign(&csb->cs, st->shoulder_angle_steps);

	/* set speed and position of elbow */
	ax12_user_write_int(&gen.ax12, arm->config.elbow_ax12,
			    AA_MOVING_SPEED_L, st->elbow_speed);
	ax12_user_write_int(&gen.ax12, arm->config.elbow_ax12,
			    AA_GOAL_POSITION_L, st->elbow_angle_steps);
}

/* return true if one of the mask condition is true */
uint8_t arm_test_traj_end(struct arm *arm, uint8_t mask)
{
	if ((mask & ARM_TRAJ_END) && (arm->status.state & ARM_FLAG_IN_WINDOW))
		return ARM_TRAJ_END;

	if ((mask & ARM_TRAJ_NEAR) && (arm->status.state & ARM_FLAG_LAST_STEP))
		return ARM_TRAJ_NEAR;

	if ((mask & ARM_TRAJ_TIMEOUT) && (arm->status.state & ARM_FLAG_TIMEOUT))
		return ARM_TRAJ_TIMEOUT;

	if ((mask & ARM_TRAJ_ERROR) && (arm->status.state & ARM_FLAG_ERROR))
		return ARM_TRAJ_ERROR;

	return 0;
}

uint8_t arm_wait_traj_end(struct arm *arm, uint8_t mask)
{
	uint8_t ret;
	while(1) {
		ret = arm_test_traj_end(arm, mask);
		if (ret)
			return ret;
	}
}

/* return true if one of the mask condition is true */
uint8_t arm_in_window(struct arm *arm, uint8_t *status)
{
	int8_t err;
/* 	uint16_t spd; */
	int16_t pos;
	int32_t cs_err;

	*status = 0;

	if (arm->config.simulate)
		return 1;

	/* shoulder, just check position */
	cs_err = cs_get_error(&arm->config.csb->cs);
	if (ABS(cs_err) > ARM_SHOULDER_WINDOW_POS)
		*status |= SHOULDER_NOT_IN_WIN;

#if 0	
	/* check elbow speed */
	err = ax12_user_read_int(&gen.ax12, arm->config.elbow_ax12,
				 AA_PRESENT_SPEED_L, &spd);
	if (err)
		goto fail;
	if (spd > ARM_ELBOW_WINDOW_SPEED)
		return 0;

	/* check wrist speed */
	err = ax12_user_read_int(&gen.ax12, arm->config.wrist_ax12,
				 AA_PRESENT_SPEED_L, &spd);
	if (err)
		goto fail;
	if (spd > ARM_WRIST_WINDOW_SPEED)
		return 0;
#endif	
	/* check elbow pos */
	err = ax12_user_read_int(&gen.ax12, arm->config.elbow_ax12,
				 AA_PRESENT_POSITION_L, (uint16_t *)&pos);
	if (err)
		goto fail;
	if (ABS(arm->status.elbow_angle_steps - pos) > ARM_ELBOW_WINDOW_POS)
		*status |= ELBOW_NOT_IN_WIN;

	/* check wrist pos */
	err = ax12_user_read_int(&gen.ax12, arm->config.wrist_ax12,
				 AA_PRESENT_POSITION_L, (uint16_t *)&pos);
	if (err)
		goto fail;
	if (ABS(arm->status.wrist_angle_steps - pos) > ARM_WRIST_WINDOW_POS)
		*status |= WRIST_NOT_IN_WIN;
	
	if (*status)
		return 0;

	ARM_NOTICE("arm is in window (%ld us after reach pos)",
		   time_get_us2() - arm->status.pos_reached_time);
	return 1; /* ok, we are in window */

 fail:
	return 0;
}

/* process wrist pos and apply it. it's done only once. */
static int8_t arm_set_wrist(struct arm *arm)
{
	int8_t err;
	int32_t as_deg, ae_deg, aw_deg;
	uint16_t wrist_out_u16;
	double wrist_out, as_rad, ae_rad;
	int16_t pos;
	uint32_t diff_time;

	/* calculate the destination angle of arm in radian */
	err = cart2angle(arm->req.h_mm, arm->req.d_mm, 
			 &as_rad, &ae_rad);
	if (err)
		return -1;

	/* calc angle destination */
	as_deg = DEG(as_rad);
	ae_deg = DEG(ae_rad);
	ARM_DEBUG("as_dest_deg=%d ae_dest_deg=%d", as_deg, ae_deg);
	aw_deg = as_deg + ae_deg - arm->req.w_deg;
	arm->config.wrist_angle_deg2robot(aw_deg, &wrist_out);
	wrist_out_u16 = wrist_out;

	ARM_DEBUG("set wrist to %ld degrees (%d steps)", aw_deg,
		  wrist_out_u16);

	/* process the theorical reach time for the wrist */
	if (arm->config.simulate) {
		pos = arm->status.wrist_angle_steps;
	}
	else {
		err = ax12_user_read_int(&gen.ax12, arm->config.wrist_ax12,
				    AA_PRESENT_POSITION_L, (uint16_t *)&pos);
		if (err)
			pos = arm->status.wrist_angle_steps;
	}
	/* 600 is the number of steps/s */
	diff_time = (ABS((int16_t)wrist_out_u16 - pos) * 1000000L) / 600;
	arm->status.wrist_reach_time = arm->status.start_time + diff_time;
	ARM_DEBUG("wrist reach time is %ld (diff=%ld)", 
		  arm->status.wrist_reach_time, diff_time);

	/* update current position to destination */
	arm->status.wrist_angle_steps = wrist_out_u16;
	
	if (arm->config.simulate)
		return 0;

	/* send it to ax12 */
	ax12_user_write_int(&gen.ax12, arm->config.wrist_ax12, 
			    AA_GOAL_POSITION_L, wrist_out_u16);
	return 0;
}

/* event callback */
static void arm_do_xy_cb(struct arm *arm)
{
	uint8_t win_status;

	arm->status.event = -1;

	/* if consign haven't reach destination */
	if ((arm->status.state & ARM_FLAG_LAST_STEP) == 0) {
		if (arm_do_step(arm))
			arm->status.state |= ARM_FLAG_ERROR;

		/* it's the first call for the traj */
		if (arm->status.state == ARM_STATE_INIT) {
			arm->status.state |= ARM_FLAG_MOVING;
			if (arm_set_wrist(arm))
				arm->status.state |= ARM_FLAG_ERROR;
		}

		/* we have more steps to do */
		if (arm->status.next_update_time == 0) {
			arm->status.state &= ~ARM_FLAG_MOVING;
			arm->status.state |= ARM_FLAG_LAST_STEP;
			arm->status.pos_reached_time = time_get_us2();
		}
		arm_apply(arm);
	}
	/* last step is reached, we can check that arm is in window */
	else if ((arm->status.state & ARM_FLAG_IN_WINDOW) == 0) {
		if (arm_in_window(arm, &win_status))
			arm->status.state |= ARM_FLAG_IN_WINDOW;
		
		/* check for window arm timeout */
		else {
			microseconds t;
			int32_t diff1, diff2;
			t = time_get_us2();
			diff1 = t - arm->status.pos_reached_time; 
			diff2 = t - arm->status.wrist_reach_time; 
			if (diff1 > ARM_WINDOW_TIMEOUT &&
			    diff2 > ARM_WINDOW_TIMEOUT) {
				ARM_NOTICE("win timeout at %ld win_status=%x",
					   t, win_status);
				arm->status.state |= ARM_FLAG_TIMEOUT;
			}
		}
	}

	/* check for global arm timeout */
	if ((time_get_us2() - arm->status.start_time) > ARM_GLOBAL_TIMEOUT) {
		ARM_NOTICE("global timeout at %ld", time_get_us2());
		arm->status.state |= ARM_FLAG_TIMEOUT;
	}
	
	/* reload event if needed */
	if ((arm->status.state & ARM_FLAG_FINISHED) == ARM_FLAG_FINISHED) {
		ARM_NOTICE("arm traj finished");
		return; /* no more event, position reached */
	}
	if (arm->status.state & (ARM_FLAG_ERROR|ARM_FLAG_TIMEOUT)) {
		ARM_NOTICE("error or timeout");
		return; /* no more event */
	}
	else if (arm->status.state & ARM_FLAG_LAST_STEP) {
		/* theorical position is reached, but reload an event
		 * for position survey (window), every 25ms */
		arm_schedule_event(arm, ARM_SURVEY_PERIOD);
	}
	else {
		/* reload event for next position step */
		arm_schedule_event(arm, arm->status.next_update_time);
	}
}

/* schedule a single event for this arm */
static void arm_schedule_event(struct arm *arm, uint32_t time)
{
	uint8_t flags;
	int8_t ret;

	arm_delete_event(arm);
	if (time < SCHEDULER_UNIT)
		time = SCHEDULER_UNIT;
	IRQ_LOCK(flags);
	ret = scheduler_add_event(SCHEDULER_SINGLE,
				  (void *)arm_do_xy_cb,
				  arm, time/SCHEDULER_UNIT, ARM_PRIO);
	if (ret == -1) {
		IRQ_UNLOCK(flags);
		ARM_ERROR("Cannot load arm event");
		return;
	}
	arm->status.event = ret;
	IRQ_UNLOCK(flags);
}

int8_t arm_do_xy(struct arm *arm, int16_t d_mm, int16_t h_mm, int16_t w_deg)
{
	ARM_NOTICE("arm_do_xy: d_mm=%d h_mm=%d w_deg=%d", d_mm, h_mm, w_deg);

	/* remove previous event if any */
	arm_delete_event(arm);

	/* init mandatory params */
	arm->req.d_mm = d_mm; 
	arm->req.h_mm = h_mm; 
	arm->req.w_deg = w_deg; 
	arm->status.start_time = time_get_us2();
	arm->status.state = ARM_STATE_INIT;

	/* all the job will be done asynchronously now */
	arm_schedule_event(arm, 0);
	return 0;
}

void arm_dump(struct arm *arm)
{
	printf_P(PSTR("config: simulate=%d\r\n"),
		 arm->config.simulate);
	printf_P(PSTR("req: d_mm=%ld h_mm=%ld w_deg=%ld\r\n"),
		 arm->req.d_mm, arm->req.h_mm, arm->req.w_deg);
	printf_P(PSTR("status: "));
	if (arm->status.state == ARM_STATE_INIT)
		printf_P(PSTR("ARM_STATE_INIT "));
	if (arm->status.state & ARM_FLAG_MOVING)
		printf_P(PSTR("ARM_FLAG_MOVING "));
	if (arm->status.state & ARM_FLAG_LAST_STEP)
		printf_P(PSTR("ARM_FLAG_LAST_STEP "));
	if (arm->status.state & ARM_FLAG_IN_WINDOW)
		printf_P(PSTR("ARM_FLAG_IN_WINDOW "));
	if (arm->status.state & ARM_FLAG_ERROR)
		printf_P(PSTR("ARM_FLAG_ERROR "));
	if (arm->status.state & ARM_FLAG_TIMEOUT)
		printf_P(PSTR("ARM_FLAG_TIMEOUT "));
	printf_P(PSTR("\r\n"));

	printf_P(PSTR("   d_mm=%ld h_mm=%ld goal_w_steps=%d\r\n"),
		 arm->status.d_mm, arm->status.h_mm, arm->status.wrist_angle_steps);
	printf_P(PSTR("   cur_shl_steps=%ld cur_elb_steps=%ld\r\n"),
		 arm->status.shoulder_angle_steps, arm->status.elbow_angle_steps);
	printf_P(PSTR("   cur_shl_rad=%f cur_elb_rad=%f\r\n"),
		 arm->status.shoulder_angle_rad, arm->status.elbow_angle_rad);
	printf_P(PSTR("   cur_shl_deg=%f cur_elb_deg=%f\r\n"),
		 DEG(arm->status.shoulder_angle_rad), DEG(arm->status.elbow_angle_rad));
	printf_P(PSTR("   event=%d next_update_time=%ld\r\n"),
		 arm->status.event, arm->status.next_update_time);
	printf_P(PSTR("   start_time=%ld pos_reached_time=%ld wrist_reach_time=%ld\r\n"),
		 arm->status.start_time, arm->status.pos_reached_time,
		 arm->status.wrist_reach_time);
}

#define CALIB_ANGLE (RAD(-93.))

void arm_calibrate(void)
{
	double shoulder, elbow;

	pwm_ng_set(LEFT_ARM_PWM, 500);
	pwm_ng_set(RIGHT_ARM_PWM, -500);
	wait_ms(200);

	pwm_ng_set(LEFT_ARM_PWM, 300);
	pwm_ng_set(RIGHT_ARM_PWM, -300);
	wait_ms(700);

	printf_P(PSTR("Init arm, please wait..."));
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x1);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x04);

	angle_rad2robot_r(0, CALIB_ANGLE, &shoulder, &elbow);
	ax12_user_write_int(&gen.ax12, R_ELBOW_AX12, AA_GOAL_POSITION_L, elbow);
	ax12_user_write_int(&gen.ax12, R_WRIST_AX12, AA_GOAL_POSITION_L, 628);

	angle_rad2robot_l(0, CALIB_ANGLE, &shoulder, &elbow);
	ax12_user_write_int(&gen.ax12, L_ELBOW_AX12, AA_GOAL_POSITION_L, elbow);
	ax12_user_write_int(&gen.ax12, L_WRIST_AX12, AA_GOAL_POSITION_L, 394);
	pwm_ng_set(LEFT_ARM_PWM, -100);
	pwm_ng_set(RIGHT_ARM_PWM, 100);

	wait_ms(2000);

	cs_set_consign(&mechboard.left_arm.cs, 0);
	cs_set_consign(&mechboard.right_arm.cs, 0);
	encoders_spi_set_value(LEFT_ARM_ENCODER, 0);
	encoders_spi_set_value(RIGHT_ARM_ENCODER, 0);

	printf_P(PSTR("ok\r\n"));
}

/* init arm config */
void arm_init(void)
{
	uint32_t shoulder_robot;
	uint16_t elbow_robot, wrist_robot;
	double shoulder_rad, elbow_rad;
	int32_t h, d;
	uint8_t err = 0;

	memset(&left_arm.status, 0, sizeof(left_arm.status));
	memset(&right_arm.status, 0, sizeof(right_arm.status));
	left_arm.status.event = -1;
	right_arm.status.event = -1;

	arm_calibrate();

	/* set des slopes XXX */

	/* set maximum moving speeds */
	err |= ax12_user_write_int(&gen.ax12, L_ELBOW_AX12, AA_MOVING_SPEED_L, 0x3ff);
	err |= ax12_user_write_int(&gen.ax12, L_WRIST_AX12, AA_MOVING_SPEED_L, 0x3ff);
	err |= ax12_user_write_int(&gen.ax12, R_ELBOW_AX12, AA_MOVING_SPEED_L, 0x3ff);
	err |= ax12_user_write_int(&gen.ax12, R_WRIST_AX12, AA_MOVING_SPEED_L, 0x3ff);

	/* left arm init */
	shoulder_robot = encoders_spi_get_value(LEFT_ARM_ENCODER);
	err |= ax12_user_read_int(&gen.ax12, L_ELBOW_AX12, AA_PRESENT_POSITION_L, &elbow_robot);
	err |= ax12_user_read_int(&gen.ax12, L_WRIST_AX12, AA_PRESENT_POSITION_L, &wrist_robot);
	
	angle_robot2rad_l(shoulder_robot, elbow_robot,
			  &shoulder_rad, &elbow_rad);
	angle2cart(shoulder_rad, elbow_rad, &h, &d);
	printf_P(PSTR("left arm: h:%ld d:%ld w:%d\r\n"), h, d, wrist_robot);
	left_arm.status.h_mm = h;
	left_arm.status.d_mm = d;
	left_arm.status.wrist_angle_steps = wrist_robot;
	left_arm.status.state = ARM_FLAG_FINISHED;
	left_arm.config.csb = &mechboard.left_arm;

	/* left arm init */
	shoulder_robot = encoders_spi_get_value(RIGHT_ARM_ENCODER);
	err |= ax12_user_read_int(&gen.ax12, R_ELBOW_AX12, AA_PRESENT_POSITION_L, &elbow_robot);
	err |= ax12_user_read_int(&gen.ax12, R_WRIST_AX12, AA_PRESENT_POSITION_L, &wrist_robot);
	
	angle_robot2rad_r(shoulder_robot, elbow_robot,
			  &shoulder_rad, &elbow_rad);
	angle2cart(shoulder_rad, elbow_rad, &h, &d);
	printf_P(PSTR("right arm: h:%ld d:%ld w:%d\r\n"), h, d, wrist_robot);
	right_arm.status.h_mm = h;
	right_arm.status.d_mm = d;
	right_arm.status.wrist_angle_steps = wrist_robot;
	right_arm.status.state = ARM_FLAG_FINISHED;
	right_arm.config.csb = &mechboard.right_arm;

	if (err)
		ARM_ERROR("ARM INIT ERROR");
}
