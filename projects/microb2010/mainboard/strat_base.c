/*
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: strat_base.c,v 1.8 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

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

#include "../common/i2c_commands.h"

#include "main.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat.h"
#include "sensor.h"
#include "i2c_protocol.h"

/* true if we want to interrupt a trajectory */
static uint8_t traj_intr=0;

/* filled when a END_OBSTACLE is returned */
struct opponent_obstacle opponent_obstacle;

/* asked speed */
static volatile int16_t strat_speed_a = SPEED_DIST_FAST;
static volatile int16_t strat_speed_d = SPEED_ANGLE_FAST;
static volatile uint16_t strat_limit_speed_a = 0; /* no limit */
static volatile uint16_t strat_limit_speed_d = 0;

static volatile uint8_t strat_limit_speed_enabled = 1;


/* Strings that match the end traj cause */
/* /!\ keep it sync with stat_base.h */
const char *err_tab []= {
	"END_TRAJ",
	"END_BLOCKING",
	"END_NEAR",
	"END_OBSTACLE",
	"END_ERROR",
	"END_INTR",
	"END_TIMER",
	"END_RESERVED",
};

/* return string from end traj type num */
const char *get_err(uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i=0 ; i<8; i++) {
		if (err & (1 <<i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}

void strat_hardstop(void)
{
	DEBUG(E_USER_STRAT, "strat_hardstop");

	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);

	while ((ABS(mainboard.speed_d) > 200) ||
	       (ABS(mainboard.speed_a) > 200))

	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);
}

/* go to an x,y point without checking for obstacle or blocking. It
 * should be used for very small dist only. Return END_TRAJ if we
 * reach destination, or END_BLOCKING if the robot blocked more than 3
 * times. */
uint8_t strat_goto_xy_force(int16_t x, int16_t y)
{
	uint8_t i, err;

	for (i=0; i<3; i++) {
		trajectory_goto_xy_abs(&mainboard.traj, x, y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (TRAJ_SUCCESS(err))
			return END_TRAJ;
		if (err == END_BLOCKING) {
			time_wait_ms(500);
			strat_hardstop();
		}
	}
	return END_BLOCKING;
}

/* reset position */
void strat_reset_pos(int16_t x, int16_t y, int16_t a)
{
	int16_t posx = position_get_x_s16(&mainboard.pos);
	int16_t posy = position_get_y_s16(&mainboard.pos);
	int16_t posa = position_get_a_deg_s16(&mainboard.pos);

	if (x == DO_NOT_SET_POS)
		x = posx;
	if (y == DO_NOT_SET_POS)
		y = posy;
	if (a == DO_NOT_SET_POS)
		a = posa;

	DEBUG(E_USER_STRAT, "reset pos (%s%s%s)",
	      x == DO_NOT_SET_POS ? "" : "x",
	      y == DO_NOT_SET_POS ? "" : "y",
	      a == DO_NOT_SET_POS ? "" : "a");
	position_set(&mainboard.pos, x, y, a);
	DEBUG(E_USER_STRAT, "pos resetted", __FUNCTION__);
}

/*
 * decrease gain on angle PID, and go forward until we reach the
 * border.
 */
uint8_t strat_calib(int16_t dist, uint8_t flags)
{
	int32_t p = pid_get_gain_P(&mainboard.angle.pid);
	int32_t i = pid_get_gain_I(&mainboard.angle.pid);
	int32_t d = pid_get_gain_D(&mainboard.angle.pid);
	int32_t max_in = pid_get_max_in(&mainboard.angle.pid);
	int32_t max_i = pid_get_max_I(&mainboard.angle.pid);
	int32_t max_out = pid_get_max_out(&mainboard.angle.pid);
	uint8_t err;

	pid_set_maximums(&mainboard.distance.pid, 0, 20000, 1000);
	pid_set_gains(&mainboard.angle.pid, 200, 0, 2000);
	trajectory_d_rel(&mainboard.traj, dist);
	err = wait_traj_end(flags);
	pid_set_gains(&mainboard.angle.pid, p, i, d);
	pid_set_maximums(&mainboard.distance.pid, max_in, max_i, max_out);
	return err;
}

static void strat_update_traj_speed(void)
{
	uint16_t d, a;

	d = strat_speed_d;
	if (strat_limit_speed_d && d > strat_limit_speed_d)
		d = strat_limit_speed_d;
	a = strat_speed_a;
	if (strat_limit_speed_a && a > strat_limit_speed_a)
		a = strat_limit_speed_a;

	trajectory_set_speed(&mainboard.traj, d, a);
}

void strat_set_speed(uint16_t d, uint16_t a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_speed_d = d;
	strat_speed_a = a;
	strat_update_traj_speed();
	IRQ_UNLOCK(flags);
}

void strat_set_acc(double d, double a)
{
	trajectory_set_acc(&mainboard.traj, d, a);
}

void strat_get_speed(uint16_t *d, uint16_t *a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*d = strat_speed_d;
	*a = strat_speed_a;
	IRQ_UNLOCK(flags);
}

void strat_limit_speed_enable(void)
{
	strat_limit_speed_enabled = 1;
}

void strat_limit_speed_disable(void)
{
	strat_limit_speed_enabled = 0;
}

/* called periodically */
void strat_limit_speed(void)
{
	uint16_t lim_d = 0, lim_a = 0;
	int16_t opp_d, opp_a;

	if (strat_limit_speed_enabled == 0)
		goto update;

	if (get_opponent_da(&opp_d, &opp_a) != 0)
		goto update;

	if (opp_d < 500) {
		if (mainboard.speed_d > 0 && (opp_a > 290 || opp_a < 70)) {
			lim_d = SPEED_DIST_VERY_SLOW;
			lim_a = SPEED_ANGLE_VERY_SLOW;
		}
		else if (mainboard.speed_d < 0 && (opp_a < 250 && opp_a > 110)) {
			lim_d = SPEED_DIST_VERY_SLOW;
			lim_a = SPEED_ANGLE_VERY_SLOW;
		}
		else {
			lim_d = SPEED_DIST_SLOW;
			lim_a = SPEED_ANGLE_VERY_SLOW;
		}
	}
	else if (opp_d < 800) {
		if (mainboard.speed_d > 0 && (opp_a > 290 || opp_a < 70)) {
			lim_d = SPEED_DIST_SLOW;
			lim_a = SPEED_ANGLE_SLOW;
		}
		else if (mainboard.speed_d < 0 && (opp_a < 250 && opp_a > 110)) {
			lim_d = SPEED_DIST_SLOW;
			lim_a = SPEED_ANGLE_SLOW;
		}
	}

 update:
	if (lim_d != strat_limit_speed_d ||
	    lim_a != strat_limit_speed_a) {
		strat_limit_speed_d = lim_d;
		strat_limit_speed_a = lim_a;
		DEBUG(E_USER_STRAT, "new speed limit da=%d,%d", lim_d, lim_a);
		strat_update_traj_speed();
	}
}

/* start the strat */
void strat_start(void)
{
	uint8_t err;

	strat_preinit();

#ifndef HOST_VERSION
	/* if start sw not plugged */
	if (sensor_get(S_START_SWITCH)) {
		int8_t i;

		printf_P(PSTR("No start switch, press a key or plug it\r\n"));

		/* while start sw not plugged */
		while (sensor_get(S_START_SWITCH)) {
			if (! cmdline_keypressed())
				continue;

			for (i=3; i>0; i--) {
				printf_P(PSTR("%d\r\n"), i);
				time_wait_ms(1000);
			}
			break;
		}
	}

	/* if start sw plugged */
	if (!sensor_get(S_START_SWITCH)) {
		printf_P(PSTR("Ready, unplug start switch to start\r\n"));
		/* while start sw plugged */
		while (!sensor_get(S_START_SWITCH));
	}
#endif

	strat_init();
	err = strat_main();
	NOTICE(E_USER_STRAT, "Finished !! returned %s", get_err(err));
	strat_exit();
}

/* return true if we have to brake due to an obstacle */
uint8_t strat_obstacle(void)
{
#if 0
	int16_t x_rel, y_rel;
	int16_t opp_x, opp_y, opp_d, opp_a;

	/* too slow */
	if (ABS(mainboard.speed_d) < 150)
		return 0;

	/* no opponent detected */
	if (get_opponent_xyda(&opp_x, &opp_y,
			      &opp_d, &opp_a))
		return 0;

	/* save obstacle position */
	opponent_obstacle.x = opp_x;
	opponent_obstacle.y = opp_y;
	opponent_obstacle.d = opp_d;
	opponent_obstacle.a = opp_a;
#else /* belgium cup only */
	int16_t x_rel, y_rel;
	int16_t opp_d, opp_a;
	double opp_x, opp_y;

#ifdef HOST_VERSION
	return 0;
	if (time_get_s() >= 12 && time_get_s() <= 30)
		return 1;
#endif
	return 0; /* XXX disabled */

	if (!sensor_get(S_RCOB_WHITE))
		return 0;

	opp_a = 0;
	opp_d = 300;

	rel_da_to_abs_xy(opp_d, RAD(opp_a), &opp_x, &opp_y);
	if (!is_in_area(opp_x, opp_y, 250))
		return 0;
#endif

	/* sensor are temporarily disabled */
	if (sensor_obstacle_is_disabled())
		return 0;

	/* relative position */
	x_rel = cos(RAD(opp_a)) * (double)opp_d;
	y_rel = sin(RAD(opp_a)) * (double)opp_d;

	/* opponent too far */
	if (opp_d > 600)
		return 0;

	/* opponent is in front of us */
	if (mainboard.speed_d > 0 && (opp_a > 325 || opp_a < 35)) {
		DEBUG(E_USER_STRAT, "opponent front d=%d, a=%d "
		      "xrel=%d yrel=%d (speed_d=%d)",
		      opp_d, opp_a, x_rel, y_rel, mainboard.speed_d);
		sensor_obstacle_disable();
		return 1;
	}
	/* opponent is behind us */
	if (mainboard.speed_d < 0 && (opp_a < 215 && opp_a > 145)) {
		DEBUG(E_USER_STRAT, "opponent behind d=%d, a=%d xrel=%d yrel=%d",
		      opp_d, opp_a, x_rel, y_rel);
		sensor_obstacle_disable();
		return 1;
	}

	return 0;
}

void interrupt_traj(void)
{
	traj_intr = 1;
}

void interrupt_traj_reset(void)
{
	traj_intr = 0;
}

uint8_t test_traj_end(uint8_t why)
{
	uint16_t cur_timer;
	point_t robot_pt;

	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);

	/* trigger an event at 3 sec before the end of the match if we
	 * have balls in the barrel */
	cur_timer = time_get_s();

	if ((mainboard.flags & DO_TIMER) && (why & END_TIMER)) {
		/* end of match */
		if (cur_timer >= MATCH_TIME)
			return END_TIMER;
	}

	if ((why & END_INTR) && traj_intr) {
		interrupt_traj_reset();
		return END_INTR;
	}

	if ((why & END_TRAJ) && trajectory_finished(&mainboard.traj))
		return END_TRAJ;

	/* we are near the destination point (depends on current
	 * speed) AND the robot is in the area bounding box. */
	if (why & END_NEAR) {
		int16_t d_near = 100;

		if (mainboard.speed_d > 2000)
			d_near = 150;

		if (trajectory_in_window(&mainboard.traj, d_near, RAD(5.0)) &&
		    is_in_boundingbox(&robot_pt))
			return END_NEAR;
	}

	if ((why & END_BLOCKING) && bd_get(&mainboard.angle.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	if ((why & END_BLOCKING) && bd_get(&mainboard.distance.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	if ((why & END_OBSTACLE) && strat_obstacle()) {
		strat_hardstop();
		return END_OBSTACLE;
	}

	return 0;
}

uint8_t __wait_traj_end_debug(uint8_t why, uint16_t line)
{
	uint8_t ret = 0;
	int16_t opp_x, opp_y, opp_d, opp_a;

	while (ret == 0)
		ret = test_traj_end(why);

	if (ret == END_OBSTACLE) {
		if (get_opponent_xyda(&opp_x, &opp_y,
				      &opp_d, &opp_a) == 0)
			DEBUG(E_USER_STRAT, "Got %s at line %d"
			      " xy=(%d,%d) da=(%d,%d)", get_err(ret),
			      line, opp_x, opp_y, opp_d, opp_a);
	}
	else {
		DEBUG(E_USER_STRAT, "Got %s at line %d",
		      get_err(ret), line);
	}
	return ret;
}
