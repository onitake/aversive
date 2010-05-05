/*
 *  Copyright Droids, Microb Technology (2010)
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
 *  Revision : $Id: strat.c,v 1.6 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <aversive.h>
#include <aversive/error.h>
#include <aversive/pgmspace.h>

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
#include <trajectory_manager_core.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <diagnostic.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_db.h"
#include "strat_base.h"
#include "strat_corn.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"

static volatile uint8_t clitoid_slow = 0;

/* return 1 if there is a corn near, and fill the index ptr */
int8_t corn_is_near(uint8_t *corn_idx, uint8_t side)
{
	/* XXX to be checked */
#define SENSOR_CORN_DIST  225
#define SENSOR_CORN_ANGLE 90
	double x = position_get_x_double(&mainboard.pos);
	double y = position_get_y_double(&mainboard.pos);
	double a_rad = position_get_a_rad_double(&mainboard.pos);
	double x_corn, y_corn;
	int16_t x_corn_int, y_corn_int;
	struct waypoint_db *wp;

	if (side == I2C_LEFT_SIDE) {
		x_corn = x + cos(a_rad + RAD(SENSOR_CORN_ANGLE)) * SENSOR_CORN_DIST;
		y_corn = y + sin(a_rad + RAD(SENSOR_CORN_ANGLE)) * SENSOR_CORN_DIST;
	}
	else {
		x_corn = x + cos(a_rad + RAD(-SENSOR_CORN_ANGLE)) * SENSOR_CORN_DIST;
		y_corn = y + sin(a_rad + RAD(-SENSOR_CORN_ANGLE)) * SENSOR_CORN_DIST;
	}
	x_corn_int = x_corn;
	y_corn_int = y_corn;

	wp = xycoord_to_corn_idx(&x_corn_int, &y_corn_int);
	if (wp == NULL)
		return 0;
	*corn_idx = wp->corn.idx;
	return 1;
}

/* fill 2 points that are on the line (num, dir) */
static void num2line(struct line_2pts *l, uint8_t num, uint8_t dir)
{
	float n = num;

	switch (dir) {

	case LINE_UP:
		l->p1.x = n * 450 + 375;
		l->p1.y = COLOR_Y(0);
		l->p2.x = n * 450 + 375;
		l->p2.y = COLOR_Y(2100);
		break;
	case LINE_DOWN:
		l->p1.x = n * 450 + 375;
		l->p1.y = COLOR_Y(2100);
		l->p2.x = n * 450 + 375;
		l->p2.y = COLOR_Y(0);
		break;
	case LINE_R_UP:
		l->p1.x = 150;
		l->p1.y = COLOR_Y(-n * 500 + 1472);
		l->p2.x = 2850;
		l->p2.y = COLOR_Y((-n + 4) * 500 + 972);
		break;
	case LINE_L_DOWN:
		l->p1.x = 2850;
		l->p1.y = COLOR_Y((-n + 4) * 500 + 972);
		l->p2.x = 150;
		l->p2.y = COLOR_Y(-n * 500 + 1472);
		break;
	case LINE_L_UP:
		l->p1.x = 2850;
		l->p1.y = COLOR_Y(-n * 500 + 1472);
		l->p2.x = 150;
		l->p2.y = COLOR_Y((-n + 4) * 500 + 972);
		break;
	case LINE_R_DOWN:
		l->p1.x = 150;
		l->p1.y = COLOR_Y((-n + 4) * 500 + 972);
		l->p2.x = 2850;
		l->p2.y = COLOR_Y(-n * 500 + 1472);
		break;
	default:
		break;
	}
}

/* return true if we must go slow */
static uint8_t clitoid_select_speed(uint8_t num1, uint8_t dir1,
				    uint8_t num2, uint8_t dir2)
{
	int16_t x, y;
	uint8_t i, j;
	uint8_t i2, i3, j2, j3; /* next wp */

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (get_cob_count() >= 5)
		return 0; /* fast */

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find waypoint at %d,%d",
		      __FUNCTION__, x, y);
		return 1;
	}

/* 	if (time_get_s() > 32) */
/* 		DEBUG(E_USER_STRAT, "i,j = (%d %d), count=%d", i, j, */
/* 		      corn_count_neigh(i, j)); */

	if (corn_count_neigh(i, j) == 2)
		return 1;

	/* we are on intersection, let's go slow... but as we enter in
	 * the curve-part of the clitoid, we should not go there */
	if (wp_belongs_to_line(i, j, num2, dir2))
		return 0;

	/* we can ge fast if it's a 60deg angle and if we checked the
	 * current point */
	if (is_60deg(dir1, dir2))
		return 0;

	/* get next point */
	if (wp_get_neigh(i, j, &i2, &j2, dir1) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot get neigh1",
		      __FUNCTION__);
		return 1;
	}

	/* if (i2, j2) belongs to next line, check corns */
	if (wp_belongs_to_line(i2, j2, num2, dir2)) {
		if (corn_count_neigh(i2, j2) > 0)
			return 1;
		else
			return 0;
	}

	/* get next point */
	if (wp_get_neigh(i2, j2, &i3, &j3, dir1) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot get neigh2",
		      __FUNCTION__);
		return 1;
	}

	/* if (i3, j3) belongs to next line, check corns */
	if (wp_belongs_to_line(i3, j3, num2, dir2)) {
		if (corn_count_neigh(i2, j2) > 0 ||
		    corn_count_neigh(i3, j3) > 0)
			return 1;
		else
			return 0;
	}

	/* go fast */
	return 0;
}

/*
 * handle speed before clitoid (on the line), depending on strat_db.
 * return true if clitoid started
 */
#define NORETURN_DIST 300
static uint8_t speedify_clitoid(uint8_t num1, uint8_t dir1,
				uint8_t num2, uint8_t dir2)
{
	uint8_t slow;
	double turnx, turny;

	slow = clitoid_select_speed(num1, dir1, num2, dir2);
	if (slow != clitoid_slow) {
		turnx = mainboard.traj.target.line.turn_pt.x;
		turny = mainboard.traj.target.line.turn_pt.y;
		if (distance_from_robot(turnx, turny) > NORETURN_DIST) {
			clitoid_slow = slow;
			return 1;
		}
	}

	return trajectory_get_state(&mainboard.traj) == RUNNING_CLITOID_CURVE;
}

/* process the clitoid parameters, return 0 on success or -1 if
 * clitoid cannot be executed. pack_spickles is set to I2C_LEFT_SIDE,
 * I2C_RIGHT_SIDE or I2C_NO_SIDE to tell if we need to pack a specific
 * spickle. */
static int8_t strat_calc_clitoid(uint8_t num1, uint8_t dir1,
				 uint8_t num2, uint8_t dir2,
				 uint8_t *pack_spickles)
{
	double line1_a_rad, line1_a_deg, line2_a_rad;
	double diff_a_deg, diff_a_deg_abs, beta_deg;
	double radius;
	struct line_2pts l1, l2;
	line_t ll1, ll2;
	point_t p;
	int8_t ret;

	/* convert to 2 points */
	num2line(&l1, num1, dir1);
	num2line(&l2, num2, dir2);

	DEBUG(E_USER_STRAT, "line1: (%2.2f, %2.2f) -> (%2.2f, %2.2f)",
	      l1.p1.x, l1.p1.y, l1.p2.x, l1.p2.y);
	DEBUG(E_USER_STRAT, "line2: (%2.2f, %2.2f) -> (%2.2f, %2.2f)",
	      l2.p1.x, l2.p1.y, l2.p2.x, l2.p2.y);

	/* convert to line eq and find intersection */
	pts2line(&l1.p1, &l1.p2, &ll1);
	pts2line(&l2.p1, &l2.p2, &ll2);
	intersect_line(&ll1, &ll2, &p);

	line1_a_rad = atan2(l1.p2.y - l1.p1.y,
			    l1.p2.x - l1.p1.x);
	line1_a_deg = DEG(line1_a_rad);
	line2_a_rad = atan2(l2.p2.y - l2.p1.y,
			    l2.p2.x - l2.p1.x);
	diff_a_deg = DEG(line2_a_rad - line1_a_rad);
	if (diff_a_deg < -180) {
		diff_a_deg += 360;
	}
	else if (diff_a_deg > 180) {
		diff_a_deg -= 360;
	}
	diff_a_deg_abs = fabs(diff_a_deg);

/* 	printf_P(PSTR("diff_a_deg=%2.2f\r\n"), diff_a_deg_abs); */
/* 	printf_P(PSTR("inter=%2.2f,%2.2f\r\n"), p.x, p.y); */

	*pack_spickles = I2C_NO_SIDE;

	/* small angle, 60 deg */
	if (diff_a_deg_abs < 70.) {
		radius = 150;
		if (diff_a_deg > 0) {
			beta_deg = 0;
			*pack_spickles = I2C_RIGHT_SIDE;
		}
		else {
			beta_deg = 0;
			*pack_spickles = I2C_RIGHT_SIDE;
		}
	}
	/* double 90 deg for half turn -- not used */
	else if (diff_a_deg_abs < 100.) {
		radius = 100;
		if (diff_a_deg > 0)
			beta_deg = 40;
		else
			beta_deg = -40;
	}
	/* hard turn, 120 deg */
	else {
		radius = 120;//75;
		if (diff_a_deg > 0)
			beta_deg = 0;
		else
			beta_deg = 0;
	}

	clitoid_slow = clitoid_select_speed(num1, dir1, num2, dir2);
	if (clitoid_slow) {
		DEBUG(E_USER_STRAT, "slow clito");
		strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);
	}
	else {
		DEBUG(E_USER_STRAT, "fast clito");
		strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_SLOW);
	}

	/* XXX check return value !! */
	ret = trajectory_clitoid(&mainboard.traj, l1.p1.x, l1.p1.y,
				 line1_a_deg, 150., diff_a_deg, beta_deg,
				 radius, xy_norm(l1.p1.x, l1.p1.y,
						 p.x, p.y));
	return ret;
}

/* go from line num1,dir1 to line num2,dir2. Uses trjectory flags
 * specified as argument and return END_xxx condition */
uint8_t line2line(uint8_t num1, uint8_t dir1, uint8_t num2,
		  uint8_t dir2, uint8_t flags)
{
	int8_t ret;
	uint8_t err, pack_spickles;

 reprocess:
	ret = strat_calc_clitoid(num1, dir1, num2, dir2, &pack_spickles);
	if (ret < 0)
		DEBUG(E_USER_STRAT, "clitoid failed");

	/* XXX what to do if cobboard is stucked */

	/* wait beginning of clitoid or changing of speed */
	err = WAIT_COND_OR_TRAJ_END(speedify_clitoid(num1, dir1,
						     num2, dir2),
				    flags);

	/* the speed has to change */
	if (trajectory_get_state(&mainboard.traj) != RUNNING_CLITOID_CURVE)
		goto reprocess;

	DEBUG(E_USER_STRAT, "clitoid started err=%d pack_spickles=%d",
	      err, pack_spickles);

	/* when clitoid starts and angle is 60 deg, pack external
	 * spickle */
	if (err == 0) {
		if (pack_spickles == I2C_LEFT_SIDE)
			strat_lpack60 = 1;
		else if (pack_spickles == I2C_RIGHT_SIDE)
			strat_rpack60 = 1;

		/* wait end of clitoid */
		err = wait_traj_end(flags);
	}

	DEBUG(E_USER_STRAT, "clitoid finished");

	strat_rpack60 = 0;
	strat_lpack60 = 0;
	return err;
}

