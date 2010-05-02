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
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"

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

/*
 * - send the correct commands to the spickles
 * - return 1 if we need to stop (cobboard is stucked)
*/
static uint8_t handle_spickles(void)
{
	return 0;
#if 0
	int8_t corn_idx;

	if (!corn_is_near(&corn_idx, I2C_LEFT_SIDE))
		i2c_cobboard_mode_deploy(I2C_LEFT_SIDE);
	else {
		if (corn_table[corn_idx] == TYPE_WHITE_CORN)
			i2c_cobboard_mode_harvest(I2C_LEFT_SIDE);
		else
			i2c_cobboard_mode_pack(I2C_LEFT_SIDE);
	}
/* 	printf("%d %d\n", corn_idx, corn_table[corn_idx]); */
/* 	time_wait_ms(100); */

	if (!corn_is_near(&corn_idx, I2C_RIGHT_SIDE))
		i2c_cobboard_mode_deploy(I2C_RIGHT_SIDE);
	else {
		if (corn_table[corn_idx] == TYPE_WHITE_CORN)
			i2c_cobboard_mode_harvest(I2C_RIGHT_SIDE);
		else
			i2c_cobboard_mode_pack(I2C_RIGHT_SIDE);
	}

	return 0;
#endif
}

uint8_t line2line(uint8_t num1, uint8_t dir1,
		  uint8_t num2, uint8_t dir2)
{
	double line1_a_rad, line1_a_deg, line2_a_rad;
	double diff_a_deg, diff_a_deg_abs, beta_deg;
	double radius;
	struct line_2pts l1, l2;
	line_t ll1, ll2;
	point_t p;
	uint8_t err;
	uint16_t a_speed, d_speed;
	int8_t ret;

	/* convert to 2 points */
	num2line(&l1, dir1, num1);
	num2line(&l2, dir2, num2);

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

	if (diff_a_deg_abs < 70.) {
		radius = 200;
		if (diff_a_deg > 0)
			beta_deg = 40;
		else
			beta_deg = -40;
	}
	else if (diff_a_deg_abs < 100.) {
		radius = 100;
		if (diff_a_deg > 0)
			beta_deg = 40;
		else
			beta_deg = -40;
	}
	else {
		radius = 120;
		if (diff_a_deg > 0)
			beta_deg = 60;
		else
			beta_deg = -60;
	}

	/* XXX check return value !! */
	ret = trajectory_clitoid(&mainboard.traj, l1.p1.x, l1.p1.y,
				 line1_a_deg, 150., diff_a_deg, beta_deg,
				 radius, xy_norm(l1.p1.x, l1.p1.y,
						 p.x, p.y));
	if (ret < 0)
		DEBUG(E_USER_STRAT, "clitoid failed");

	/* disabled */
	if (0) {
		err = 0;
		while (err == 0) {
			err = WAIT_COND_OR_TRAJ_END(handle_spickles(), 0xFF);
			if (err == 0) {
				/* cobboard is stucked */
				trajectory_hardstop(&mainboard.traj);
				return err; /* XXX do something */
			}
			err = test_traj_end(0xFF);
		}
	}

	err = WAIT_COND_OR_TRAJ_END(get_cob_count() == 5, 0xFF);
	strat_get_speed(&d_speed, &a_speed);

	/* XXX 600 -> cste */
	/* XXX does not work, do better */
/* 	if (err == 0 && d_speed < 600 && */
/* 	    mainboard.traj.state == RUNNING_CLITOID_LINE) */
/* 		strat_set_speed(600, SPEED_ANGLE_FAST); */

	err = wait_traj_end(0xFF);

	return err;
}

void num2line(struct line_2pts *l, uint8_t dir, uint8_t num)
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
