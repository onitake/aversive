/*  
 *  Copyright Droids, Microb Technology (2009)
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
 *  Revision : $Id: strat_column_disp.c,v 1.5 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <time.h>
#include <spi.h>

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

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "actuator.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "i2c_protocol.h"

#define ERROUT(e) do {				\
		err = e;			\
		goto end;			\
	} while(0)

/* distance between the wheel axis and the IR sensor */
#define IR_SHIFT_DISTANCE_RIGHT 85
#define IR_SHIFT_DISTANCE_LEFT  95

/* return red or green sensor */
#define COLOR_IR_SENSOR(left)						\
	({								\
		uint8_t __ret = 0;					\
		if (left)						\
			__ret = sensor_get(S_DISP_LEFT);		\
		else							\
			__ret = sensor_get(S_DISP_RIGHT);		\
									\
		__ret;							\
	})								\

/* eject one col, some error codes are ignored here: we want to be
 * sure that the column is correctly ejected. */
uint8_t strat_eject_col(int16_t eject_a, int16_t pickup_a)
{
	uint8_t err;

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	i2c_mechboard_mode_eject();
	time_wait_ms(600);
	trajectory_a_abs(&mainboard.traj, eject_a);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST | END_NEAR);
	i2c_mechboard_mode_clear();
	time_wait_ms(1000);
	trajectory_a_abs(&mainboard.traj, pickup_a);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	return err;
}

/* get columns from dispenser. Must be called when the robot is in
 * front of the dispenser. */
static uint8_t strat_pickup_col_disp(struct column_dispenser *disp)
{
	uint16_t old_spdd, old_spda;
	int16_t recalib_x, recalib_y;
	int16_t eject_a, pickup_a;
	uint8_t err, timeout = 0;
	int8_t cols_count_before, cols_count_after, cols;
        microseconds us;
	uint8_t first_try = 1;
	uint8_t pickup_mode = I2C_MECHBOARD_MODE_PICKUP;

	/* XXX set lazy pickup mode */

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	strat_get_speed(&old_spdd, &old_spda);

	cols_count_before = get_column_count();
	pickup_a = COLOR_A(disp->pickup_a);
	eject_a = COLOR_A(disp->eject_a);
	recalib_x = disp->recalib_x;
	recalib_y = COLOR_Y(disp->recalib_y);

	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);

	/* turn to dispenser */
	i2c_mechboard_mode_prepare_pickup(I2C_AUTO_SIDE);
	trajectory_a_abs(&mainboard.traj, pickup_a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go forward until blocking, then go back ~30mm */

	pickup_wheels_on();	
 retry:
	if (time_get_s() > 86) {
		DEBUG(E_USER_STRAT, "%s() too late...", __FUNCTION__);
		return END_TIMER;
	}

	if ((strat_infos.conf.flags & STRAT_CONF_BIG_3_TEMPLE) &&
	    strat_infos.col_in_boobs == 0 &&
	    strat_infos.lazy_pickup_done == 0) {
		DEBUG(E_USER_STRAT, "%s() mode lazy", __FUNCTION__);
		pickup_mode = I2C_MECHBOARD_MODE_LAZY_PICKUP;
		strat_infos.col_in_boobs = 1;
		strat_infos.lazy_pickup_done = 1;
	}
	else {
		pickup_mode = I2C_MECHBOARD_MODE_PICKUP;
		strat_infos.col_in_boobs = 0;
	}

	if (first_try)
    		i2c_mechboard_mode_lazy_harvest();
	else
    		i2c_mechboard_mode_prepare_pickup(I2C_AUTO_SIDE);
	first_try = 0;

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, 120);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);

	err = strat_calib(600, TRAJ_FLAGS_SMALL_DIST);

	trajectory_d_rel(&mainboard.traj, -DIST_BACK_DISPENSER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	if (get_mechboard_mode() == I2C_MECHBOARD_MODE_PREPARE_EJECT) {
		strat_eject_col(eject_a, pickup_a);
		goto retry;
	}

	/* start to pickup with finger / arms */

	DEBUG(E_USER_STRAT, "%s pickup now", __FUNCTION__);
	
	if (pickup_mode == I2C_MECHBOARD_MODE_PICKUP)
		i2c_mechboard_mode_pickup();
	else
		i2c_mechboard_mode_lazy_pickup();
	WAIT_COND_OR_TIMEOUT(get_mechboard_mode() == pickup_mode, 100);
        us = time_get_us2();
	cols = get_column_count();
	while (get_mechboard_mode() == pickup_mode) {
		if (get_column_count() != cols) {
			cols = get_column_count();
			us = time_get_us2();
		}
		if ((get_column_count() - cols_count_before) >= disp->count) {
			DEBUG(E_USER_STRAT, "%s no more cols in disp", __FUNCTION__);
			break;
		}
		/* 1 second timeout */
		if (time_get_us2() - us > 1000000L) {
			DEBUG(E_USER_STRAT, "%s timeout", __FUNCTION__);
			timeout = 1;
			break;
		}
	}

	/* eject if we found a bad color column */
	
	if (get_mechboard_mode() == I2C_MECHBOARD_MODE_PREPARE_EJECT) {
		strat_eject_col(eject_a, pickup_a);
		goto retry;
	}

	/* only recalib if it was not a timeout or if we got at least
	 * 2 cols. */
	if (timeout == 0 || (get_column_count() - cols_count_before >= 2))
		strat_reset_pos(recalib_x, recalib_y, pickup_a);
	else {
		/* else just update x or y depending on disp */
		if (disp == &strat_infos.c1)
			strat_reset_pos(recalib_x, DO_NOT_SET_POS,
					DO_NOT_SET_POS);
		else
			strat_reset_pos(recalib_x, DO_NOT_SET_POS,
					DO_NOT_SET_POS);
	}

	/* go back */
	
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -300);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST | END_NEAR);

	/* update dispenser count */

	cols_count_after = get_column_count();
	cols = cols_count_after - cols_count_before;
	if (cols > 0) {
		DEBUG(E_USER_STRAT, "%s we got %d cols", __FUNCTION__, cols);
		disp->count -= cols;
		if (disp->count < 0)
			disp->count = 0;
	}

	pickup_wheels_off();
	if (pickup_mode == I2C_MECHBOARD_MODE_PICKUP)
		i2c_mechboard_mode_clear();
	else
		disp->count -= 2;

        ERROUT(END_TRAJ);

end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* 
 * Go in front of a dispenser. It will update the dispenser if it is
 * c2 or c3 if we detect that this dispenser does not exist.
 */
uint8_t strat_goto_col_disp(struct column_dispenser **pdisp)
{
	uint8_t err;
	int16_t checkpoint_x, checkpoint_y;
	int16_t scan_a;
	uint16_t old_spdd, old_spda, scan_left;
	int16_t pos1x, pos1y, pos2x, pos2y, pos, dist;
	int16_t margin_col2, margin_col3;
	struct column_dispenser *disp = *pdisp;

	if (disp->count <= 0)
		return END_ERROR;

	if (disp->last_try_time >= time_get_s())
		return END_ERROR;

	disp->last_try_time = time_get_s();

	strat_get_speed(&old_spdd, &old_spda);

	i2c_mechboard_mode_prepare_pickup_next(I2C_AUTO_SIDE,
					       I2C_MECHBOARD_MODE_CLEAR);

	/* set some useful variables */
	checkpoint_x = disp->checkpoint_x;
	checkpoint_y = COLOR_Y(disp->checkpoint_y);
	scan_a = COLOR_A(disp->scan_a);
	scan_left = COLOR_INVERT(disp->scan_left);

	/* goto checkpoint */
	DEBUG(E_USER_STRAT, "%s(): goto %s (%d,%d) scan_left=%d",
	      __FUNCTION__, disp->name, checkpoint_x,
	      checkpoint_y, scan_left);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

#if 0
	/* we have an intermediate checkpoint if we are on our
	 * side. If goto_and_avoid() returns END_ERROR, skip
	 * this checkpoint.  */
	if (position_get_x_s16(&mainboard.pos) < 1500) {
		err = goto_and_avoid(1000, COLOR_Y(1500),
				     TRAJ_FLAGS_STD,
				     TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err) && err != END_ERROR)
			ERROUT(err);
	}
#endif
	/* go to checkpoint near the dispenser */
	err = goto_and_avoid(checkpoint_x, checkpoint_y,
			     TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* turn to correct angle to prepare scanning */

	trajectory_a_abs(&mainboard.traj, scan_a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	/* scan now */

	DEBUG(E_USER_STRAT, "%s(): scanning dispenser", __FUNCTION__);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -1000);
	err = WAIT_COND_OR_TRAJ_END(!COLOR_IR_SENSOR(scan_left),
				    TRAJ_FLAGS_NO_NEAR);
	if (err) /* we should not reach end */
		ERROUT(END_ERROR);
	pos1x = position_get_x_s16(&mainboard.pos);
	pos1y = position_get_y_s16(&mainboard.pos);

	err = WAIT_COND_OR_TRAJ_END(COLOR_IR_SENSOR(scan_left),
				    TRAJ_FLAGS_NO_NEAR);
	if (err)
		ERROUT(END_ERROR);
	pos2x = position_get_x_s16(&mainboard.pos);
	pos2y = position_get_y_s16(&mainboard.pos);

	dist = distance_between(pos1x, pos1y, pos2x, pos2y);
	DEBUG(E_USER_STRAT, "%s(): scan done dist=%d", __FUNCTION__, dist);

	if (scan_left)
		trajectory_d_rel(&mainboard.traj, -IR_SHIFT_DISTANCE_LEFT + dist/2);
	else
		trajectory_d_rel(&mainboard.traj, -IR_SHIFT_DISTANCE_RIGHT + dist/2);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	if (disp == &strat_infos.c1) 
		ERROUT(END_TRAJ);

	/* mark c2 or c3 as empty... */
	if (strat_infos.c2.count == 0 || strat_infos.c3.count == 0) 
		ERROUT(END_TRAJ);

	pos = (pos2y + pos1y) / 2;
	if (scan_a == 90) /* y is decreasing when scanning */
		pos -= 80;
	else if (scan_a == -90) /* y is increasing when scanning */
		pos += 80;

	margin_col2 = ABS(pos - COLOR_Y(strat_infos.c2.recalib_y));
	margin_col3 = ABS(pos - COLOR_Y(strat_infos.c3.recalib_y));
		
	if (margin_col3 > margin_col2) {
		DEBUG(E_USER_STRAT, "%s(): delete disp c3 (scan_pos=%d)", __FUNCTION__, pos);
		strat_infos.c3.count = 0;
		*pdisp = &strat_infos.c2;
		if (strat_infos.c3.last_try_time > strat_infos.c2.last_try_time)
			strat_infos.c2.last_try_time = strat_infos.c3.last_try_time;
	}
	else {
		DEBUG(E_USER_STRAT, "%s(): delete disp c2 (scan_pos=%d)", __FUNCTION__, pos);
		strat_infos.c2.count = 0;
		*pdisp = &strat_infos.c3;
		if (strat_infos.c2.last_try_time > strat_infos.c3.last_try_time)
			strat_infos.c3.last_try_time = strat_infos.c2.last_try_time;
	}
        ERROUT(END_TRAJ);

end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* return the best dispenser between the 2 */
static struct column_dispenser *
strat_disp_compare(struct column_dispenser *a,
		   struct column_dispenser *b)
{
	uint8_t want_cols = 4 - get_column_count();

	DEBUG(E_USER_STRAT, "%s() want_cols=%d", __FUNCTION__, want_cols);

	/* an empty dispenser is not valid */
	if (a->count == 0)
		return b;
	if (b->count == 0)
		return a;

	/* try to do a round robin: this is not optimal, but at least
	 * we will try another dispenser when one fails. */
	if (a->last_try_time < b->last_try_time) {
		return a;
	}
	if (b->last_try_time < a->last_try_time) {
		return b;
	}

	/* take the one with the most columns */
	if (a->count >= want_cols && b->count < want_cols)
		return a;

	/* take the one with the most columns */
	if (b->count >= want_cols && a->count < want_cols)
		return b;

	/* the closer is the better */
	if (distance_from_robot(a->recalib_x, COLOR_Y(a->recalib_y)) <
	    distance_from_robot(b->recalib_x, COLOR_Y(b->recalib_y))) {
		return a;
	}
	return b;
}

/* choose the best dispenser */
static struct column_dispenser *strat_get_best_col_disp(void)
{
	struct column_dispenser *disp;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	/* for the first call, use c3 */
	if (strat_infos.c1.last_try_time == 0 &&
	    strat_infos.c2.last_try_time == 0 &&
	    strat_infos.c3.last_try_time == 0)
		return &strat_infos.c2; // XXX c3
	
	DEBUG(E_USER_STRAT, "%s(): compare values", __FUNCTION__);

	/* else compare with standard conditions */
	disp = strat_disp_compare(&strat_infos.c1, &strat_infos.c2);
	disp = strat_disp_compare(disp, &strat_infos.c3);

	if (disp->count == 0)
		return NULL;
	
	return disp;
}

/* choose the best dispenser, depending on disp count, distance,
 * tries, ... and go pickup on it. */
uint8_t strat_pickup_columns(void)
{
	struct column_dispenser *disp;
	uint8_t err;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);
	disp = strat_get_best_col_disp();

	if (disp == NULL) {
		DEBUG(E_USER_STRAT, "%s(): no col disp found", __FUNCTION__);
		return END_ERROR;
	}

	err = strat_goto_col_disp(&disp);
	if (!TRAJ_SUCCESS(err))
		return err;

	err = strat_pickup_col_disp(disp);
	if (!TRAJ_SUCCESS(err))
		return err;

	return END_TRAJ;
}
