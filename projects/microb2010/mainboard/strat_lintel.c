/*  
 *  Copyright Droids, Microb Technology (2008)
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
 *  Revision : $Id: strat_lintel.c,v 1.5 2009-11-08 17:24:33 zer0 Exp $
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

#define X_PRE_MARGIN 20
#define X_POST_MARGIN 10

/*
 * goto lintel disp. Return END_TRAJ if success or if there is nothing
 * to do. Return END_ERROR if dest cannot be reached, else, it may
 * return END_OBSTACLE or END_BLOCKING.
 */
uint8_t strat_goto_lintel_disp(struct lintel_dispenser *disp)
{
	uint8_t err, first_try = 1, right_ok, left_ok;
	uint16_t old_spdd, old_spda;
	int16_t left_cur, right_cur, a;
	
	if (disp->count == 0)
		return END_ERROR;

	if (get_lintel_count() == 2)
		return END_ERROR;

	if (disp->last_try_time >= time_get_s())
		return END_ERROR;

	disp->last_try_time = time_get_s();
	
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	DEBUG(E_USER_STRAT, "%s(): goto %s", __FUNCTION__, disp->name);
	i2c_mechboard_mode_prepare_pickup(I2C_AUTO_SIDE);

	err = goto_and_avoid_backward(disp->x, COLOR_Y(400),
				      TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	trajectory_a_abs(&mainboard.traj, COLOR_A(-90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	if (time_get_s() > 86) {
		DEBUG(E_USER_STRAT, "%s() too late...", __FUNCTION__);
		return END_TIMER;
	}

	i2c_mechboard_mode_prepare_get_lintel();
 retry:
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	err = strat_calib(500, TRAJ_FLAGS_SMALL_DIST);
	if (err == END_BLOCKING) {
		a = position_get_a_deg_s16(&mainboard.pos);
		/* only reset pos if angle is not too different */
		if (ABS(a - COLOR_A(-90)) < 5)
			strat_reset_pos(DO_NOT_SET_POS,
					COLOR_Y(ROBOT_LENGTH/2),
					COLOR_A(-90));
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	i2c_mechboard_mode_get_lintel();
	time_wait_ms(500);

	left_cur = sensor_get_adc(ADC_CSENSE3);
	left_ok = (left_cur > I2C_MECHBOARD_CURRENT_COLUMN);
	right_cur = mechboard.pump_right1_current;
	right_ok = (right_cur > I2C_MECHBOARD_CURRENT_COLUMN);

	DEBUG(E_USER_STRAT, "%s left_ok=%d (%d), right_ok=%d (%d)", __FUNCTION__,
	      left_ok, left_cur, right_ok, right_cur);
	if (first_try) {
		if (!right_ok && !left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
		}
		/* XXX recalib x ? */
		else if (right_ok && !left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
			strat_set_speed(500, 500);
			trajectory_d_a_rel(&mainboard.traj, -200, 30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			trajectory_d_a_rel(&mainboard.traj, 190, -30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			first_try = 0;
			goto retry;
		}
		else if (!right_ok && left_ok) {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
			strat_set_speed(500, 500);
			trajectory_d_a_rel(&mainboard.traj, -200, -30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			trajectory_d_a_rel(&mainboard.traj, 190, 30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			first_try = 0;
			goto retry;
		}
		/* else, lintel is ok */
		else {
			strat_infos.taken_lintel ++;
			i2c_mechboard_mode_put_lintel();
		}
	}
	else {
		if (right_ok && left_ok) {
			/* lintel is ok */
			strat_infos.taken_lintel ++;
			i2c_mechboard_mode_put_lintel();
		}
		else {
			i2c_mechboard_mode_prepare_get_lintel();
			time_wait_ms(300);
		}
	}
	disp->count--;

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -250);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	
	ERROUT(err);

end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* go pickup lintels on dispensers. Return END_TRAJ on success or if

 * there is nothing to do, else return the error status. */
uint8_t strat_pickup_lintels(void)
{
	uint8_t err;

	if (get_column_count() != 0)
		return END_ERROR;

	if (strat_infos.l1.count == 0 && strat_infos.l2.count == 0)
		return END_TRAJ;

	/* skip if it's too early */
	if (time_get_s() < strat_infos.conf.lintel_min_time)
		return END_TRAJ;

	/* skip next lintel if we want only one */
	if (strat_infos.conf.flags & STRAT_CONF_TAKE_ONE_LINTEL) {
		if (strat_infos.taken_lintel)
			return END_TRAJ;
	}
	
	/* don't take lintel now if we already have one and if there
	 * is not much time */
	if (get_lintel_count() && time_get_s() > 75)
		return END_TRAJ;

	/* take lintel 1 */
	err = strat_goto_lintel_disp(&strat_infos.l1);
	if (!TRAJ_SUCCESS(err) && err != END_ERROR)
		return err;

	/* skip next lintel if we want only one */
	if (strat_infos.conf.flags & STRAT_CONF_TAKE_ONE_LINTEL) {
		if (strat_infos.taken_lintel)
			return END_TRAJ;
	}
	
	/* don't take lintel now if we already have one and if there
	 * is not much time */
	if (get_lintel_count() && time_get_s() > 75)
		return END_TRAJ;

	/* take lintel 2 */
	err = strat_goto_lintel_disp(&strat_infos.l2);
	if (!TRAJ_SUCCESS(err) && err != END_ERROR)
		return err;

	return END_TRAJ;
}
