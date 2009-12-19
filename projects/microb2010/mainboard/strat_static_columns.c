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
 *  Revision : $Id: strat_static_columns.c,v 1.5 2009-11-08 17:24:33 zer0 Exp $
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
#include "strat_utils.h"
#include "strat_avoid.h"
#include "sensor.h"
#include "i2c_protocol.h"

#define ERROUT(e) do {				\
		err = e;			\
		goto end;			\
	} while(0)

#define BIG_DIST 5000

/*
 * must be called from start area.
 * get 4 static columns and build a temple on the disc
 */
uint8_t strat_static_columns(uint8_t configuration)
{
	uint8_t err;
	uint8_t col1_present = 0, col4_present = 0;
	uint16_t old_spdd, old_spda;

	DEBUG(E_USER_STRAT, "%s(%d)", __FUNCTION__, configuration);

	strat_get_speed(&old_spdd, &old_spda);

	/* calibrate scanner */
	i2c_sensorboard_scanner_calib();

	i2c_mechboard_mode_harvest();

	/* go straight. total distance is less than 5 meters */
	strat_set_speed(1000, 1000);
	trajectory_d_rel(&mainboard.traj, BIG_DIST);

	/* when y > 50, break */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(500), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);

	/* turn to 90° abs while going forward */
	DEBUG(E_USER_STRAT, "turn now");
	strat_set_speed(1000, 350);
	trajectory_only_a_abs(&mainboard.traj, COLOR_A(90));

	/* when y > 100, check the presence of column 4 */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(1000), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);
	if (get_color() == I2C_COLOR_RED && sensor_get(S_COLUMN_RIGHT))
		col4_present = 1;
	if (get_color() == I2C_COLOR_GREEN && sensor_get(S_COLUMN_LEFT))
		col4_present = 1;

	/* when y > 120, check the presence of column 1 */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(1200), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);
	if (get_color() == I2C_COLOR_RED && sensor_get(S_COLUMN_RIGHT))
		col1_present = 1;
	if (get_color() == I2C_COLOR_GREEN && sensor_get(S_COLUMN_LEFT))
		col1_present = 1;

	/* when y > 130, break */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(1300), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);

	strat_infos.s_cols.flags |= STATIC_COL_LINE0_DONE;

	DEBUG(E_USER_STRAT, "col4=%d col1=%d", col4_present, col1_present);
	DEBUG(E_USER_STRAT, "have %d cols", get_column_count());

	if (configuration == 0) {
		if (get_column_count() > 2) {
			configuration = 1;
			if (col4_present || col1_present) {
				strat_infos.s_cols.flags |= 
					STATIC_COL_LINE2_DONE;
			}
			else {
				strat_infos.s_cols.flags |= 
					STATIC_COL_LINE1_DONE;
			}
		}

		/* only 2 colums on the first line */
		else {
			/* all other colums are on line 1 */
			if (col4_present && col1_present) {
				configuration = 2;
				strat_infos.s_cols.flags |= 
					STATIC_COL_LINE2_DONE;
			}

			/* only 2 columns on line 1, so there are also
			 * 2 on line 2 */
			else if (col4_present || col1_present) {
				configuration = 4;
				strat_infos.s_cols.flags |= 
					STATIC_COL_LINE2_DONE;
			}

			/* all other columns are on line 2 */
			else {
				configuration = 3;
				strat_infos.s_cols.flags |= 
					STATIC_COL_LINE1_DONE;
			}
		}
	}

	strat_infos.s_cols.configuration = configuration;
	DEBUG(E_USER_STRAT, "use configuration %d", configuration);

	if (configuration == 1) {
		/* we already got 4 columns, go to the disc directly */

		strat_set_speed(1500, 900);
		trajectory_only_a_abs(&mainboard.traj, COLOR_A(0));
		err = WAIT_COND_OR_TRAJ_END(x_is_more_than(1100), TRAJ_FLAGS_STD);

		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
	}
	else if (configuration == 2 /* go from line 0 to line 1 */) {
		strat_set_speed(800, 1000);
		/* relative is needed here */
		trajectory_only_a_rel(&mainboard.traj, COLOR_A(-180));
		err = WAIT_COND_OR_TRAJ_END(!y_is_more_than(1300), TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
		strat_set_speed(1000, 600);
		err = WAIT_COND_OR_TRAJ_END(!y_is_more_than(1100),
					    TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
	}
	else if (configuration == 3 /* go from line 0 to line 2 and there is 4 columns
		    on line 2*/) {
		strat_set_speed(1000, 600);
		/* relative is needed here */
		trajectory_only_a_rel(&mainboard.traj, COLOR_A(-180));
		err = WAIT_COND_OR_TRAJ_END(!y_is_more_than(1110), TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
	} 	
	else if (configuration == 4 /* go from line 0 to line 2 and there is 2 columns
		      on line 2 */) {
		strat_set_speed(1000, 600);
		/* relative is needed here */
		trajectory_only_a_rel(&mainboard.traj, COLOR_A(-180));
		err = WAIT_COND_OR_TRAJ_END(!y_is_more_than(600), TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
	}
	else {
		trajectory_stop(&mainboard.traj);
	}

	ERROUT(END_TRAJ);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/*
 * get last 2 columns
 * must be called after the first temple building
 */
uint8_t strat_static_columns_pass2(void)
{
	uint16_t old_spdd, old_spda;
	uint8_t side, err, next_mode;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	strat_get_speed(&old_spdd, &old_spda);

	if (get_color() == I2C_COLOR_RED)
		side = I2C_RIGHT_SIDE;
	else
		side = I2C_LEFT_SIDE;

	if (strat_infos.conf.flags & STRAT_CONF_STORE_STATIC2)
		next_mode = I2C_MECHBOARD_MODE_STORE;
	else
		next_mode = I2C_MECHBOARD_MODE_HARVEST;

	switch (strat_infos.s_cols.configuration) {

	/* configuration 1: 4 cols on line 0 */
	case 1:
		if (strat_infos.s_cols.flags & STATIC_COL_LINE1_DONE) {
			/* go on line 2 */

			strat_set_speed(2000, 700);
			trajectory_d_a_rel(&mainboard.traj, -450, COLOR_A(35));
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
			
			i2c_mechboard_mode_prepare_pickup_next(side, 
							       next_mode);

			strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
			trajectory_goto_forward_xy_abs(&mainboard.traj,
						       LINE2_X, 
						       COLOR_Y(400));
			err = WAIT_COND_OR_TRAJ_END(get_column_count() == 2,
						    TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		}
		else {
			/* go on line 1 */
			strat_set_speed(2000, 700);
			trajectory_d_a_rel(&mainboard.traj, -650, COLOR_A(55));
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
			
			i2c_mechboard_mode_prepare_pickup_next(side, 
							       next_mode);

			strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

			err = goto_and_avoid_forward(LINE1_X, 
						     COLOR_Y(400),
						     TRAJ_FLAGS_NO_NEAR,
						     TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		}

		ERROUT(END_TRAJ);
		break;

	/* configuration 2: 2 cols on line 0,
	   all other colums are on line 1 */
	case 2:
		/* go on line 1 */
		strat_set_speed(2000, 700);
		trajectory_d_a_rel(&mainboard.traj, -410, COLOR_A(-20));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
			
		i2c_mechboard_mode_prepare_pickup_next(side, 
						       next_mode);

		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

		err = goto_and_avoid_forward(COL10_X, COLOR_Y(400),
					     TRAJ_FLAGS_NO_NEAR,
					     TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		
		ERROUT(END_TRAJ);
		break;

	/* configuration 3: 2 cols on line 0,
	   all other colums are on line 2 */
	case 3:
		/* go on line 2 */
		strat_set_speed(2000, 700);
		trajectory_d_a_rel(&mainboard.traj, -150, COLOR_A(-30));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
			
		i2c_mechboard_mode_prepare_pickup_next(side, 
						       next_mode);

		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

		trajectory_goto_forward_xy_abs(&mainboard.traj,
					       LINE2_X, 
					       COLOR_Y(400));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		ERROUT(END_TRAJ);
		break;

	/* configuration 4: 2 cols on line 0,
	   2 on line 1, 2 on line 2 */
	case 4:
		/* go on line 1 */
		strat_set_speed(600, 2000);
		trajectory_d_a_rel(&mainboard.traj, -BIG_DIST,
				   COLOR_A(-135));
		err = WAIT_COND_OR_TRAJ_END(y_is_more_than(900),
					    TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);

		DEBUG(E_USER_STRAT, "%s():%d", __FUNCTION__, __LINE__);
		i2c_mechboard_mode_prepare_pickup_next(side, 
						       next_mode);

		strat_set_speed(2000, 2000);
		trajectory_d_rel(&mainboard.traj, -BIG_DIST);
		err = WAIT_COND_OR_TRAJ_END(y_is_more_than(1100),
					    TRAJ_FLAGS_STD);
		if (TRAJ_SUCCESS(err)) /* we should not reach end */
			ERROUT(END_ERROR);
		else if (err)
			ERROUT(err);
		
		DEBUG(E_USER_STRAT, "%s():%d", __FUNCTION__, __LINE__);
		trajectory_d_a_rel(&mainboard.traj, -600, COLOR_A(40));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
			
		DEBUG(E_USER_STRAT, "%s():%d", __FUNCTION__, __LINE__);
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
		err = goto_and_avoid_forward(LINE1_X, 
					     COLOR_Y(400),
					     TRAJ_FLAGS_NO_NEAR,
					     TRAJ_FLAGS_NO_NEAR);
		ERROUT(END_TRAJ);
		break;

	default:
		break;
	}
	
	/* should not reach this point */
	ERROUT(END_ERROR);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}
