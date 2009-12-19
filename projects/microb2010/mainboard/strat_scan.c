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
 *  Revision : $Id: strat_scan.c,v 1.2 2009-11-08 17:24:33 zer0 Exp $
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
#include "cmdline.h"
#include "i2c_protocol.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"
#include "strat_avoid.h"
#include "sensor.h"

#define ERROUT(e) do {				\
		err = e;			\
		goto end;			\
	} while(0)


void scanner_dump_state(void)
{
	uint8_t status;

	printf_P(PSTR("scanner state:\r\n"));
	status = sensorboard.scan_status;

	printf_P(PSTR("  status=%x: "), sensorboard.scan_status);

	if (status & I2C_SCAN_DONE)
		printf_P(PSTR("DONE "));
	else
		printf_P(PSTR("RUNNING "));
	if (status & I2C_SCAN_MAX_COLUMN)
		printf_P(PSTR("OBSTACLE "));

	printf_P(PSTR("\r\n"));

	if (sensorboard.dropzone_h == -1) {
		printf_P(PSTR("No zone found\r\n"));
		return;
	}
	
	printf_P(PSTR("  column_h=%d\r\n"), sensorboard.dropzone_h);
	printf_P(PSTR("  column_x=%d\r\n"), sensorboard.dropzone_x);
	printf_P(PSTR("  column_y=%d\r\n"), sensorboard.dropzone_y);
}

/* must be larger than the disc poly */
#define CHECKPOINT_DIST 600

/* go to a specific angle on disc, if level == -1, don't move arms */
uint8_t strat_goto_disc_angle(int16_t a_deg, int8_t level)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	double x, y;
	uint8_t need_clear = 0;

	DEBUG(E_USER_STRAT, "%s(a_deg=%d, level=%d)", __FUNCTION__,
	      a_deg, level);

	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* workaround for some static cols configurations */
	if ((strat_infos.conf.flags & STRAT_CONF_EARLY_SCAN) == 0) {
		if (time_get_s() > 15)
			i2c_mechboard_mode_loaded();
	}
	/* another workaround for offensive configuration */
	else {
		if (strat_infos.i2c_loaded_skipped == 0) {
			DEBUG(E_USER_STRAT, "%s() need clear");
			strat_infos.i2c_loaded_skipped = 1;
			i2c_mechboard_mode_prepare_pickup_next(I2C_AUTO_SIDE,
							       I2C_MECHBOARD_MODE_CLEAR);
			need_clear = 1;
		}
		else
			i2c_mechboard_mode_loaded();
	}


	/* calculate the checkpoint */
	x = CHECKPOINT_DIST;
	y = 0;
	rotate(&x, &y, RAD(a_deg));
	x += CENTER_X;
	y += CENTER_Y;

	err = goto_and_avoid(x, y, TRAJ_FLAGS_STD,
			     TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	/* early offensive conf only */
	if (need_clear) {
		err = WAIT_COND_OR_TIMEOUT(get_column_count() == 2,
					   3000L);
		DEBUG(E_USER_STRAT, "%s() offensive: err=%d", err);
		if (err == 0) /* timeout */
			return END_ERROR;
	}
	err = strat_goto_disc(level);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
	
}

/* only valid for temple on disc */
int16_t strat_get_temple_angle(struct temple *temple)
{
	int16_t x, y;
	double a;

	x = temple->x;
	y = temple->y;
	x -= CENTER_X;
	y -= CENTER_Y;
	a = atan2(y, x);
	return DEG(a);
}

#define SCAN_ANGLE_OFFSET (-40)
int16_t strat_temple_angle_to_scan_angle(int16_t temple_angle)
{
	return temple_angle + SCAN_ANGLE_OFFSET;
}

/* start to scan after this distance */
#define DIST_START_SCAN 50

/* scan during this distance (includes DIST_START_SCAN) */
#define DIST_SCAN 430

/* speed of the scan */
#define SPEED_SCAN 450

/* from scanner point of view */
#define DISC_CENTER_X 15
#define DISC_CENTER_Y 13

/* distance of the checkpoint */
#define CKPT_DST 550.

/* to convert in robot coordinates */
#define SIDE_OFFSET (ROBOT_WIDTH/2)
#define DIST_OFFSET (DIST_SCAN - DIST_START_SCAN)

/* center of the disc in robot coordinates */
#define CENTER_X_SCANNER 166
#define CENTER_Y_SCANNER 174

/* center of the disc in scanner millimeters coordinates */
#define CENTER_X_SCANNER2 120
#define CENTER_Y_SCANNER2 155

/* structure filled by strat_scan_disc() */
struct scan_disc_result {	
#define SCAN_FAILED              0
#define SCAN_VALID               1
	uint8_t status;

#define SCAN_ACTION_BUILD_TEMPLE 0
#define SCAN_ACTION_BUILD_COL    1
	uint8_t action;

	uint8_t level;
};

#define SCAN_MODE_CHECK_TEMPLE 0
#define SCAN_MODE_SCAN_COL     1
#define SCAN_MODE_SCAN_TEMPLE  2

int8_t strat_scan_get_checkpoint(uint8_t mode, int16_t *ckpt_rel_x,
				 int16_t *ckpt_rel_y, int16_t *back_mm)
{
	int16_t center_rel_x, center_rel_y;
	int16_t col_rel_x, col_rel_y;
	int16_t col_vect_x, col_vect_y;
	double col_vect_norm;
	int16_t ckpt_vect_x, ckpt_vect_y;

	/* do some filtering */
	if (mode == SCAN_MODE_SCAN_TEMPLE &&
	    sensorboard.dropzone_x > CENTER_X_SCANNER) {
		DEBUG(E_USER_STRAT, "x too big");
		return -1;
	}
		
	/* process relative pos from robot point of view */
	center_rel_x = DIST_OFFSET - CENTER_Y_SCANNER;
	center_rel_y = -(SIDE_OFFSET + CENTER_X_SCANNER);
	
	col_rel_x = DIST_OFFSET - sensorboard.dropzone_y;
	col_rel_y = -(SIDE_OFFSET + sensorboard.dropzone_x);
	DEBUG(E_USER_STRAT, "col_rel = %d,%d", col_rel_x, col_rel_y);
	
	/* vector from center to column */
	col_vect_x = col_rel_x - center_rel_x;
	col_vect_y = col_rel_y - center_rel_y;
	col_vect_norm = norm(col_vect_x, col_vect_y);
	
	/* vector from center to ckpt */
	ckpt_vect_x = (double)(col_vect_x) * CKPT_DST / col_vect_norm;
	ckpt_vect_y = (double)(col_vect_y) * CKPT_DST / col_vect_norm;
	
	/* rel pos of ckpt */
	*ckpt_rel_x = center_rel_x + ckpt_vect_x;
	*ckpt_rel_y = center_rel_y + ckpt_vect_y;

	/* do some filtering */
	if (col_vect_norm > 150 || col_vect_norm < 30) {
		DEBUG(E_USER_STRAT, "bad norm");
		return -1;
	}
		
	if (mode == SCAN_MODE_SCAN_TEMPLE) {
		if (col_vect_norm > 50) {
			*back_mm = ABS(col_vect_norm-50);
		}
	}
	return 0;
}

/* 
 * scan the disc: return END_TRAJ on success (and status in result is
 * set to SCAN_VALID). In this case, all the scan_disc_result
 * structure is filled with appropriate parameters.  mode can be
 * 'check' or 'scan_col'. Note that if we do a check_temple, the level
 * field in structure must be filled first by the caller.
 */
uint8_t strat_scan_disc(int16_t angle, uint8_t mode,
			struct scan_disc_result *result)
{
	uint16_t old_spdd, old_spda;
	uint8_t err, stop_scanner = 0;
	uint8_t original_mode = mode;
	int16_t pos1x, pos1y, dist;
	int16_t back_mm = 0;

	int16_t ckpt_rel_x = 0, ckpt_rel_y = 0;

	double center_abs_x, center_abs_y;
	double ckpt_rel_d, ckpt_rel_a;
	double ckpt_abs_x, ckpt_abs_y;

	/* mark status as failed for now */
	result->status = SCAN_FAILED;

	DEBUG(E_USER_STRAT, "%s(angle=%d)", __FUNCTION__, angle);

	strat_get_speed(&old_spdd, &old_spda);

	/* go on disc */
	err = strat_goto_disc_angle(angle, -1);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	/* wait opponent before scanning */
	if (strat_infos.conf.wait_opponent > 0) {
		int16_t opp_x, opp_y, opp_d, opp_a;
		int8_t err;
		microseconds us;

		us = time_get_us2();
		while ((err = get_opponent_xyda(&opp_x, &opp_y,
						&opp_d, &opp_a)) == 0) {
			if (opp_d > 600)
				break;
			if (opp_a < 180)
				break;

			if (time_get_us2() - us >= (uint32_t)strat_infos.conf.wait_opponent * 1000000L)
				return END_ERROR;
		}
	}
	
	/* save absolute position of disc */
	rel_da_to_abs_xy(265, 0, &center_abs_x, &center_abs_y);

	strat_limit_speed_disable();

	/* go back and prepare to scan */
	strat_set_speed(1000, 1000);
	trajectory_d_a_rel(&mainboard.traj, -140, 130);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* XXX check that opp is not behind us */

	/* prepare scanner */

	stop_scanner = 1;
	i2c_sensorboard_scanner_prepare();
	time_wait_ms(250); /* XXX to remove ? */

	strat_set_speed(SPEED_SCAN, 1000);

	pos1x = position_get_x_s16(&mainboard.pos);
	pos1y = position_get_y_s16(&mainboard.pos);
	trajectory_d_rel(&mainboard.traj, -DIST_SCAN);
	
	while (1) {
		err = test_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (err != 0)
			break;
		
		dist = distance_from_robot(pos1x, pos1y);

		if (dist > DIST_START_SCAN)
			break;

		if (get_scanner_status() & I2C_SCAN_MAX_COLUMN) {
			err = END_ERROR;
			break;
		}
	}
	
	if (err) {
		if (TRAJ_SUCCESS(err))
			err = END_ERROR; /* should not reach end */
		strat_hardstop();
		trajectory_goto_xy_abs(&mainboard.traj, pos1x, pos1y);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		ERROUT(err);
	}

	/* start the scanner */

	i2c_sensorboard_scanner_start();

	err = WAIT_COND_OR_TRAJ_END(get_scanner_status() & I2C_SCAN_MAX_COLUMN,
				    TRAJ_FLAGS_NO_NEAR);
	if (err == 0)
		err = END_ERROR;
	if (!TRAJ_SUCCESS(err)) {
		strat_hardstop();
		trajectory_goto_xy_abs(&mainboard.traj, pos1x, pos1y);
		wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		ERROUT(err);
	}

	wait_scan_done(1000);

	i2c_sensorboard_scanner_stop();
	stop_scanner = 0;

	if (mode == SCAN_MODE_CHECK_TEMPLE) {
		i2c_sensorboard_scanner_algo_check(result->level,
						   CENTER_X_SCANNER2,
						   CENTER_Y_SCANNER2);
		i2cproto_wait_update();
		wait_scan_done(1000);
		scanner_dump_state();

		if (sensorboard.dropzone_h == -1 && 
		    !(strat_infos.conf.flags & STRAT_CONF_SKIP_WHEN_CHECK_FAILS)) {
			DEBUG(E_USER_STRAT, "-- try to build a temple");
			mode = SCAN_MODE_SCAN_TEMPLE;
		}
		else {
			result->action = SCAN_ACTION_BUILD_TEMPLE;
			/* level is already set by caller */
		}
	}

	if (mode == SCAN_MODE_SCAN_TEMPLE) {
		i2c_sensorboard_scanner_algo_temple(I2C_SCANNER_ZONE_DISC,
						    DISC_CENTER_X,
						    DISC_CENTER_Y);
		i2cproto_wait_update();
		wait_scan_done(1000);
		scanner_dump_state();

		if (sensorboard.dropzone_h == -1 ||
		    strat_scan_get_checkpoint(mode, &ckpt_rel_x,
					      &ckpt_rel_y, &back_mm)) {
			if (original_mode != SCAN_MODE_CHECK_TEMPLE) {
				DEBUG(E_USER_STRAT, "-- try to build a column");
				mode = SCAN_MODE_SCAN_COL;
			}
			else {
				DEBUG(E_USER_STRAT, "-- check failed");
			}
		}
		else {
			result->action = SCAN_ACTION_BUILD_TEMPLE;
			result->level = sensorboard.dropzone_h;
		}
	}

	if (mode == SCAN_MODE_SCAN_COL) {
		i2c_sensorboard_scanner_algo_column(I2C_SCANNER_ZONE_DISC,
						    DISC_CENTER_X,
						    DISC_CENTER_Y);
		i2cproto_wait_update();
		wait_scan_done(1000);
		scanner_dump_state();
		
		if (sensorboard.dropzone_h == -1 ||
		    strat_scan_get_checkpoint(mode, &ckpt_rel_x,
					      &ckpt_rel_y, &back_mm)) {
			ERROUT(END_ERROR);
		}
		else {
			result->action = SCAN_ACTION_BUILD_COL;
			result->level = sensorboard.dropzone_h;
		}
	}

	if (sensorboard.dropzone_h == -1) {
		ERROUT(END_ERROR);
	}

	if (mode == SCAN_MODE_CHECK_TEMPLE) {
		ckpt_rel_x = 220;
		ckpt_rel_y = 100;
	}

	DEBUG(E_USER_STRAT, "rel xy for ckpt is %d,%d", ckpt_rel_x, ckpt_rel_y);

	rel_xy_to_abs_xy(ckpt_rel_x, ckpt_rel_y, &ckpt_abs_x, &ckpt_abs_y);
	abs_xy_to_rel_da(ckpt_abs_x, ckpt_abs_y, &ckpt_rel_d, &ckpt_rel_a);

	DEBUG(E_USER_STRAT, "abs ckpt is %2.2f,%2.2f", ckpt_abs_x, ckpt_abs_y);

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* intermediate checkpoint for some positions */
	if ( (DEG(ckpt_rel_a) < 0 && DEG(ckpt_rel_a) > -90) ) {
		trajectory_goto_xy_rel(&mainboard.traj, 200, 100);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	trajectory_goto_xy_abs(&mainboard.traj, ckpt_abs_x, ckpt_abs_y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	if (result->action == SCAN_ACTION_BUILD_TEMPLE) {
		i2c_mechboard_mode_prepare_build_both(result->level);
	}

	trajectory_turnto_xy(&mainboard.traj, center_abs_x, center_abs_y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	pos1x = position_get_x_s16(&mainboard.pos);
	pos1y = position_get_y_s16(&mainboard.pos);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, 400);
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(pos1x, pos1y) > 200,
				    TRAJ_FLAGS_SMALL_DIST);
	if (err == 0) {
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, 400);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}
	if (TRAJ_SUCCESS(err))
		err = END_ERROR; /* should not reach end */
	if (err != END_BLOCKING && !TRAJ_SUCCESS(err)) 
		ERROUT(err);

	if (back_mm) {
		trajectory_d_rel(&mainboard.traj, -back_mm);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

	result->status = SCAN_VALID;

	strat_limit_speed_enable();
	return END_TRAJ;

 end:
	if (stop_scanner)
		i2c_sensorboard_scanner_stop();
	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
	return err;
	
}

/* do action according to scanner result. temple argument can be NULL
 * if it's a new one (from opponent) or it can be our previous
 * temple. */
uint8_t strat_scan_do_action(struct scan_disc_result *scan_result,
			     struct temple *temple, struct build_zone *zone)
{
	uint8_t err;

	/* remove the temple from the list */
	if (scan_result->status != SCAN_VALID)
		return END_ERROR;

	if (temple) {
		/* we were scanning a temple, remove it */
		if (scan_result->level != temple->level_l) {
			temple->flags = 0;
			temple = NULL;
		}
	}

	if (temple == NULL) {
		temple = strat_get_free_temple();
		if (temple == NULL)
			return END_ERROR;
		memset(temple, 0, sizeof(*temple));
		temple->level_l = scan_result->level;
		temple->level_r = scan_result->level;
		temple->flags = TEMPLE_F_OPPONENT | 
			TEMPLE_F_VALID | TEMPLE_F_LINTEL;
		temple->zone = zone;
	}
	zone->flags |= ZONE_F_BUSY;

	switch (scan_result->action) {

	case SCAN_ACTION_BUILD_COL:
		err = strat_grow_temple_column(temple);
		break;

	case SCAN_ACTION_BUILD_TEMPLE:
		err = strat_grow_temple(temple);
		break;
	default:
		err = END_TRAJ;
		break;
	}
	if (!TRAJ_SUCCESS(err))
		temple->flags = 0;
	return err;
}

uint8_t strat_build_on_opponent_temple(void)
{
	struct temple *temple;
	uint8_t err;
	struct scan_disc_result scan_result;
	int16_t temple_angle;

	if (time_get_s() < strat_infos.conf.scan_opp_min_time)
		return END_TRAJ;

	strat_infos.conf.scan_opp_min_time = 
		time_get_s() + strat_infos.conf.delay_between_opp_scan;

	/* scan on disc only */
	if (strat_infos.conf.scan_opp_angle == -1) {
		temple = strat_get_our_temple_on_disc(0);

		/* scan the opposite of our temple if we found
		 * one on disc */
		if (temple) {
			temple_angle = strat_get_temple_angle(temple);
			temple_angle += 180;
			if (temple_angle > 180)
				temple_angle -= 360;
		}
		/* else scan at 0 deg (opponent side) */
		else {
			temple_angle = 0;
		}
	}
	else {
		/* user specified scan position */
		temple_angle = strat_infos.conf.delay_between_opp_scan;
		if (temple_angle > 180)
			temple_angle -= 360;
	}
	temple_angle = strat_temple_angle_to_scan_angle(temple_angle);
	

	err = strat_scan_disc(temple_angle, SCAN_MODE_SCAN_TEMPLE,
			      &scan_result);
	if (!TRAJ_SUCCESS(err))
		return err;

	/* XXX on disc only */
	err = strat_scan_do_action(&scan_result, NULL,
				   &strat_infos.zone_list[0]);

	if (!TRAJ_SUCCESS(err))
		return err;
	
	err = strat_escape(&strat_infos.zone_list[0], TRAJ_FLAGS_STD);
	return err;
}

uint8_t strat_check_temple_and_build(void)
{
	struct temple *temple;
	uint8_t err;
	struct scan_disc_result scan_result;
	int16_t temple_angle;

	if (time_get_s() < strat_infos.conf.scan_our_min_time)
		return END_TRAJ;
	strat_infos.conf.scan_our_min_time = 
		time_get_s() + strat_infos.conf.delay_between_our_scan;

	/* on disc only, symetric only */
	temple = strat_get_our_temple_on_disc(1);
	if (temple == NULL)
		return END_TRAJ;

	temple_angle = strat_get_temple_angle(temple);
	temple_angle = strat_temple_angle_to_scan_angle(temple_angle);
	
	scan_result.level = temple->level_l;
	err = strat_scan_disc(temple_angle, SCAN_MODE_CHECK_TEMPLE,
			      &scan_result);
	if (scan_result.status != SCAN_VALID) {
		temple->flags = 0;
		temple = NULL;
	}
	/* no column after a temple check */
	else if (scan_result.action == SCAN_ACTION_BUILD_COL &&
		 time_get_s() < 70)
		err = END_ERROR;
	if (!TRAJ_SUCCESS(err))
		return err;

	err = strat_scan_do_action(&scan_result, temple,
				   temple->zone);
	if (!TRAJ_SUCCESS(err))
		return err;
	
	err = strat_escape(&strat_infos.zone_list[0], TRAJ_FLAGS_STD);
	return err;
}
