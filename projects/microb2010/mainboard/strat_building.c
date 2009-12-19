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
 *  Revision : $Id: strat_building.c,v 1.5 2009-11-08 17:24:33 zer0 Exp $
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


#define DISC_DIST_NEED_GOTO_AVOID 1000
#define DISC_DIST_PREPARE_BUILD   700
#define DISC_DIST_SLOW            500

#define ERROUT(e) do {				\
		err = e;			\
		goto end;			\
	} while(0)

static uint8_t is_ready_for_prepare_build(void)
{
	double d, a;
	if (distance_from_robot(CENTER_X, CENTER_Y) >
	    DISC_DIST_PREPARE_BUILD)
		return 0;
	abs_xy_to_rel_da(CENTER_X, CENTER_Y, &d, &a);
	if (a < RAD(-30))
		return 0;
	if (a > RAD(30))
		return 0;
	return 1;
}

/* go to the nearest place on the disc. Also prepare the arms for
 * building at the correct level. If level==-1, don't move the
 * arms. */
uint8_t strat_goto_disc(int8_t level)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	double d, a, x, y;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* workaround for some static cols configurations */
	if ((strat_infos.conf.flags & STRAT_CONF_EARLY_SCAN) == 0) {
		if (time_get_s() > 15)
			i2c_mechboard_mode_loaded();
	}

	/* if we are far from the disc, goto backward faster */
	abs_xy_to_rel_da(CENTER_X, CENTER_Y, &d, &a);
	if (d > DISC_DIST_NEED_GOTO_AVOID) {
		rel_da_to_abs_xy(d - DISC_DIST_PREPARE_BUILD, a, &x, &y);
		err = goto_and_avoid(x, y, TRAJ_FLAGS_STD,
				     TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

#ifdef HOMOLOGATION
	{
		int16_t opp_d, opp_a;
		trajectory_turnto_xy(&mainboard.traj,
				     CENTER_X, CENTER_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		
		time_wait_ms(500);
		
		err = get_opponent_da(&opp_d, &opp_a);
		if (err == 0 && opp_d < 600 &&
		    (opp_a > 325 || opp_a < 35))
			return END_ERROR;
	}
#endif

	trajectory_goto_forward_xy_abs(&mainboard.traj,
				       CENTER_X, CENTER_Y);
	err = WAIT_COND_OR_TRAJ_END(is_ready_for_prepare_build(),
				    TRAJ_FLAGS_NO_NEAR);

	if (err == END_BLOCKING) 
		ERROUT(END_BLOCKING);
	if (TRAJ_SUCCESS(err)) /* should not reach dest */
		ERROUT(END_ERROR);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	if (level != -1)
		i2c_mechboard_mode_prepare_build_both(level);
	
	err = WAIT_COND_OR_TRAJ_END(distance_from_robot(CENTER_X,
							CENTER_Y) < DISC_DIST_SLOW,
				    TRAJ_FLAGS_NO_NEAR);

	if (err == END_BLOCKING) 
		ERROUT(END_BLOCKING);
	if (TRAJ_SUCCESS(err)) /* should not reach dest */
		ERROUT(END_ERROR);

	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	if (err == END_BLOCKING) 
		ERROUT(END_TRAJ);
	if (TRAJ_SUCCESS(err)) /* should not reach dest */
		ERROUT(END_ERROR);

	ERROUT(err);
 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* must be called from the checkpoint before zone 1. */
static uint8_t strat_goto_build_zone1_near(uint8_t level)
{
	uint8_t err;

	/* turn to build zone */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_a_abs(&mainboard.traj, COLOR_A(90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		return err;

	/* move forward to reach the build zone */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
	i2c_mechboard_mode_prepare_build_both(level);
	err = strat_calib(500, TRAJ_FLAGS_SMALL_DIST);
	if (err == END_BLOCKING) {
		err = END_TRAJ;
	}

	DEBUG(E_USER_STRAT, "build zone reached");
	return err;
}

/* must be called from the checkpoint before zone 0 */
static uint8_t strat_goto_build_zone0_near(uint8_t level)
{
	uint8_t err;
#ifdef OLD_STYLE
	int16_t cur_y, diff_y, dst_y;
#endif

	/* turn to build zone */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_a_abs(&mainboard.traj, COLOR_A(90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		return err;

#ifdef OLD_STYLE
	cur_y = position_get_y_s16(&mainboard.pos);
	dst_y = COLOR_Y(AREA_Y - (ROBOT_LENGTH/2) - 100);
	diff_y = ABS(cur_y - dst_y);

	/* move forward to reach the build zone */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
	i2c_mechboard_mode_prepare_build_both(level);
	trajectory_d_rel(&mainboard.traj, diff_y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (err == END_BLOCKING) { /* not very good for z0 but... */
		err = END_TRAJ;
	}
#else
	/* move forward to reach the build zone */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
	i2c_mechboard_mode_prepare_build_both(level);
	err = strat_calib(500, TRAJ_FLAGS_SMALL_DIST);
	if (err == END_BLOCKING) {
		err = END_TRAJ;
	}
#endif

	DEBUG(E_USER_STRAT, "build zone reached");
	return err;
}

/* Go to any build zone: disc, 1a or 1b. Doesn't work with zone 0 for
 * now... */
uint8_t strat_goto_build_zone(struct build_zone *zone, uint8_t level)
{
	uint8_t err = END_TRAJ;
	uint16_t old_spdd, old_spda;
	int16_t checkpoint_x, checkpoint_y;
	int16_t errx;

	zone->last_try_time = time_get_s();

	if (zone->flags & ZONE_F_DISC)
		return strat_goto_disc(level);

	DEBUG(E_USER_STRAT, "goto build zone x=%d", zone->checkpoint_x);
	
	/* workaround for some static cols configurations */
	if (time_get_s() > 15)
		i2c_mechboard_mode_loaded();

	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	checkpoint_x = zone->checkpoint_x;
	checkpoint_y = COLOR_Y(zone->checkpoint_y);
	errx = position_get_x_s16(&mainboard.pos) - checkpoint_x;

	/* goto checkpoint if we are too far from it, or if error on x
	 * is too big. */
	if (distance_from_robot(checkpoint_x, checkpoint_y) > 300 ||
	    ABS(errx) > 15) {
		err = goto_and_avoid(checkpoint_x, checkpoint_y,
				     TRAJ_FLAGS_STD,
				     TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	if (zone->flags & ZONE_F_ZONE1)
		err = strat_goto_build_zone1_near(level);
	else if (zone->flags & ZONE_F_ZONE0)
		err = strat_goto_build_zone0_near(level);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* return a free temple structure */
struct temple *strat_get_free_temple(void)
{
	uint8_t i;

	for (i=0; i<MAX_TEMPLE; i++) {
		if (!(strat_infos.temple_list[i].flags & TEMPLE_F_VALID))
			return &strat_infos.temple_list[i];
	}
	return NULL;
}

uint8_t strat_can_build_on_temple(struct temple *temple)
{
	uint8_t col_l, col_r, max_col, lintel;

	col_l = get_column_count_left();
	col_r = get_column_count_right();
	max_col = (col_l > col_r ? col_l : col_r);
	lintel = (get_lintel_count() > 0);
	
	if (strat_infos.conf.flags & STRAT_CONF_ONLY_ONE_ON_DISC) {
		if ((temple->level_r > 5) && 
		    (temple->flags & TEMPLE_F_ON_DISC))
			return 0;
	}

	/* return symetric temples only */
	if (temple->level_l != temple->level_r)
		return 0;

	if ((time_get_s() - temple->last_try_time) < TEMPLE_DISABLE_TIME)
		return 0;
		
	/* we could do better to work on non-symetric temples */
	if (temple->level_l + max_col + lintel > 9)
		return 0;

	if (temple->flags & TEMPLE_F_MONOCOL)
		return 0;

	/* XXX don't allow to build on opponent temple. For that we
	 * must support the little back_mm. */
	if (temple->flags & TEMPLE_F_OPPONENT)
		return 0;

	return 1;
}


/* return the best existing temple for building */
struct temple *strat_get_best_temple(void)
{
	uint8_t i;
	struct temple *best = NULL;
	struct temple *temple = NULL;

	for (i=0; i<MAX_TEMPLE; i++) {
		temple = &strat_infos.temple_list[i];
		
		if (!(temple->flags & TEMPLE_F_VALID))
			continue;

		if (strat_can_build_on_temple(temple) == 0)
			continue;

		if (best == NULL) {
			best = temple;
			continue;
		}

		/* take the higher temple between 'best' and 'temple' */
		if (best->level_l < temple->level_l)
			best = temple;
	}

	DEBUG(E_USER_STRAT, "%s() return %p", __FUNCTION__, best);
	return best;
}

/* return the temple we built on the disc if any. If valid == 1, the
 * temple must be buildable. */
struct temple *strat_get_our_temple_on_disc(uint8_t valid)
{
	uint8_t i;
	struct temple *temple = NULL;

	if (strat_infos.conf.flags & STRAT_CONF_ONLY_ONE_ON_DISC) {
		return NULL;
	}

	for (i=0; i<MAX_TEMPLE; i++) {
		temple = &strat_infos.temple_list[i];
		
		if (!(temple->flags & TEMPLE_F_VALID))
			continue;

		if (valid == 1 && strat_can_build_on_temple(temple) == 0)
			continue;

		if (temple->flags & TEMPLE_F_ON_DISC)
			return temple;
	}
	return NULL;
}

#define COL_MAX(x,y) (((x)>(y)) ? (x) : (y))

#define TIME_FOR_LINTEL 3000L
#define TIME_FOR_BUILD  0L
#define TIME_FOR_COL    800L
#define TIME_MARGIN     2000L

#define CHECKPOINT_DISC_DIST 380
#define CHECKPOINT_OTHER_DIST 200
/* Grow a temple. It will update temple list. */
uint8_t strat_grow_temple(struct temple *temple)
{
	double checkpoint_x, checkpoint_y;
	uint8_t add_level = 0;
	uint8_t do_lintel = 1;
	uint8_t col_l, col_r, col_max;
	uint8_t err;
	uint16_t timeout;

	/* XXX temple must be symetric */
	uint8_t level = temple->level_l;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	if (temple->level_l >= 9)
		return END_ERROR;

	if ( (temple->zone->flags & ZONE_F_ZONE1) ||
	     (temple->zone->flags & ZONE_F_ZONE0) ) {
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
		trajectory_d_rel(&mainboard.traj, -17);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

	col_l = get_column_count_left();
	col_r = get_column_count_right();
	
	if (time_get_s() < 75) {
		/* make temple symetric: if we have 1 col on left and 2 cols
		 * on right, only build one on both sides. */
		if (col_l > col_r) {
			col_r = col_l;
			do_lintel = 0;
		}
		if (col_r > col_l) {
			col_r = col_l;
			do_lintel = 0;
		}
		if (get_lintel_count() == 0)
			do_lintel = 0;
	}
	else if (col_l != col_r)
		do_lintel = 0;

	if (col_l == 0 || col_r == 0) {
		if (temple->flags & TEMPLE_F_LINTEL)
			do_lintel = 0;
	}

	if (col_l == 0 && col_r == 0 && do_lintel == 0) {
		DEBUG(E_USER_STRAT, "nothing to do");
		return END_ERROR;
	}

	add_level = do_lintel + col_l;
	while (level + add_level > 9) {
		if (do_lintel) {
			do_lintel = 0;
			add_level = do_lintel + col_l;
			continue;
		}
		/* we know col_r and col_l are > 0 */
		col_r--;
		col_l--;
	}

	col_max = COL_MAX(col_r, col_l);

	/* Reduce nb elts if we don't have time */
	timeout = (!!col_max) * TIME_FOR_BUILD;
	timeout += col_max * TIME_FOR_COL;
	timeout += do_lintel * TIME_FOR_LINTEL;
	if ((timeout / 1000L) + time_get_s() > 89 && do_lintel) {
		do_lintel = 0;
		timeout -= TIME_FOR_LINTEL;
	}
	if ((timeout / 1000L) + time_get_s() > 89 && col_max) {
		if (col_r > 0)
			col_r--;
		if (col_l > 0)
			col_l--;
		col_max--;
		timeout -= TIME_FOR_COL;
	}

	/* take a margin for timeout */
	timeout += (!!col_max) * TIME_MARGIN;
	
	if (col_l == 0 && col_r == 0 && do_lintel == 0) {
		DEBUG(E_USER_STRAT, "nothing to do (2)");
		return END_ERROR;
	}

	DEBUG(E_USER_STRAT, "Autobuild: left=%d,%d right=%d,%d lintel=%d",
	      level, col_l, level, col_r, do_lintel);
		
	i2c_mechboard_mode_autobuild(level, col_l, I2C_AUTOBUILD_DEFAULT_DIST,
				     level, col_r, I2C_AUTOBUILD_DEFAULT_DIST,
				     do_lintel);
	WAIT_COND_OR_TIMEOUT(get_mechboard_mode() == 
			     I2C_MECHBOARD_MODE_AUTOBUILD, 100);
	err = WAIT_COND_OR_TIMEOUT(get_mechboard_mode() != 
				   I2C_MECHBOARD_MODE_AUTOBUILD, timeout);
	if (err == 0) {
		DEBUG(E_USER_STRAT, "timeout building temple (timeout was %d)", timeout);
		temple->flags = 0; /* remove temple from list */
		return END_TRAJ;
	}
	else
		DEBUG(E_USER_STRAT, "temple built");

	/* position of the robot when build the new temple */
	temple->x = position_get_x_s16(&mainboard.pos);
	temple->y = position_get_y_s16(&mainboard.pos);
	temple->a = position_get_a_deg_s16(&mainboard.pos);

	/* checkpoint is a bit behind us */
	if (temple->zone->flags & ZONE_F_DISC) {
		rel_da_to_abs_xy(CHECKPOINT_DISC_DIST, M_PI,
				 &checkpoint_x, &checkpoint_y);
	}
	else {
		rel_da_to_abs_xy(CHECKPOINT_OTHER_DIST, M_PI,
				 &checkpoint_x, &checkpoint_y);
	}
	temple->checkpoint_x = checkpoint_x;
	temple->checkpoint_y = checkpoint_y;

	temple->level_l = level + add_level;
	temple->dist_l = 0;
	temple->angle_l = 0;
	
	temple->level_r = level + add_level;
	temple->dist_r = 0;
	temple->angle_r = 0;

	temple->flags = TEMPLE_F_VALID;
	
	if (distance_from_robot(CENTER_X, CENTER_Y) < 400)
		temple->flags |= TEMPLE_F_ON_DISC;

	if (do_lintel)
		temple->flags |= TEMPLE_F_LINTEL;

	/* we must push the temple */
	if ( ((temple->zone->flags & ZONE_F_ZONE1) ||
	      (temple->zone->flags & ZONE_F_ZONE0)) &&
	     level <= 1) {
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
		trajectory_d_rel(&mainboard.traj, -100);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		i2c_mechboard_mode_push_temple(level);
		time_wait_ms(400);
		strat_set_speed(200, SPEED_ANGLE_SLOW);
		trajectory_d_rel(&mainboard.traj, 100);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

	/* Special case for big 3 */
	if (strat_infos.col_in_boobs) {
		uint16_t old_spdd, old_spda;
		strat_get_speed(&old_spdd, &old_spda);
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_SLOW);
		DEBUG(E_USER_STRAT, "%s() big 3", __FUNCTION__);
		strat_infos.col_in_boobs = 0;
		trajectory_d_rel(&mainboard.traj, -120);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		i2c_mechboard_mode_prepare_pickup_next(I2C_AUTO_SIDE,
						       I2C_MECHBOARD_MODE_CLEAR);
		WAIT_COND_OR_TIMEOUT(get_column_count() >= 2, 4000L);
		i2c_mechboard_mode_prepare_build_both(level + add_level);
		time_wait_ms(800);
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
		trajectory_d_rel(&mainboard.traj, 120);
		wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		err = strat_grow_temple(temple);
		strat_set_speed(old_spdd, old_spda);
		return err;
	}

	return END_TRAJ;
}

#define COL_BACK_DIST 70
#define COL_ANGLE     20
#define COL_ARM_DIST  220

#define COL_BACK_DIST_ZONE1 35
#define COL_ARM_DIST_ZONE1  230
#define COL_ANGLE_ZONE1     19

static uint8_t try_build_col(uint8_t l, uint8_t r,
			     uint8_t lp, uint8_t rp,
			     uint8_t lvl)
{
	uint8_t max_lvl = lvl + r + l;

	if (l == 0 && r == 0)
		return 0;
	if (lp - l == 2 && rp - r == 0)
		return 0;
	if (lp - l == 0 && rp - r == 2)
		return 0;
	if (max_lvl > 9)
		return 0;
	if (max_lvl == 9 && rp == 2 && r == 1)
		return 0;
	return max_lvl;
}

/* Grow a temple by building a column on it. It will update temple
 * list. */
uint8_t strat_grow_temple_column(struct temple *temple)
{
	uint16_t old_spdd, old_spda;
	double checkpoint_x, checkpoint_y;
	uint8_t add_level = 0;
	uint8_t col_l, col_r;
	uint8_t col_l_before, col_r_before;
	uint8_t err;
	int16_t a_abs, a;
	uint8_t level = temple->level_l;
	uint8_t lvl_ok = 0, col_l_ok = 0, col_r_ok = 0;
	uint8_t tmp_lvl;
	int16_t col_arm_dist = COL_ARM_DIST;
	int16_t col_back_dist = COL_BACK_DIST;
	int16_t col_angle = COL_ANGLE;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	if (level >= 9)
		return END_ERROR;

	strat_get_speed(&old_spdd, &old_spda);

	if ( (temple->zone->flags & ZONE_F_ZONE1) ||
	     (temple->zone->flags & ZONE_F_ZONE0) ) {
		     if (level == 1)
			     col_arm_dist = COL_ARM_DIST_ZONE1;
		     col_back_dist = COL_BACK_DIST_ZONE1;
		     col_angle = COL_ANGLE_ZONE1;
	}

	a_abs = position_get_a_deg_s16(&mainboard.pos);

	col_l_before = get_column_count_left();
	col_r_before = get_column_count_right();
	col_l = col_l_before;
	col_r = col_r_before;
	
	/* check number of cols */
	for (col_l = 0; col_l < col_l_before + 1; col_l++) {
		for (col_r = 0; col_r < col_r_before + 1; col_r++) {
			tmp_lvl = try_build_col(col_l, col_r,
						col_l_before,
						col_r_before, level);
			if (tmp_lvl > lvl_ok) {
				lvl_ok = tmp_lvl;
				col_l_ok = col_l;
				col_r_ok = col_r;
			}
		}
	}

	col_l = col_l_ok;
	col_r = col_r_ok;
	add_level = col_l + col_r;

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	
	if (col_l == 0 && col_r == 0)
		ERROUT(END_ERROR);

	DEBUG(E_USER_STRAT, "Build col: left=%d right=%d",
	      col_l, col_r);

	i2c_mechboard_mode_prepare_inside_both(level);
	trajectory_d_rel(&mainboard.traj, -col_back_dist);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
		
	/* build with left arm */
	if (col_l) {
		a = a_abs - col_angle;
		if (a < -180)
			a += 360;
		trajectory_a_abs(&mainboard.traj, a);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		if (time_get_s() > 88)
			return END_TIMER;

		if (level >= 7 && get_column_count_left() == 2)
			i2c_mechboard_mode_prepare_build_select(level+1, -1);
		else
			i2c_mechboard_mode_prepare_build_select(level, -1);
		time_wait_ms(200);
		i2c_mechboard_mode_autobuild(level, col_l, col_arm_dist,
					     0, 0, col_arm_dist, 0);
		while (get_mechboard_mode() != I2C_MECHBOARD_MODE_AUTOBUILD);
		while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);

		if ((strat_infos.conf.flags & STRAT_CONF_PUSH_OPP_COLS) &&
		    (col_r == 0) &&
		    (temple->flags & TEMPLE_F_OPPONENT)) {
			strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
			trajectory_d_rel(&mainboard.traj, -100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			i2c_mechboard_mode_push_temple_disc(I2C_LEFT_SIDE);
			time_wait_ms(500);
			trajectory_d_rel(&mainboard.traj, 100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		}
		else if ((level == 1 || level == 0) && (col_r == 0)) {
			trajectory_d_rel(&mainboard.traj, -100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			i2c_mechboard_mode_push_temple(level);
			time_wait_ms(400);
			strat_set_speed(200, SPEED_ANGLE_SLOW);
			trajectory_d_rel(&mainboard.traj, 120);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		}

		i2c_mechboard_mode_prepare_inside_select(level+col_l, -1);
	}

	/* build with right arm */
	if (col_r) {
		a = a_abs + col_angle;
		if (a > 180)
			a -= 360;
		trajectory_a_abs(&mainboard.traj, a);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		
		if (time_get_s() > 88)
			return END_TIMER;
		
		if ((level+col_l) >= 7 && get_column_count_right() == 2)
			i2c_mechboard_mode_prepare_build_select(-1, level + col_l + 1);
		else
			i2c_mechboard_mode_prepare_build_select(-1, level + col_l);
		time_wait_ms(200);
		i2c_mechboard_mode_autobuild(0, 0, col_arm_dist,
					     level + col_l, col_r,
					     col_arm_dist, 0);
		while (get_mechboard_mode() != I2C_MECHBOARD_MODE_AUTOBUILD);
		while (get_mechboard_mode() == I2C_MECHBOARD_MODE_AUTOBUILD);
		
		if ((strat_infos.conf.flags & STRAT_CONF_PUSH_OPP_COLS) &&
		    (temple->flags & TEMPLE_F_OPPONENT)) {
			strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
			trajectory_d_rel(&mainboard.traj, -100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			i2c_mechboard_mode_push_temple_disc(I2C_RIGHT_SIDE);
			time_wait_ms(500);
			trajectory_d_rel(&mainboard.traj, 100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		}
		else if (level == 1 || level == 0) {
			trajectory_d_rel(&mainboard.traj, -100);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			i2c_mechboard_mode_push_temple(level);
			time_wait_ms(400);
			strat_set_speed(200, SPEED_ANGLE_SLOW);
			trajectory_d_rel(&mainboard.traj, 120);
			wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		}

		i2c_mechboard_mode_prepare_inside_select(-1, level + col_l + col_r);
		
	}

	trajectory_a_abs(&mainboard.traj, a_abs);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* position of the robot when build the new temple */
	temple->x = position_get_x_s16(&mainboard.pos);
	temple->y = position_get_y_s16(&mainboard.pos);
	temple->a = position_get_a_deg_s16(&mainboard.pos);

	/* checkpoint is a bit behind us */
	if (temple->zone->flags | ZONE_F_DISC) {
		rel_da_to_abs_xy(CHECKPOINT_DISC_DIST, M_PI,
				 &checkpoint_x, &checkpoint_y);
	}
	else {
		rel_da_to_abs_xy(CHECKPOINT_OTHER_DIST, M_PI,
				 &checkpoint_x, &checkpoint_y);
	}
	temple->checkpoint_x = checkpoint_x;
	temple->checkpoint_y = checkpoint_y;

	temple->level_l = level + add_level;
	temple->dist_l = 0; /* XXX */
	temple->angle_l = 0;
	
	temple->level_r = level + add_level;
	temple->dist_r = 0;
	temple->angle_r = 0;

	temple->flags = TEMPLE_F_VALID | TEMPLE_F_MONOCOL;
	
	if (distance_from_robot(CENTER_X, CENTER_Y) < 400)
		temple->flags |= TEMPLE_F_ON_DISC;

	if ( (temple->zone->flags & ZONE_F_ZONE1) ||
	     (temple->zone->flags & ZONE_F_ZONE0) ) {

	}
	return END_TRAJ;
 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

uint8_t strat_build_new_temple(struct build_zone *zone)
{
	struct temple *temple;
	uint8_t level = zone->level;
	uint8_t err;

	/* create a dummy temple */
	temple = strat_get_free_temple();
	if (!temple)
		return END_ERROR;
	
	memset(temple, 0, sizeof(*temple));
	temple->level_l = level;
	temple->level_r = level;
	temple->flags = TEMPLE_F_VALID | TEMPLE_F_LINTEL;
	temple->zone = zone;

	zone->flags |= ZONE_F_BUSY;

	if (time_get_s() > 50 && time_get_s() < 85 &&
	    get_lintel_count() == 0)
		err = strat_grow_temple_column(temple);
	else
		err = strat_grow_temple(temple);

	if (!TRAJ_SUCCESS(err))
		temple->flags = 0;
	return err;
}

uint8_t strat_goto_temple(struct temple *temple)
{
	uint16_t old_spdd, old_spda;
	uint8_t err;

	DEBUG(E_USER_STRAT, "goto temple %p checkpoint=%d,%d",
	      temple, temple->checkpoint_x, temple->checkpoint_y);
	
	temple->last_try_time = time_get_s();

	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	i2c_mechboard_mode_loaded();

	err = goto_and_avoid(temple->checkpoint_x, 
			     temple->checkpoint_y,
			     TRAJ_FLAGS_STD,
			     TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	err = strat_goto_build_zone(temple->zone, temple->level_r);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	DEBUG(E_USER_STRAT, "zone reached");
	ERROUT(END_TRAJ);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* return the best existing temple for building */
struct build_zone *strat_get_best_zone(void)
{
	uint8_t i;
	struct build_zone *zone = NULL;

	for (i=0; i<MAX_ZONE; i++) {
		zone = &strat_infos.zone_list[i];

		if (zone->flags & ZONE_F_BUSY)
			continue;
		if ((time_get_s() - zone->last_try_time) < ZONE_DISABLE_TIME)
			continue;
		
		return zone;
	}
	return NULL;
}
