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
 *  Revision : $Id: strat.c,v 1.6 2009-11-08 17:24:33 zer0 Exp $
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
#include "cs.h"
#include "strat.h"
#include "strat_db.h"
#include "strat_base.h"
#include "strat_corn.h"
#include "strat_utils.h"
#include "strat_avoid.h"
#include "sensor.h"
#include "actuator.h"

#define COL_DISP_MARGIN 400 /* stop 40 cm in front of dispenser */
#define COL_SCAN_PRE_MARGIN 250

static volatile uint8_t strat_running = 0;
volatile uint8_t strat_want_pack = 0;
volatile uint8_t strat_lpack60 = 0;
volatile uint8_t strat_rpack60 = 0;
struct strat_conf strat_conf = {
	.dump_enabled = 0,
	.opp_orange = 90,
	.orphan_tomato = 50,
	.flags = 0,
};

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
		DO_POS | DO_BD | DO_POWER;

	strat_db_init();
	strat_conf_dump(__FUNCTION__);
	strat_db_dump(__FUNCTION__);
}

void strat_conf_dump(const char *caller)
{
	if (!strat_conf.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));
	printf_P(PSTR("opp_orange = %d\n"), strat_conf.opp_orange);
	printf_P(PSTR("orphan_tomato = %d\n"), strat_conf.orphan_tomato);
	printf_P(PSTR("our_orange = %s\n"),
		 (strat_conf.flags & STRAT_CONF_OUR_ORANGE) ? "y":"n");
}

void strat_event_enable(void)
{
	strat_running = 1;
}

void strat_event_disable(void)
{
	strat_running = 0;
}

/* call it just before launching the strat */
void strat_init(void)
{
#ifdef HOST_VERSION
	position_set(&mainboard.pos, 298.16,
		     COLOR_Y(308.78), COLOR_A(70.00));
#endif

	/* we consider that the color is correctly set */
	strat_running = 1;
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
	i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_HARVEST);

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
		DO_POS | DO_BD | DO_TIMER | DO_POWER;
}

/* call it after each strat */
void strat_exit(void)
{
#ifndef HOST_VERSION
	uint8_t flags;
#endif

	strat_running = 0;
	mainboard.flags &= ~(DO_TIMER);
	strat_hardstop();
	time_reset();
	wait_ms(100);
#ifndef HOST_VERSION
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	IRQ_UNLOCK(flags);
	pwm_ng_set(LEFT_PWM, 0);
	pwm_ng_set(RIGHT_PWM, 0);
#endif
}

/* mark tomato as not present */
static void check_tomato(void)
{
	int16_t x, y;
	uint8_t i, j;
	static uint8_t prev_check_time;
	uint8_t cur_time;

	/* check present tomatoes once per second */
	cur_time = time_get_s();
	if (cur_time != prev_check_time) {
		uint8_t k;

		for (k = 0; k < TOMATO_NB; k++) {
			if (strat_db.tomato_table[k]->present == 1 &&
			    strat_db.tomato_table[k]->time_removed != -1 &&
			    strat_db.tomato_table[k]->time_removed + 2 <= time_get_s()) {
#ifdef HOST_VERSION
				printf("remove tomato %d\n", k);
#endif
				strat_db.tomato_table[k]->present = 0;
			}
		}
		prev_check_time = cur_time;
	}

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0)
		return;

	if (strat_db.wp_table[i][j].type != WP_TYPE_TOMATO)
		return;

	if (strat_db.wp_table[i][j].present == 0)
		return;

	strat_db.wp_table[i][j].time_removed = time_get_s();
	strat_db.wp_table[i][j].present = 0;
#ifdef HOST_VERSION
	ballboard.ball_count ++;
	printf("add ball %d,%d\n", i, j);
#endif
}

/* mark corn as not present and give correct commands to the cobboard
 * for spickles */
static void check_corn(void)
{
	uint8_t flags;
	int8_t lcob_near, rcob_near;
	uint8_t lcob, rcob;
	uint8_t lidx, ridx;
	static uint8_t prev_check_time;
	uint8_t cur_time;

	/* read sensors from ballboard */
	IRQ_LOCK(flags);
	lcob = ballboard.lcob;
	ballboard.lcob = I2C_COB_NONE;
	rcob = ballboard.rcob;
	ballboard.rcob = I2C_COB_NONE;
	IRQ_UNLOCK(flags);

	/* XXX take opponent position into account */

	/* check present cobs once per second */
	cur_time = time_get_s();
	if (cur_time != prev_check_time) {
		uint8_t i;

		for (i = 0; i < CORN_NB; i++) {
			if (strat_db.corn_table[i]->present == 1 &&
			    strat_db.corn_table[i]->time_removed != -1 &&
			    strat_db.corn_table[i]->time_removed + 2 <= time_get_s()) {
#ifdef HOST_VERSION
				printf("remove cob %d\n", i);
#endif
				strat_db.corn_table[i]->present = 0;
			}
		}
		prev_check_time = cur_time;
	}

	/* detect cob on left side */
	lcob_near = corn_is_near(&lidx, I2C_LEFT_SIDE);
	if (lcob_near && lcob != I2C_COB_NONE) {
		if (strat_db.corn_table[lidx]->corn.color == I2C_COB_UNKNOWN)
			DEBUG(E_USER_STRAT, "lcob %s %d",
			      lcob == I2C_COB_WHITE ? "white" : "black", lidx);
		corn_set_color(strat_db.corn_table[lidx], lcob);
	}

	/* detect cob on right side */
	rcob_near = corn_is_near(&ridx, I2C_RIGHT_SIDE);
	if (rcob_near && rcob != I2C_COB_NONE) {
		if (strat_db.corn_table[ridx]->corn.color == I2C_COB_UNKNOWN)
			DEBUG(E_USER_STRAT, "rcob %s %d",
			      rcob == I2C_COB_WHITE ? "white" : "black", ridx);
		corn_set_color(strat_db.corn_table[ridx], rcob);
	}

	/* control the cobboard mode for left spickle */
	if (lcob_near && strat_db.corn_table[lidx]->present) {
		if (get_cob_count() >= 5 || strat_want_pack || strat_lpack60) {
			/* nothing  */
		}
		else {
			/* deploy spickle and harvest white ones */
			if (strat_db.corn_table[lidx]->corn.color == I2C_COB_WHITE) {
				i2c_cobboard_autoharvest_nomove(I2C_LEFT_SIDE);
				if (strat_db.corn_table[lidx]->time_removed == -1
#ifndef HOST_VERSION
				    && cobboard.status == I2C_COBBOARD_STATUS_LBUSY
#endif
				    ) {
					strat_db.corn_table[lidx]->time_removed = time_get_s();
#ifdef HOST_VERSION
					cobboard.cob_count ++;
					printf("add cob %d\n", lidx);
#endif
				}
			}
			else
				i2c_cobboard_deploy_nomove(I2C_LEFT_SIDE);
		}
	}
	else {
		/* no cob near us, we can pack or deploy freely */
		if (get_cob_count() >= 5 || strat_want_pack || strat_lpack60)
			i2c_cobboard_pack_weak(I2C_LEFT_SIDE);
		else
			i2c_cobboard_deploy(I2C_LEFT_SIDE);
	}

	/* control the cobboard mode for right spickle */
	if (rcob_near && strat_db.corn_table[ridx]->present) {
		if (get_cob_count() >= 5 || strat_want_pack || strat_rpack60) {
			/* nothing */
		}
		else {
			/* deploy spickle and harvest white ones */
			if (strat_db.corn_table[ridx]->corn.color == I2C_COB_WHITE) {
				i2c_cobboard_autoharvest_nomove(I2C_RIGHT_SIDE);
				if (strat_db.corn_table[ridx]->time_removed == -1
#ifndef HOST_VERSION
				    && cobboard.status == I2C_COBBOARD_STATUS_RBUSY
#endif
				    ) {
					strat_db.corn_table[ridx]->time_removed = time_get_s();
#ifdef HOST_VERSION
					cobboard.cob_count ++;
					printf("add cob %d\n", ridx);
#endif
				}
			}
			else
				i2c_cobboard_deploy_nomove(I2C_RIGHT_SIDE);
		}
	}
	else {
		/* no cob near us, we can pack or deploy freely */
		if (get_cob_count() >= 5 || strat_want_pack || strat_rpack60)
			i2c_cobboard_pack_weak(I2C_RIGHT_SIDE);
		else
			i2c_cobboard_deploy(I2C_RIGHT_SIDE);
	}
}

/* called periodically (10ms) */
void strat_event(void *dummy)
{
	/* ignore when strat is not running */
	if (strat_running == 0)
		return;

	check_tomato();
	check_corn();

	/* limit speed when opponent is near */
	/* disabled for 2010, we are already slow :) */
	//strat_limit_speed();
}

/* check that we are on an eject line */
static uint8_t robot_is_on_eject_line(void)
{
	int16_t x, y;
	uint8_t i, j;

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0)
		return 0;

	if (!wp_belongs_to_line(i, j, 5, LINE_UP) &&
	    !wp_belongs_to_line(i, j, 2, LINE_R_UP))
		return 0;

	return 1;
}

/* 0 = fast, 1 = slow */
static uint8_t eject_select_speed(void)
{
	int16_t x, y;
	uint8_t i, j;

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (get_cob_count() >= 5) {
		strat_want_pack = 1;
		return 0; /* fast */
	}

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find waypoint at %d,%d",
		      __FUNCTION__, x, y);
		return 1; /* slow */
	}

	if (corn_count_neigh(i, j) == 2)
		return 1; /* slow */

	return 0; /* fast */
}

/* called multiple times while we are waiting to reach the ejection
 * point */
static uint8_t speedify_eject(void)
{
	if (eject_select_speed())
		strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);
	else
		strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_SLOW);
	return 0;
}

/* must be called from a terminal line */
static uint8_t strat_eject(void)
{
	uint8_t err;

	DEBUG(E_USER_STRAT, "%s() cob_count=%d ball_count=%d",
	      __FUNCTION__, get_cob_count(), get_ball_count());

	/* check that we are called from an eject line */
	if (!robot_is_on_eject_line()) {
		DEBUG(E_USER_STRAT, "%s() not on eject line", __FUNCTION__);
		return END_ERROR;
	}

	/* go to eject point */
	trajectory_goto_forward_xy_abs(&mainboard.traj, 2625, COLOR_Y(1847));
	err = WAIT_COND_OR_TRAJ_END(speedify_eject(),
				    TRAJ_FLAGS_NO_NEAR);
	/* err is never == 0 because speedify_eject() always return 0 */
	if (!TRAJ_SUCCESS(err))
		return err;

	/* pack arms (force), and disable strat_event */
	strat_event_disable();
	i2c_cobboard_pack_weak(I2C_LEFT_SIDE);
	i2c_cobboard_pack_weak(I2C_RIGHT_SIDE);

	/* ball ejection */
	if (get_ball_count() > 0) {
		i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_EJECT);
		trajectory_a_abs(&mainboard.traj, COLOR_A(70));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			goto fail;

		DEBUG(E_USER_STRAT, "%s():%d", __FUNCTION__, __LINE__);
		strat_hardstop();
#ifdef HOST_VERSION
		time_wait_ms(2000);
#else
		WAIT_COND_OR_TIMEOUT(ballboard.status == I2C_BALLBOARD_STATUS_F_BUSY,
				     2000);
		WAIT_COND_OR_TIMEOUT(ballboard.status == I2C_BALLBOARD_STATUS_F_READY,
				     2000);
#endif
	}

	/* half turn */
	trajectory_a_abs(&mainboard.traj, COLOR_A(-110));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	/* cob ejection */
	trajectory_d_rel(&mainboard.traj, -70);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	if (get_cob_count() > 0) {
		i2c_cobboard_set_mode(I2C_COBBOARD_MODE_EJECT);
		time_wait_ms(2000);
	}

	/* cob ejection */
	trajectory_d_rel(&mainboard.traj, 70);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	strat_db_dump(__FUNCTION__);
	err = END_TRAJ;

 fail:
	strat_event_enable();
	strat_want_pack = 0;
	return err;
}

static uint8_t strat_beginning(uint8_t do_initturn)
{
	uint8_t err;

	strat_set_acc(ACC_DIST, ACC_ANGLE);

	if (do_initturn) {
		strat_set_speed(600, 60); /* OK */
		trajectory_d_a_rel(&mainboard.traj, 1000, COLOR_A(20));
		err = WAIT_COND_OR_TRAJ_END(trajectory_angle_finished(&mainboard.traj),
					    TRAJ_FLAGS_STD);
	}

	strat_set_acc(ACC_DIST, ACC_ANGLE);
	strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);

	err = line2line(0, LINE_UP, 2, LINE_R_DOWN, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		return err;

	err = line2line(2, LINE_R_DOWN, 2, LINE_R_UP, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		return err;
	}

	return END_TRAJ;
}

/* dump state (every 5 s max) */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


#if 0
/* return true if we need to grab some more elements */
static uint8_t need_more_elements(void)
{
	if (time_get_s() <= 75) {
		/* we have enough time left */
		if (get_ball_count() >= 4)
			return 0;
		if (get_cob_count() >= 4)
			return 0;
		if ((get_ball_count() >= 2) &&
		    (get_cob_count() >= 2))
			return 0;
		return 1;
	}
	else {
		/* not much time remaining */
		if ((get_ball_count() >= 1) &&
		    (get_cob_count() >= 1))
			return 0;
		return 1;
	}
}
#endif

/* get tomatoes near our goals (12,5 and 12,3) */
uint8_t get_opp_oranges(void)
{
	int16_t x, y;
	uint8_t i, j;
	uint8_t err;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	/* only if oranges are present */
	if (strat_db.opp_oranges_count == 0)
		return END_TRAJ;

	strat_db.opp_oranges_count = 0;
	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0)
		return END_ERROR;

	/* not on eject point */
	if (i != 11 || j != 6)
		return END_ERROR;

	strat_want_pack = 1;
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* turn in the correct direction */
	trajectory_a_abs(&mainboard.traj, COLOR_A(-90));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	trajectory_goto_forward_xy_abs(&mainboard.traj, 2625, COLOR_Y(597));
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_goto_forward_xy_abs(&mainboard.traj, 2750, COLOR_Y(250));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	err = run_to_the_hills(get_opponent_color());

 fail:
	strat_want_pack = 0;
	return err;
}

/* get tomatoes near our goals (12,5 and 12,3) */
uint8_t get_orphan_tomatoes(void)
{
#define CLITOID_TOMATO_RADIUS 100.
#define TOMATO_BACK_X 2760
#define TOMATO_BACK_LEN 200

	int16_t x, y, a;
	uint8_t i, j;
	uint8_t err;
	int8_t ret;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	/* only go if both tomatoes are present */
	if (!strat_db.wp_table[12][5].present ||
	    !strat_db.wp_table[12][3].present) {
		return END_TRAJ;
	}

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0)
		return END_ERROR;

	/* not on eject point */
	if (i != 11 || j != 6)
		return END_ERROR;

	strat_want_pack = 1;
	strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_SLOW);

	/* turn in the correct direction */
	trajectory_a_abs(&mainboard.traj, COLOR_A(-90));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	/* clitoid to turn and take the first ball */
	ret = trajectory_clitoid(&mainboard.traj, 2625, COLOR_Y(1847),
				 COLOR_A(-90), 150., COLOR_A(90), 0,
				 CLITOID_TOMATO_RADIUS, 3*125);
	if (ret < 0) {
		err = END_ERROR;
		goto fail;
	}

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);
	err = strat_calib(300, END_TRAJ|END_BLOCKING);
	a = position_get_a_deg_s16(&mainboard.pos);
	if (ABS(a) < 10)
		strat_reset_pos(AREA_X - ROBOT_HALF_LENGTH_FRONT,
				DO_NOT_SET_POS,
				COLOR_A(0));

	strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -250);
	err = WAIT_COND_OR_TRAJ_END(!x_is_more_than(TOMATO_BACK_X),
				    TRAJ_FLAGS_NO_NEAR);

	if (err != 0 && !TRAJ_SUCCESS(err))
		goto fail;

	trajectory_d_a_rel(&mainboard.traj, -TOMATO_BACK_LEN, COLOR_A(-90));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	/* next ball */

	/* clitoid to turn and take the first ball */
	strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_SLOW);
	ret = trajectory_clitoid(&mainboard.traj, 2625, COLOR_Y(1847),
				 COLOR_A(-90), 150., COLOR_A(90), 0,
				 CLITOID_TOMATO_RADIUS, 7*125);
	if (ret < 0) {
		err = END_ERROR;
		goto fail;
	}
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	strat_hardstop();
	strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -250);
	err = WAIT_COND_OR_TRAJ_END(!x_is_more_than(TOMATO_BACK_X),
				    TRAJ_FLAGS_NO_NEAR);
	trajectory_d_a_rel(&mainboard.traj, -TOMATO_BACK_LEN, COLOR_A(-90));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

fail:
	strat_want_pack = 0;
	return err;
}


#define HILL_LEN 1000

#define HILL_ANGLE 0
#define HILL_POSY_YELLOW 310
#define HILL_POSY_BLUE 190

#define HILL_POSX_BALLS_DOWN1 830
#define HILL_POSX_BALLS_DOWN2 920
#define HILL_POSX_BALLS_DOWN3 730
#define HILL_START_POSX 580

uint8_t prepare_hill(uint8_t orange_color, int16_t posx)
{
	int16_t startx, starty;
	uint8_t our_color = get_color();
	uint8_t err;

	if (orange_color == I2C_COLOR_YELLOW)
		starty = HILL_POSY_YELLOW;
	else
		starty = HILL_POSY_BLUE;
	if (orange_color == our_color)
		startx = posx;
	else
		startx = AREA_X - posx;
	trajectory_goto_forward_xy_abs(&mainboard.traj, startx, COLOR_Y(starty));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		return err;

	/* turn to the hills */
	if (orange_color == our_color)
		trajectory_a_abs(&mainboard.traj, COLOR_A(HILL_ANGLE));
	else
		trajectory_a_abs(&mainboard.traj, COLOR_A(-180+HILL_ANGLE));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		return err;

	return END_TRAJ;
}

/* get oranges, must be called near game area */
uint8_t run_to_the_hills(uint8_t orange_color)
{
	double aa, ad;
	uint16_t sa, sd;
	uint8_t err;
	uint8_t our_color = get_color();
	int32_t p = pid_get_gain_P(&mainboard.angle.pid);
	int32_t i = pid_get_gain_I(&mainboard.angle.pid);
	int32_t d = pid_get_gain_D(&mainboard.angle.pid);
	int32_t max_in = pid_get_max_in(&mainboard.angle.pid);
	int32_t max_i = pid_get_max_I(&mainboard.angle.pid);
	int32_t max_out = pid_get_max_out(&mainboard.angle.pid);

	strat_want_pack = 1;
	strat_get_acc(&ad, &aa);
	strat_get_speed(&sd, &sa);

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	err = prepare_hill(orange_color, HILL_START_POSX);
	if (!TRAJ_SUCCESS(err))
		return err;

	strat_set_acc(5, ACC_ANGLE);
	strat_set_speed(300, SPEED_ANGLE_SLOW);
	bd_set_current_thresholds(&mainboard.distance.bd, 500, 8000, 2000000, 80);
	bd_set_current_thresholds(&mainboard.angle.bd, 500, 8000, 2000000, 80);
	bd_set_speed_threshold(&mainboard.distance.bd, 10);
	support_balls_pack();
	i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_PREP_FORK);

	/* decrease angle gains */
	pid_set_gains(&mainboard.angle.pid, 200, 0, 2000);

	/* here it is difficult to handle return values, because we
	 * are on the hill */
	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_KICKSTAND_DOWN);
	trajectory_d_rel(&mainboard.traj, HILL_LEN);
	err = WAIT_COND_OR_TRAJ_END(position_get_x_s16(&mainboard.pos) >
				    HILL_POSX_BALLS_DOWN1,
				    TRAJ_FLAGS_SMALL_DIST);
	DEBUG(E_USER_STRAT, "deploy support balls");
	support_balls_deploy();
	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_KICKSTAND_UP);
	trajectory_only_a_rel(&mainboard.traj, 2);
	err = WAIT_COND_OR_TE_TO(0, 0, 2200);

	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_KICKSTAND_DOWN);
	strat_set_acc(3, 3);
	strat_hardstop();
	i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_TAKE_FORK);

	time_wait_ms(1800);

	/* reach top, go down */
	trajectory_d_rel(&mainboard.traj, -HILL_LEN);

	err = WAIT_COND_OR_TRAJ_END(position_get_x_s16(&mainboard.pos) <
				    HILL_POSX_BALLS_DOWN2,
				    TRAJ_FLAGS_SMALL_DIST);
	DEBUG(E_USER_STRAT, "pack support balls");
	support_balls_pack();
	err = WAIT_COND_OR_TRAJ_END(position_get_x_s16(&mainboard.pos) <
				    HILL_POSX_BALLS_DOWN3,
				    TRAJ_FLAGS_SMALL_DIST);
	DEBUG(E_USER_STRAT, "deploy support balls");
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	strat_set_acc(ad, aa);
	support_balls_deploy();
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);

	/* wait to be near the wall */
	err = WAIT_COND_OR_TRAJ_END(position_get_x_s16(&mainboard.pos) < 200,
				    TRAJ_FLAGS_SMALL_DIST);

	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);

	/* restore BD coefs */
	bd_set_current_thresholds(&mainboard.distance.bd, 500, 8000, 1000000, 20);
	bd_set_current_thresholds(&mainboard.distance.bd, 500, 8000, 1000000, 20);
	bd_set_speed_threshold(&mainboard.distance.bd, 60);

	/* calibrate position on the wall */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	err = strat_calib(-400, TRAJ_FLAGS_SMALL_DIST);
	if (orange_color == our_color)
		strat_reset_pos(ROBOT_HALF_LENGTH_REAR,
			     DO_NOT_SET_POS, COLOR_A(0));
	else
		strat_reset_pos(AREA_X - ROBOT_HALF_LENGTH_REAR,
			     DO_NOT_SET_POS, COLOR_A(180));
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	trajectory_d_rel(&mainboard.traj, 250);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (orange_color == I2C_COLOR_YELLOW)
		trajectory_a_rel(&mainboard.traj, 90);
	else
		trajectory_a_rel(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	time_wait_ms(200);

	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	err = strat_calib(-400, TRAJ_FLAGS_SMALL_DIST);
	strat_reset_pos(DO_NOT_SET_POS,
			COLOR_Y(ROBOT_HALF_LENGTH_REAR),
			COLOR_A(90));
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	trajectory_d_rel(&mainboard.traj, 250);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	/* revert acceleration and speed */
	pid_set_gains(&mainboard.angle.pid, p, i, d);
	pid_set_maximums(&mainboard.distance.pid, max_in, max_i, max_out);
	strat_want_pack = 0;
	strat_set_speed(sd, sa);
	support_balls_deploy();
	return err;
}

uint8_t strat_main(void)
{
	uint8_t err, do_initturn = 1;

	/* get oranges */
	if (strat_conf.flags & STRAT_CONF_OUR_ORANGE) {
		err = run_to_the_hills(get_color());
		strat_db.our_oranges_count = 0;
		do_initturn = 0;
	}

	/* harvest the first cobs + balls */
	err = strat_beginning(do_initturn);

	if (!TRAJ_SUCCESS(err))
		strat_unblock();
	else
		err = strat_eject();

	/* choose circuit, and harvest on it */
	while (1) {

		DEBUG(E_USER_STRAT, "start main loop");

		/* if it's time to get tomatoes, do it */
		if (time_get_s() > strat_conf.orphan_tomato) {
			err = get_orphan_tomatoes();
			if (err == END_ERROR) {
				DEBUG(E_USER_STRAT,
				      "get_orphan_tomatoes returned END_ERROR");
			}
			else if (err == END_TIMER) {
				DEBUG(E_USER_STRAT, "End of time");
				strat_exit();
				break;
			}
			else if (!TRAJ_SUCCESS(err)) {
				/* don't retry these tomatoes if it failed */
				strat_conf.orphan_tomato = 90;
				strat_unblock();
			}
		}

		/* if it's time to get opponent oranges, do it */
		if (time_get_s() > strat_conf.opp_orange) {
			err = get_opp_oranges();
			if (err == END_ERROR) {
				DEBUG(E_USER_STRAT,
				      "get_opp_oranges returned END_ERROR");
			}
			else if (err == END_TIMER) {
				DEBUG(E_USER_STRAT, "End of time");
				strat_exit();
				break;
			}
			else if (!TRAJ_SUCCESS(err)) {
				/* don't retry oranges if it failed */
				strat_conf.opp_orange = 90;
				strat_unblock();
			}
		}

		/**********************/
		/* harvest on circuit */
		/**********************/

		err = strat_harvest_circuit();
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}
		if (!TRAJ_SUCCESS(err)) {
			strat_unblock();
			continue;
		}

		/***********************/
		/* eject game elements */
		/***********************/

		err = strat_eject();
		/* end of time exit ! */
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}
		if (!TRAJ_SUCCESS(err)) {
			strat_unblock();
			continue;
		}
	}

	return err;
}
