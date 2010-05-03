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
#include "strat_avoid.h"
#include "sensor.h"
#include "actuator.h"

#define COL_DISP_MARGIN 400 /* stop 40 cm in front of dispenser */
#define COL_SCAN_PRE_MARGIN 250

static volatile uint8_t strat_running = 0;
static volatile uint8_t want_pack = 0;
volatile uint8_t strat_lpack60 = 0;
volatile uint8_t strat_rpack60 = 0;
struct strat_conf strat_conf;

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

/* called periodically (10ms) */
void strat_event(void *dummy)
{
	uint8_t flags;
	int8_t lcob_near, rcob_near;
	uint8_t lcob, rcob;
	uint8_t lidx, ridx;

	/* ignore when strat is not running */
	if (strat_running == 0)
		return;

	/* read sensors from ballboard */
	IRQ_LOCK(flags);
	lcob = ballboard.lcob;
	ballboard.lcob = I2C_COB_NONE;
	rcob = ballboard.rcob;
	ballboard.rcob = I2C_COB_NONE;
	IRQ_UNLOCK(flags);

/* 	if (lcob != I2C_COB_NONE) */
/* 		DEBUG(E_USER_STRAT, "XXX lcob %s", */
/* 		      lcob == I2C_COB_WHITE ? "white" : "black"); */
/* 	if (rcob != I2C_COB_NONE) */
/* 		DEBUG(E_USER_STRAT, "XXX rcob %s", */
/* 		      rcob == I2C_COB_WHITE ? "white" : "black"); */
	/* XXX take opponent position into account */

#ifdef HOST_VERSION
	if (time_get_s() == 15)
		cobboard.cob_count = 5;
	if (time_get_s() == 16)
		cobboard.cob_count = 0;
#endif

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
		if (get_cob_count() >= 5 || want_pack || strat_lpack60) {
			/* nothing  */
		}
		else {
			/* deploy spickle and harvest white ones */
			if (strat_db.corn_table[lidx]->corn.color == I2C_COB_WHITE) {
				i2c_cobboard_autoharvest_nomove(I2C_LEFT_SIDE);
				if (cobboard.status == I2C_COBBOARD_STATUS_LBUSY)
					strat_db.corn_table[lidx]->present = 0;
			}
			else
				i2c_cobboard_deploy_nomove(I2C_LEFT_SIDE);
		}
	}
	else {
		/* no cob near us, we can pack or deploy freely */
		if (get_cob_count() >= 5 || want_pack || strat_lpack60)
			i2c_cobboard_pack(I2C_LEFT_SIDE);
		else
			i2c_cobboard_deploy(I2C_LEFT_SIDE);
	}

	/* control the cobboard mode for right spickle */
	if (rcob_near && strat_db.corn_table[ridx]->present) {
		if (get_cob_count() >= 5 || want_pack || strat_rpack60) {
			/* nothing */
		}
		else {
			/* deploy spickle and harvest white ones */
			if (strat_db.corn_table[ridx]->corn.color == I2C_COB_WHITE) {
				i2c_cobboard_autoharvest_nomove(I2C_RIGHT_SIDE);
				if (cobboard.status == I2C_COBBOARD_STATUS_RBUSY)
					strat_db.corn_table[ridx]->present = 0;
			}
			else
				i2c_cobboard_deploy_nomove(I2C_RIGHT_SIDE);
		}
	}
	else {
		/* no cob near us, we can pack or deploy freely */
		if (get_cob_count() >= 5 || want_pack || strat_rpack60)
			i2c_cobboard_pack(I2C_RIGHT_SIDE);
		else
			i2c_cobboard_deploy(I2C_RIGHT_SIDE);
	}

	/* limit speed when opponent is near */
	strat_limit_speed();
}


static uint8_t strat_harvest(void)
{
	return 0;
}

/* must be called from a terminal line */
static uint8_t strat_eject(void)
{
	uint8_t err;
	//XXX return vals

	/* go to eject point */
	trajectory_goto_xy_abs(&mainboard.traj, 2625, COLOR_Y(1847));
	err = WAIT_COND_OR_TRAJ_END(get_cob_count() >= 5,
				    TRAJ_FLAGS_NO_NEAR);
	if (err == 0) {
		want_pack = 1;
		strat_set_speed(SPEED_CLITOID_FAST, SPEED_ANGLE_SLOW);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	}

	/* pack arms */
	strat_event_disable();
	i2c_cobboard_pack(I2C_LEFT_SIDE);
	i2c_cobboard_pack(I2C_RIGHT_SIDE);

	/* ball ejection */
	i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_EJECT);
	trajectory_a_abs(&mainboard.traj, COLOR_A(70));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	DEBUG(E_USER_STRAT, "%s():%d", __FUNCTION__, __LINE__);
	strat_hardstop();
	time_wait_ms(2000);

	/* half turn */
	trajectory_a_rel(&mainboard.traj, COLOR_A(180));
	err = wait_traj_end(END_INTR|END_TRAJ);

	/* cob ejection */
	trajectory_d_rel(&mainboard.traj, -70);
	err = wait_traj_end(END_INTR|END_TRAJ);

	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_EJECT);
	strat_db_dump(__FUNCTION__);
	time_wait_ms(2000);

	strat_event_enable();
	want_pack = 0;
	return 0;
}

static uint8_t strat_beginning(void)
{
	uint8_t err;

	strat_set_acc(ACC_DIST, ACC_ANGLE);
	strat_set_speed(600, 60); /* OK */
	//strat_set_speed(250, 28); /* OK */

	trajectory_d_a_rel(&mainboard.traj, 1000, COLOR_A(20));
	err = WAIT_COND_OR_TRAJ_END(trajectory_angle_finished(&mainboard.traj),
				    TRAJ_FLAGS_STD);

	strat_set_acc(ACC_DIST, ACC_ANGLE);
	strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);

 l1:
	DEBUG(E_USER_STRAT, "%s():%d count=%d", __FUNCTION__, __LINE__, get_cob_count());
	err = line2line(0, LINE_UP, 2, LINE_R_DOWN, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		strat_hardstop();
		time_wait_ms(2000);
		goto l1;
	}

 l2:
	DEBUG(E_USER_STRAT, "%s():%d count=%d", __FUNCTION__, __LINE__, get_cob_count());
	err = line2line(2, LINE_R_DOWN, 2, LINE_R_UP, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		strat_hardstop();
		time_wait_ms(2000);
		goto l2;
	}

	strat_eject();

	while (1) {
		strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);
		strat_harvest_circuit();
		strat_eject();
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

uint8_t strat_main(void)
{
	uint8_t err;

	/* harvest the first cobs + balls */
	err = strat_beginning();

	while (1) {
		/* end of time exit ! */
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}

		if (need_more_elements() == 0) {
			/* we have enough elements, go to eject */
			err = strat_eject();
			if (!TRAJ_SUCCESS(err))
				continue;
		}
		else {
			/* harvest */
			err = strat_harvest();
			if (!TRAJ_SUCCESS(err))
				continue;
		}
	}

	return err;
}
