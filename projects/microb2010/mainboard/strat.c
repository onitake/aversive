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
#include "sensor.h"
#include "actuator.h"

#define COL_DISP_MARGIN 400 /* stop 40 cm in front of dispenser */
#define COL_SCAN_PRE_MARGIN 250

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

	//i2c_cobboard_mode_init();
	strat_conf_dump(__FUNCTION__);
	strat_db_dump(__FUNCTION__);
}

void strat_conf_dump(const char *caller)
{
	if (!strat_conf.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));

}

/* call it just before launching the strat */
void strat_init(void)
{
	/* XXX init rollers, .. */

	strat_db_init();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_HARVEST);
	i2c_cobboard_harvest(I2C_LEFT_SIDE);
	i2c_cobboard_harvest(I2C_RIGHT_SIDE);
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

/* called periodically */
void strat_event(void *dummy)
{
#if 0
	/* pack or deploy spickle */
	if (strat_infos.status.flags & STRAT_STATUS_LHARVEST) {
		if (sensor_get(S_LCOB_PRESENT)) {
			if (sensor_get(S_LCOB_WHITE))
				i2c_ballboard_set_mode();
			else
				;
		}
	}
#endif
	/* limit speed when opponent is close */
	strat_limit_speed();
}

static uint8_t strat_beginning(void)
{
	uint8_t err;

	strat_set_speed(250, SPEED_ANGLE_FAST);

 l1:
	err = line2line(LINE_UP, 0, LINE_R_DOWN, 2);
	if (!TRAJ_SUCCESS(err)) {
		trajectory_hardstop(&mainboard.traj);
		time_wait_ms(2000);
		goto l1;
	}

 l2:
	err = line2line(LINE_R_DOWN, 2, LINE_R_UP, 2);
	if (!TRAJ_SUCCESS(err)) {
		trajectory_hardstop(&mainboard.traj);
		time_wait_ms(2000);
		goto l2;
	}

 l3:
	err = line2line(LINE_R_UP, 2, LINE_UP, 5);
	if (!TRAJ_SUCCESS(err)) {
		trajectory_hardstop(&mainboard.traj);
		time_wait_ms(2000);
		goto l3;
	}

	trajectory_hardstop(&mainboard.traj);

	/* ball ejection */
	trajectory_a_abs(&mainboard.traj, COLOR_A(90));
	i2c_ballboard_set_mode(I2C_BALLBOARD_MODE_EJECT);
	time_wait_ms(2000);

	/* half turn */
	trajectory_goto_xy_abs(&mainboard.traj, 2625, COLOR_Y(1847));
	err = wait_traj_end(END_INTR|END_TRAJ);
	i2c_cobboard_pack(I2C_LEFT_SIDE);
	i2c_cobboard_pack(I2C_RIGHT_SIDE);
	trajectory_a_rel(&mainboard.traj, COLOR_A(180));
	err = wait_traj_end(END_INTR|END_TRAJ);

	/* cob ejection */
	trajectory_d_rel(&mainboard.traj, -100);
	err = wait_traj_end(END_INTR|END_TRAJ);
	i2c_cobboard_set_mode(I2C_COBBOARD_MODE_EJECT);
	time_wait_ms(2000);

	trajectory_hardstop(&mainboard.traj);
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


uint8_t strat_main(void)
{
	uint8_t err;

	/* */
	err = strat_beginning();

	return err;
}
