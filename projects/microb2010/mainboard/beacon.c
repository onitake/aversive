/*
 *  Copyright Droids Corporation (2010)
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
 *  Revision : $Id: strat.h,v 1.7 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <i2c.h>
#include <clock_time.h>

#include <scheduler.h>
#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "strat_utils.h"

#define BEACON_UART_NUM 0

#define INIT 0
#define OPP0 1
#define OPP1 2
#define OPP2 3
#define OPP3 4
#define STA0 5
#define STA1 6
#define STA2 7
#define STA3 8

static volatile uint8_t opp_age = 0;
static volatile int16_t opp_a = I2C_OPPONENT_NOT_THERE;
static volatile int16_t opp_d = I2C_OPPONENT_NOT_THERE;

#ifndef HOST_VERSION
static void beacon_uart_cb(char c)
{
	static uint8_t state;
	static uint16_t d, a, x, y;

	/* init command */
	if ((c & 0x80) == 0)
		state = INIT;

	switch (state) {
	case INIT:
		/* recv opp */
		if (c == 0) {
			state = OPP0;
			d = 0;
			a = 0;
		}
		/* recv opp */
		else if (c == 0) {
			state = STA0;
			x = 0;
			y = 0;
		}
		break;
	case OPP0:
		d = ((uint16_t)c) & 0x7F;
		break;
	case OPP1:
		d |= (((uint16_t)c << 7) & 0x3F80);
		break;
	case OPP2:
		a = ((uint16_t)c) & 0x7F;
		break;
	case OPP3:
		a |= (((uint16_t)c << 7) & 0x3F80);
		opp_a = a;
		opp_d = d;
		opp_age = 0;
		break;
	case STA0:
		x = ((uint16_t)c) & 0x7F;
		break;
	case STA1:
		x |= (((uint16_t)c << 7) & 0x3F80);
		break;
	case STA2:
		y = ((uint16_t)c) & 0x7F;
		break;
	case STA3:
		y |= (((uint16_t)c << 7) & 0x3F80);
		beaconboard.posx = x;
		beaconboard.posy = y;
		break;
		/* XXX STA4 with angle */
	default:
		state = INIT;
		break;
	}
}
#endif

static void beacon_event(void *dummy)
{
#ifdef HOST_VERSION
	uint8_t flags;
	int16_t oppx, oppy;
	double oppa, oppd;

	IRQ_LOCK(flags);
	if (beaconboard.oppx == I2C_OPPONENT_NOT_THERE) {
		IRQ_UNLOCK(flags);
		return;
	}
	oppx = beaconboard.oppx;
	oppy = beaconboard.oppy;
	abs_xy_to_rel_da(oppx, oppy, &oppd, &oppa);
	beaconboard.oppa = DEG(oppa);
	if (beaconboard.oppa < 0)
		beaconboard.oppa += 360;
	beaconboard.oppd = oppd;
	IRQ_UNLOCK(flags);
#else
	uint8_t flags;
	double fd, fa, fx, fy;
	int16_t id, ia, ix, iy;

	/* if beacon is too old, remove it */
	IRQ_LOCK(flags);
	if (opp_age < 3)
		opp_age ++;
	else
		beaconboard.oppx = I2C_OPPONENT_NOT_THERE;

	ia = opp_a;
	id = opp_d;
	IRQ_UNLOCK(flags);

	fa = ia;
	fa = RAD(fa);
	fd = id;
	rel_da_to_abs_xy(fd, fa, &fx, &fy);

	ix = fx;
	iy = fy;

	IRQ_LOCK(flags);
	beaconboard.oppx = ix;
	beaconboard.oppy = iy;
	beaconboard.oppa = ia;
	beaconboard.oppd = id;
	IRQ_UNLOCK(flags);
#endif
}

void beacon_init(void)
{
#ifndef HOST_VERSION
	uart_register_rx_event(BEACON_UART_NUM, beacon_uart_cb);
#endif
	scheduler_add_periodical_event_priority(beacon_event, NULL,
						100000L / SCHEDULER_UNIT,
						BEACON_PRIO);
}