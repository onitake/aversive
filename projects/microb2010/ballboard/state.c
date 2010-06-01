/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: state.c,v 1.5 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <spi.h>
#include <encoders_spi.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "state.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

static struct vt100 local_vt100;
static volatile uint8_t state_mode;
static volatile uint8_t state_status;
static volatile uint8_t ball_count;

/* short aliases */
#define INIT I2C_BALLBOARD_MODE_INIT
#define OFF I2C_BALLBOARD_MODE_OFF
#define HARVEST I2C_BALLBOARD_MODE_HARVEST
#define EJECT I2C_BALLBOARD_MODE_EJECT
#define PREP_FORK I2C_BALLBOARD_MODE_PREP_FORK
#define TAKE_FORK I2C_BALLBOARD_MODE_TAKE_FORK

uint8_t state_debug = 0;

#if 0
static void state_dump_sensors(void)
{
	STMCH_DEBUG("TODO\n");
}
#endif

uint8_t state_get_ball_count(void)
{
	return ball_count;
}

#if 0
static void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while (!cmdline_keypressed());
}
#endif

/* set a new state, return 0 on success */
int8_t state_set_mode(uint8_t mode)
{
	state_mode = mode;
	STMCH_DEBUG("%s(): mode=%x ", __FUNCTION__, mode);

/* 	STMCH_DEBUG("%s(): l_deploy=%d l_harvest=%d " */
/* 		    "r_deploy=%d r_harvest=%d eject=%d", */
/* 		    __FUNCTION__, L_DEPLOY(mode), L_HARVEST(mode), */
/* 		    R_DEPLOY(mode), R_HARVEST(mode), EJECT(mode)); */

	return 0;
}

/* check that state is the one in parameter and that state did not
 * changed */
static uint8_t state_want_exit(void)
{
	int16_t c;

	/* force quit when CTRL-C is typed */
	c = cmdline_getchar();
	if (c == -1)
		return 0;
	if (vt100_parser(&local_vt100, c) == KEY_CTRL_C)
		return 1;
	printf_P(PSTR("CTRL-C\r\n"));
	return 0;
}

uint8_t state_get_status(void)
{
	return state_status;
}

/* harvest balls from area */
static void state_do_harvest(void)
{
	//state_debug_wait_key_pressed();

	if (bd_get(&ballboard.roller.bd)) {
		STMCH_DEBUG("%s(): roller blocked", __FUNCTION__);
		roller_reverse();
		time_wait_ms(500);
		bd_reset(&ballboard.roller.bd);
		STMCH_DEBUG("%s(): roller restart", __FUNCTION__);
		return;
	}

	/* deduct ball count */
	if (sensor_get(S_LOW_BARRIER) && !sensor_get(S_HIGH_BARRIER))
		ball_count = 1;
	else if (sensor_get(S_LOW_BARRIER) && sensor_get(S_HIGH_BARRIER))
		ball_count = 3;

	if (sensor_get(S_HIGH_BARRIER))
		roller_off();
	else
		roller_on();
}

/* eject balls */
static void state_do_eject(void)
{
	uint8_t i, blocked;
	microseconds us;

	for (i = 0; i < 3; i ++) {

		roller_reverse();

		us = time_get_us2();
		blocked = 0;

		while (1) {
			/* move fork during ball ejection */
			if ((us % 600) < 300)
				fork_eject();
			else
				fork_pack();

			/* no more balls (sensor is heavily filtered) */
			if (!sensor_get(S_LOW_BARRIER) &&
			    !sensor_get(S_HIGH_BARRIER)) {
				STMCH_DEBUG("%s(): no more balls", __FUNCTION__);
				break;
			}

			/* timeout */
			if ((time_get_us2() - us) > 2000UL * 1000UL) {
				STMCH_DEBUG("%s(): eject timeout", __FUNCTION__);
				blocked = 1;
				break;
			}

			/* blocking ! */
			if (bd_get(&ballboard.roller.bd)) {
				blocked = 1;
				break;
			}
		}
		fork_pack();

		if (!blocked)
			break;

		STMCH_DEBUG("%s(): roller blocked", __FUNCTION__);
		roller_on();
		time_wait_ms(500);
		bd_reset(&ballboard.roller.bd);
		STMCH_DEBUG("%s(): roller restart", __FUNCTION__);
	}
}

/* main state machine */
void state_machine(void)
{
	uint8_t mode = 0;

	while (state_want_exit() == 0) {

		if (state_mode != mode) {
			mode = state_mode;
			STMCH_DEBUG("%s(): mode=%x ", __FUNCTION__, mode);
		}

		switch (state_mode) {

		case INIT:
			state_init();
			fork_pack();
			state_mode = OFF;
			state_status = I2C_BALLBOARD_STATUS_F_READY;
			break;

		case OFF:
			state_status = I2C_BALLBOARD_STATUS_F_READY;
			roller_off();
			fork_pack();
			break;

		case HARVEST:
			state_status = I2C_BALLBOARD_STATUS_F_READY;
			fork_pack();
			state_do_harvest();
			break;

		case EJECT:
			state_status = I2C_BALLBOARD_STATUS_F_BUSY;
			fork_pack();
			state_do_eject();
			state_status = I2C_BALLBOARD_STATUS_F_READY;
			state_mode = HARVEST;
			break;

		case PREP_FORK:
			roller_off();
			fork_deploy();
			break;

		case TAKE_FORK:
			roller_off();
			fork_mid1();
			time_wait_ms(666);
			fork_mid2();
			time_wait_ms(666);
			while (1) {
				uint8_t packed;

				fork_pack();
				packed = WAIT_COND_OR_TIMEOUT(fork_is_packed(),
							      500);
				if (packed)
					break;
				fork_mid2();
				time_wait_ms(200);
			}
			state_mode = OFF;
			break;

		default:
			break;
		}
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	state_mode = 0;
	state_status = I2C_BALLBOARD_STATUS_F_READY;
	ball_count = 0;
}
