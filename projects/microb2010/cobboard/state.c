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
#include "spickle.h"
#include "shovel.h"
#include "state.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

static struct vt100 local_vt100;
static volatile uint8_t state_mode;
static uint8_t cob_count;

/* short aliases */
#define L_DEPLOY(mode)   (!!((mode) & I2C_COBBOARD_MODE_L_DEPLOY))
#define R_DEPLOY(mode)   (!!((mode) & I2C_COBBOARD_MODE_R_DEPLOY))
#define DEPLOY(side, mode) ((side) == I2C_LEFT_SIDE ? L_DEPLOY(mode) : R_DEPLOY(mode))
#define L_HARVEST(mode)  (!!((mode) & I2C_COBBOARD_MODE_L_HARVEST))
#define R_HARVEST(mode)  (!!((mode) & I2C_COBBOARD_MODE_R_HARVEST))
#define HARVEST(side, mode) ((side) == I2C_LEFT_SIDE ? L_HARVEST(mode) : R_HARVEST(mode))
#define EJECT(mode)      (!!((mode) & I2C_COBBOARD_MODE_EJECT))
#define INIT(mode)       (!!((mode) & I2C_COBBOARD_MODE_INIT))

uint8_t state_debug = 0;

#if 0
static void state_dump_sensors(void)
{
	STMCH_DEBUG("TODO\n");
}
#endif

uint8_t state_get_cob_count(void)
{
	return cob_count;
}

static void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* return true if cob is present */
static uint8_t state_cob_present(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return sensor_get(S_LCOB);
	else
		/* XXX */
		//		return sensor_get(S_LCOB);
		return 0;
}

/* return the detected color of the cob (only valid if present) */
static uint8_t state_cob_color(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return I2C_COB_WHITE;
	else
		/* XXX */
		//		return sensor_get(S_LCOB);
		return 0;
}

/* set a new state, return 0 on success */
int8_t state_set_mode(uint8_t mode)
{
	state_mode = mode;

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

uint8_t state_get_mode(void)
{
	return state_mode;
}

/* harvest cobs from area */
static void state_do_harvest(uint8_t side)
{
	uint16_t delay;

	/* if there is no cob, return */
	if (state_cob_present(side))
		return;

	/* if it is black, nothing to do */
	if (state_cob_color(side) == I2C_COB_BLACK)
		return;

	/* eat the cob */
	spickle_pack(side);
	state_debug_wait_key_pressed();
	/* xxx */
	time_wait_ms(250);
	left_cobroller_on();
	delay = spickle_get_pack_delay(side);
	time_wait_ms(delay);

	/* redeploy the spickle */
	spickle_deploy(side);
	state_debug_wait_key_pressed();

	cob_count ++;

	/* store it */
	shovel_up();
	wait_ms(200);
	state_debug_wait_key_pressed();
	shovel_down();
	left_cobroller_off();
	state_debug_wait_key_pressed();
	time_wait_ms(500);
}

/* eject cobs */
static void state_do_eject(void)
{
	cob_count = 0;
	shovel_mid();
	servo_door_open();
	time_wait_ms(2000);
	shovel_down();
	servo_door_close();
}

/* main state machine */
void state_machine(void)
{
	while (state_want_exit() == 0) {

		/* init */
		if (INIT(state_mode)) {
			state_mode &= (~I2C_COBBOARD_MODE_INIT);
			state_init();
		}

		/* pack/deply spickles, enable/disable roller */
		if (L_DEPLOY(state_mode)) {
			spickle_deploy(I2C_LEFT_SIDE);
			//left_cobroller_on();
			left_cobroller_off();
		}
		else {
			spickle_pack(I2C_LEFT_SIDE);
			left_cobroller_off();
		}
		if (R_DEPLOY(state_mode)) {
			spickle_deploy(I2C_RIGHT_SIDE);
			right_cobroller_on();
		}
		else {
			spickle_pack(I2C_RIGHT_SIDE);
			right_cobroller_off();
		}

		/* harvest */
		if (L_DEPLOY(state_mode) && L_HARVEST(state_mode))
			state_do_harvest(I2C_LEFT_SIDE);
		if (R_DEPLOY(state_mode) && R_HARVEST(state_mode))
			state_do_harvest(I2C_RIGHT_SIDE);

		/* eject */
		if (EJECT(state_mode)) {
			state_mode &= (~I2C_COBBOARD_MODE_EJECT);
			state_do_eject();
		}
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	shovel_down();
	servo_door_close();
	spickle_pack(I2C_LEFT_SIDE);
	spickle_pack(I2C_RIGHT_SIDE);
	state_mode = 0;
	cob_count = 0;
}
