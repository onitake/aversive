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
static volatile uint8_t state_mode, lspickle, rspickle;
static volatile uint8_t state_status;
static volatile uint8_t state_i2c_ignore = 0;
static uint8_t cob_count;

/* short aliases */
#define INIT(mode)       ((mode) == I2C_COBBOARD_MODE_INIT)
#define HARVEST(mode)    ((mode) == I2C_COBBOARD_MODE_HARVEST)
#define EJECT(mode)      ((mode) == I2C_COBBOARD_MODE_EJECT)
#define KICKSTAND_UP(mode)    ((mode) == I2C_COBBOARD_MODE_KICKSTAND_UP)
#define KICKSTAND_DOWN(mode)  ((mode) == I2C_COBBOARD_MODE_KICKSTAND_DOWN)

uint8_t state_debug = 0;

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

/* return true if the cob is correctly inside */
static uint8_t state_cob_inside(void)
{
	return sensor_get(S_COB_INSIDE_L) &&
		sensor_get(S_COB_INSIDE_R);
}

/* return true if there is no cob correctly inside */
static uint8_t state_no_cob_inside(void)
{
	return !sensor_get(S_COB_INSIDE_L) &&
		!sensor_get(S_COB_INSIDE_R);
}

static uint8_t state_spicklemode_deployed(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return lspickle & I2C_COBBOARD_SPK_DEPLOY;
	else
		return rspickle & I2C_COBBOARD_SPK_DEPLOY;
}

static uint8_t state_spicklemode_autoharvest(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return lspickle & I2C_COBBOARD_SPK_AUTOHARVEST;
	else
		return rspickle & I2C_COBBOARD_SPK_AUTOHARVEST;
}

static uint8_t state_spicklemode_nomove(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return lspickle & I2C_COBBOARD_SPK_NO_MOVE;
	else
		return rspickle & I2C_COBBOARD_SPK_NO_MOVE;
}

uint8_t state_spicklemode_weak(uint8_t side)
{
	if (side == I2C_LEFT_SIDE)
		return lspickle & I2C_COBBOARD_SPK_WEAK;
	else
		return rspickle & I2C_COBBOARD_SPK_WEAK;
}

/* pack/deploy spickles depending on mode */
static void spickle_prepare(uint8_t side)
{
	/* pack spickle if we are not in harvest mode */
	if (!HARVEST(state_mode))
		spickle_pack(side);

	/* we do nothing in mode no out */
	if (state_spicklemode_nomove(side))
		return;

	if (state_spicklemode_deployed(side))
		spickle_deploy(side);
	else
		spickle_pack(side);
}

void state_set_spickle(uint8_t side, uint8_t flags)
{
	if (side == I2C_LEFT_SIDE) {
		/* the PACK command preempts the current action if the
		 * arm is not busy */
		if (lspickle != 0 && flags == 0 &&
		    state_status != I2C_COBBOARD_STATUS_LBUSY)
			spickle_prepare(I2C_LEFT_SIDE);
		lspickle = flags;
	}
	else {
		/* the PACK command preempts the current action if the
		 * arm is not busy */
		if (rspickle != 0 && flags == 0 &&
		    state_status != I2C_COBBOARD_STATUS_RBUSY)
			spickle_prepare(I2C_RIGHT_SIDE);
		rspickle = flags;
	}
}

/* set a new state, return 0 on success */
int8_t state_set_mode(uint8_t mode)
{
	state_mode = mode;
	STMCH_DEBUG("%s(): mode=%x", __FUNCTION__, mode);
	return 0;
}

void state_set_i2c_ignore(uint8_t val)
{
	state_i2c_ignore = val;
}

uint8_t state_get_i2c_ignore(void)
{
	return state_i2c_ignore;
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

uint8_t state_get_status(void)
{
	return state_status;
}

/* harvest cobs from area */
static void state_do_harvest(uint8_t side)
{
	/* if there is no cob, return */
	if (cob_falling_edge(side) == 0)
		return;

	if (side == I2C_LEFT_SIDE)
		state_status = I2C_COBBOARD_STATUS_LBUSY;
	else
		state_status = I2C_COBBOARD_STATUS_RBUSY;

	STMCH_DEBUG("start");

	/* eat the cob */
	spickle_pack(side);

	time_wait_ms(200);
	cobroller_on(side);

	/* check that cob is correctly in place */
	if (WAIT_COND_OR_TIMEOUT(state_cob_inside(), 750) == 0) {
		if (state_no_cob_inside()) {
			STMCH_DEBUG("no cob");
			return;
		}
		STMCH_DEBUG("bad cob state");

		/* while cob is not correctly placed try to extract
		 * it as much as we can */
		while (state_cob_inside() == 0 &&
		       state_no_cob_inside() == 0) {
			uint8_t cpt = 0;
			if (cpt == 0)
				cobroller_reverse(side);
			else if (cpt == 1)
				cobroller_off(side);
			else if (cpt == 2)
				cobroller_on(side);
			cpt ++;
			cpt %= 3;
			shovel_mid();
			time_wait_ms(250);
			shovel_down();
			time_wait_ms(250);
		}

		STMCH_DEBUG("cob removed");
		if (state_no_cob_inside()) {
			STMCH_DEBUG("no cob");
			return;
		}
	}

	/* cob is inside, switch off roller */
	cobroller_off(side);
	cob_count ++;

	state_debug_wait_key_pressed();

	/* last cob, nothing to do */
	if (cob_count == 5)
		return;

	/* redeploy the spickle if there is a black cob */
	if (!state_spicklemode_nomove(side))
		spickle_deploy(side);
	state_debug_wait_key_pressed();

	/* let the loaded cobs go */
	servo_door_block();
	servo_carry_open();
	time_wait_ms(100);
	state_debug_wait_key_pressed();

	/* store it */
	shovel_up();

	while (WAIT_COND_OR_TIMEOUT(shovel_is_up(), 600) == 0) {
		STMCH_DEBUG("shovel blocked");
		shovel_down();
		time_wait_ms(250);
		shovel_up();
	}

	state_debug_wait_key_pressed();

	/* close the carry servos */
	servo_carry_close();
	servo_door_close();
	time_wait_ms(200);
	state_debug_wait_key_pressed();

	shovel_down();

	while (WAIT_COND_OR_TIMEOUT(shovel_is_down(), 400) == 0) {
		STMCH_DEBUG("shovel blocked");
		shovel_up();
		time_wait_ms(250);
		shovel_down();
	}

	STMCH_DEBUG("end");
}

/* eject cobs */
static void state_do_eject(void)
{
	state_status = I2C_COBBOARD_STATUS_EJECT;
	cob_count = 0;
	shovel_mid();
	servo_carry_open();
	servo_door_open();
	time_wait_ms(2000);
	shovel_down();
	servo_door_close();
	servo_carry_close();
}

/* main state machine */
void state_machine(void)
{
	while (state_want_exit() == 0) {

		state_status = I2C_COBBOARD_STATUS_READY;

		/* init */
		if (INIT(state_mode)) {
			state_init();
			state_mode = I2C_COBBOARD_MODE_HARVEST;
		}

		if (HARVEST(state_mode)) {
			/* init for each loop */
			shovel_down();
			servo_carry_close();
			servo_door_close();

			/* pack/deply spickles, enable/disable roller */
			cobroller_off(I2C_LEFT_SIDE);
			cobroller_off(I2C_RIGHT_SIDE);
			spickle_prepare(I2C_LEFT_SIDE);
			spickle_prepare(I2C_RIGHT_SIDE);

			/* harvest if not many cobs */
			if (cob_count < 5) {
				if (state_spicklemode_deployed(I2C_LEFT_SIDE) &&
				    state_spicklemode_autoharvest(I2C_LEFT_SIDE))
					state_do_harvest(I2C_LEFT_SIDE);
				if (state_spicklemode_deployed(I2C_RIGHT_SIDE) &&
				    state_spicklemode_autoharvest(I2C_RIGHT_SIDE))
					state_do_harvest(I2C_RIGHT_SIDE);
			}
		}

		/* help to climb the hill */
		if (KICKSTAND_UP(state_mode)) {
			state_status = I2C_COBBOARD_STATUS_KICKSTAND_UP;
			servo_carry_open();
			servo_door_open();
			shovel_kickstand_up();
		}

		if (KICKSTAND_DOWN(state_mode)) {
			state_status = I2C_COBBOARD_STATUS_KICKSTAND_DOWN;
			servo_carry_open();
			servo_door_open();
			shovel_kickstand_down();
		}

		/* eject */
		if (EJECT(state_mode)) {
			state_mode = I2C_COBBOARD_MODE_HARVEST;
			state_do_eject();
		}
	}
	state_status = I2C_COBBOARD_STATUS_OFF;
}

void state_init(void)
{
	vt100_init(&local_vt100);
	shovel_down();
	servo_door_close();
	servo_carry_close();
	spickle_pack(I2C_LEFT_SIDE);
	spickle_pack(I2C_RIGHT_SIDE);
	state_mode = 0;
	cob_count = 0;
	state_status = I2C_COBBOARD_STATUS_OFF;
}
