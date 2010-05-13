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
 *  Revision : $Id: actuator.c,v 1.4 2009-04-24 19:30:41 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/pgmspace.h>
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

#include "sensor.h"
#include "../common/i2c_commands.h"
#include "state.h"
#include "main.h"
#include "actuator.h"

struct spickle_params {
	/* current limit (common to left and right) */
	int32_t sk1;
	int32_t sk2;
	int32_t wk1;
	int32_t wk2;

	/* cs blocks */
	struct cs_block * const csb[2];

	/* params */
	int32_t pos_deployed[2];
	int32_t pos_mid[2];
	int32_t pos_packed[2];
};

static struct spickle_params spickle = {
	.sk1 = 1000,
	.sk2 = 20,
	.wk1 = 200,
	.wk2 = 5,
	.csb = {
		&cobboard.left_spickle,
		&cobboard.right_spickle,
	},
	.pos_deployed = {
		7000, // 200, /* left */
		7000, // 200, /* right */
	},
	.pos_mid = {
		25000, /* left */
		26000, /* right */
	},
	.pos_packed = {
		55800, /* left */
		55800, /* right */
	},
};

/* init spickle position at beginning */
static void spickle_autopos(void)
{
	printf_P(PSTR("spickle autopos..."));
	pwm_ng_set(LEFT_SPICKLE_PWM, -700);
	pwm_ng_set(RIGHT_SPICKLE_PWM, -700);
	wait_ms(2500);
	pwm_ng_set(LEFT_SPICKLE_PWM, 0);
	pwm_ng_set(RIGHT_SPICKLE_PWM, 0);
	encoders_spi_set_value(LEFT_SPICKLE_ENCODER, 0);
	encoders_spi_set_value(RIGHT_SPICKLE_ENCODER, 0);
	printf_P(PSTR("ok\r\n"));
}

/* Set CS command for spickle. Called by CS manager. */
void spickle_set(void *mot, int32_t cmd)
{
	static int32_t oldpos_left, oldpos_right;
	int32_t oldpos, pos, maxcmd, speed;
	int32_t k1, k2;

	if (mot == LEFT_SPICKLE_PWM) {
		pos = encoders_spi_get_value(LEFT_SPICKLE_ENCODER);
		oldpos = oldpos_left;
		if (state_spicklemode_weak(I2C_LEFT_SIDE)) {
			k1 = spickle.wk1;
			k2 = spickle.wk2;
		}
		else {
			k1 = spickle.sk1;
			k2 = spickle.sk2;
		}
	}
	else {
		pos = encoders_spi_get_value(RIGHT_SPICKLE_ENCODER);
		oldpos = oldpos_right;
		if (state_spicklemode_weak(I2C_RIGHT_SIDE)) {
			k1 = spickle.wk1;
			k2 = spickle.wk2;
		}
		else {
			k1 = spickle.sk1;
			k2 = spickle.sk2;
		}
	}

	speed = pos - oldpos;
	if (speed > 0 && cmd < 0)
		maxcmd = k1;
	else if (speed < 0 && cmd > 0)
		maxcmd = k1;
	else {
		speed = ABS(speed);
		maxcmd = k1 + k2 * speed;
	}
	if (cmd > maxcmd)
		cmd = maxcmd;
	else if (cmd < -maxcmd)
		cmd = -maxcmd;

	pwm_ng_set(mot, cmd);

	if (mot == LEFT_SPICKLE_PWM)
		oldpos_left = pos;
	else
		oldpos_right = pos;
}

void spickle_set_wcoefs(uint32_t k1, uint32_t k2)
{
	spickle.wk1 = k1;
	spickle.wk2 = k2;
}

void spickle_set_scoefs(uint32_t k1, uint32_t k2)
{
	spickle.sk1 = k1;
	spickle.sk2 = k2;
}

void spickle_set_pos(uint8_t side, int32_t pos_packed,
		     int32_t pos_mid, int32_t pos_deployed)
{
	spickle.pos_deployed[side] = pos_deployed;
	spickle.pos_mid[side] = pos_mid;
	spickle.pos_packed[side] = pos_packed;
}

void spickle_dump_params(void)
{
	printf_P(PSTR("strong_coef %ld %ld\r\n"), spickle.sk1, spickle.sk2);
	printf_P(PSTR("weak_coef %ld %ld\r\n"), spickle.wk1, spickle.wk2);
	printf_P(PSTR("left pos %ld %ld\r\n"),
		 spickle.pos_packed[I2C_LEFT_SIDE],
		 spickle.pos_mid[I2C_LEFT_SIDE],
		 spickle.pos_deployed[I2C_LEFT_SIDE]);
	printf_P(PSTR("right pos %ld %ld\r\n"),
		 spickle.pos_packed[I2C_RIGHT_SIDE],
		 spickle.pos_mid[I2C_RIGHT_SIDE],
		 spickle.pos_deployed[I2C_RIGHT_SIDE]);
}

static uint8_t spickle_is_at_pos(uint8_t side, int32_t pos)
{
	int32_t diff;
	int32_t enc;
	if (side == I2C_LEFT_SIDE)
		enc = encoders_spi_get_value(LEFT_SPICKLE_ENCODER);
	else
		enc = encoders_spi_get_value(RIGHT_SPICKLE_ENCODER);
	diff = pos - enc;
	if (diff < 0)
		diff = -diff;
	if (diff < 500)
		return 1;
	return 0;
}

static void spickle_set_qr(uint8_t side)
{
	struct quadramp_filter *qr;

	if (side == I2C_LEFT_SIDE)
		qr = &cobboard.left_spickle.qr;
	else
		qr = &cobboard.right_spickle.qr;

	if (state_spicklemode_weak(side))
		quadramp_set_1st_order_vars(qr, 700, 700); /* set speed */
	else
		quadramp_set_1st_order_vars(qr, 3000, 3000); /* set speed */
}

uint8_t spickle_is_packed(uint8_t side)
{
	return spickle_is_at_pos(side, spickle.pos_packed[side]);
}

uint8_t spickle_is_deployed(uint8_t side)
{
	return spickle_is_at_pos(side, spickle.pos_deployed[side]);
}

void spickle_deploy(uint8_t side)
{
	spickle_set_qr(side);
	cs_set_consign(&spickle.csb[side]->cs, spickle.pos_deployed[side]);
}

void spickle_mid(uint8_t side)
{
	spickle_set_qr(side);
	cs_set_consign(&spickle.csb[side]->cs, spickle.pos_mid[side]);
}

void spickle_pack(uint8_t side)
{
	spickle_set_qr(side);
	cs_set_consign(&spickle.csb[side]->cs, spickle.pos_packed[side]);
}

void spickle_init(void)
{
	spickle_autopos();
	cobboard.left_spickle.on = 1;
	cobboard.right_spickle.on = 1;
}
