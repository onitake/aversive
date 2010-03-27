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
#include "main.h"
#include "actuator.h"

struct spickle_params {
	/* current limit (common to left and right) */
	int32_t k1;
	int32_t k2;

	/* cs blocks */
	struct cs_block * const csb[2];

	/* params */
	int16_t delay_deployed[2];
	int16_t delay_packed[2];
	int32_t pos_deployed[2];
	int32_t pos_packed[2];
};

static struct spickle_params spickle = {
	.k1 = 500,
	.k2 = 20,
	.csb = {
		&cobboard.left_spickle,
		&cobboard.right_spickle,
	},
	.delay_deployed = {
		500, /* left */
		500, /* right */
	},
	.delay_packed = {
		500, /* left */
		500, /* right */
	},
	.pos_deployed = {
		35000, /* left */
		35000, /* right */
	},
	.pos_packed = {
		0, /* left */
		0, /* right */
	},
};

/* init spickle position at beginning */
static void spickle_autopos(void)
{
	pwm_ng_set(LEFT_SPICKLE_PWM, -500);
	//pwm_ng_set(RIGHT_SPICKLE_PWM, -500);
	wait_ms(1000);
	pwm_ng_set(LEFT_SPICKLE_PWM, 0);
	pwm_ng_set(RIGHT_SPICKLE_PWM, 0);
	encoders_spi_set_value(LEFT_SPICKLE_ENCODER, 0);
	encoders_spi_set_value(RIGHT_SPICKLE_ENCODER, 0);
}

/* Set CS command for spickle. Called by CS manager. */
void spickle_set(void *mot, int32_t cmd)
{
	static int32_t oldpos_left, oldpos_right;
	int32_t oldpos, pos, maxcmd, speed;
	
	if (mot == LEFT_SPICKLE_PWM) {
		pos = encoders_spi_get_value(LEFT_SPICKLE_ENCODER);
		oldpos = oldpos_left;
	}
	else {
		pos = encoders_spi_get_value(RIGHT_SPICKLE_ENCODER);
		oldpos = oldpos_right;
	}

	speed = pos - oldpos;
	if (speed > 0 && cmd < 0)
		maxcmd = spickle.k1;
	else if (speed < 0 && cmd > 0)
		maxcmd = spickle.k1;
	else {
		speed = ABS(speed);
		maxcmd = spickle.k1 + spickle.k2 * speed;
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

void spickle_set_coefs(uint32_t k1, uint32_t k2)
{
	spickle.k1 = k1;
	spickle.k2 = k2;
}

void spickle_set_pos(uint8_t side, uint32_t pos_deployed, uint32_t pos_packed)
{
	spickle.pos_deployed[side] = pos_deployed;
	spickle.pos_packed[side] = pos_packed;
}

void spickle_set_delay(uint8_t side, uint32_t delay_deployed, uint32_t delay_packed)
{
	spickle.delay_deployed[side] = delay_deployed;
	spickle.delay_packed[side] = delay_packed;
}

void spickle_dump_params(void)
{
	printf_P(PSTR("coef %ld %ld\r\n"), spickle.k1, spickle.k2);
	printf_P(PSTR("left pos %ld %ld\r\n"),
		 spickle.pos_deployed[I2C_LEFT_SIDE],
		 spickle.pos_packed[I2C_LEFT_SIDE]);
	printf_P(PSTR("left delay %ld %ld\r\n"),
		 spickle.delay_deployed[I2C_LEFT_SIDE],
		 spickle.delay_packed[I2C_LEFT_SIDE]);
	printf_P(PSTR("right pos %ld %ld\r\n"),
		 spickle.pos_deployed[I2C_RIGHT_SIDE],
		 spickle.pos_packed[I2C_RIGHT_SIDE]);
	printf_P(PSTR("right delay %ld %ld\r\n"),
		 spickle.delay_deployed[I2C_RIGHT_SIDE],
		 spickle.delay_packed[I2C_RIGHT_SIDE]);
}

void spickle_deploy(uint8_t side)
{
	cs_set_consign(&spickle.csb[side]->cs, spickle.pos_deployed[side]);
}

void spickle_pack(uint8_t side)
{
	cs_set_consign(&spickle.csb[side]->cs, spickle.pos_deployed[side]);
}

uint16_t spickle_get_deploy_delay(uint8_t side)
{
	return spickle.delay_deployed[side];
}

uint16_t spickle_get_pack_delay(uint8_t side)
{
	return spickle.delay_packed[side];
}

void spickle_init(void)
{
	spickle_autopos();
	cobboard.left_spickle.on = 1;
	//cobboard.right_spickle.on = 1;
}
