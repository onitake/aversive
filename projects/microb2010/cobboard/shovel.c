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

#include "main.h"
#include "state.h"
#include "shovel.h"

#define SHOVEL_DOWN 100
#define SHOVEL_MID  4500
#define SHOVEL_UP   11300
#define SHOVEL_KICKSTAND_UP   12800
#define SHOVEL_KICKSTAND_DOWN 10000

static int32_t shovel_k1 = 1000;
static int32_t shovel_k2 = 20;
static uint8_t shovel_current_limit = 1;

/* init spickle position at beginning */
static void shovel_autopos(void)
{
	printf_P(PSTR("shovel autopos..."));
	pwm_ng_set(SHOVEL_PWM, -500);
	wait_ms(1000);
	pwm_ng_set(LEFT_SPICKLE_PWM, 0);
	encoders_spi_set_value(SHOVEL_ENCODER, 0);
	printf_P(PSTR("ok\r\n"));
}

static uint8_t shovel_is_at_pos(int32_t pos)
{
	int32_t diff;
	diff = pos - encoders_spi_get_value(SHOVEL_ENCODER);
	if (diff < 0)
		diff = -diff;
	if (diff < 500)
		return 1;
	return 0;
}

void shovel_set_current_limit_coefs(int32_t k1, int32_t k2)
{
	shovel_k1 = k1;
	shovel_k2 = k2;
}

uint8_t shovel_get_current_limit_coefs(int32_t *k1, int32_t *k2)
{
	*k1 = shovel_k1;
	*k2 = shovel_k2;
	return shovel_current_limit;
}

void shovel_current_limit_enable(uint8_t enable)
{
	shovel_current_limit = enable;
}

/* Set CS command for shovel. Called by CS manager. */
void shovel_set(void *mot, int32_t cmd)
{
	static int32_t oldpos;
	int32_t pos, maxcmd, speed;

	pos = encoders_spi_get_value(SHOVEL_ENCODER);
	if (shovel_current_limit) {
		speed = pos - oldpos;
		if (speed > 0 && cmd < 0)
			maxcmd = shovel_k1;
		else if (speed < 0 && cmd > 0)
			maxcmd = shovel_k1;
		else {
			speed = ABS(speed);
			maxcmd = shovel_k1 + shovel_k2 * speed;
		}
		if (cmd > maxcmd)
			cmd = maxcmd;
		else if (cmd < -maxcmd)
			cmd = -maxcmd;
	}

	pwm_ng_set(mot, cmd);
	oldpos = pos;
}

void shovel_down(void)
{
	shovel_current_limit_enable(0);
	quadramp_set_1st_order_vars(&cobboard.shovel.qr, 2500, 2500);
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 50, 80);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_DOWN);
}

void shovel_mid(void)
{
	shovel_current_limit_enable(0);
	quadramp_set_1st_order_vars(&cobboard.shovel.qr, 2500, 2500);
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 80, 80);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_MID);
}

void shovel_up(void)
{
	shovel_current_limit_enable(0);
	if (state_get_cob_count() <= 1)
		quadramp_set_1st_order_vars(&cobboard.shovel.qr, 1000, 2500);
	else
		quadramp_set_1st_order_vars(&cobboard.shovel.qr, 2000, 2500);
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 80, 15);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_UP);
}

void shovel_kickstand_up(void)
{
	shovel_set_current_limit_coefs(1000, 20);
	shovel_current_limit_enable(1);
	quadramp_set_1st_order_vars(&cobboard.shovel.qr, 200, 200);
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 10, 10);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_KICKSTAND_UP);
}

void shovel_kickstand_down(void)
{
	shovel_set_current_limit_coefs(500, 0);
	shovel_current_limit_enable(1);
	quadramp_set_1st_order_vars(&cobboard.shovel.qr, 200, 200);
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 10, 10);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_KICKSTAND_DOWN);
}

uint8_t shovel_is_up(void)
{
	return shovel_is_at_pos(SHOVEL_UP);
}

uint8_t shovel_is_down(void)
{
	return shovel_is_at_pos(SHOVEL_DOWN);
}

void shovel_init(void)
{
	shovel_autopos();
	cobboard.shovel.on = 1;
}
