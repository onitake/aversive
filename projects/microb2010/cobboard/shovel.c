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
#include "shovel.h"

#define SHOVEL_DOWN 100
#define SHOVEL_MID  4900
#define SHOVEL_UP   10000

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

void shovel_down(void)
{
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 50, 80);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_DOWN);
}

void shovel_mid(void)
{
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 80, 80);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_MID);
}

void shovel_up(void)
{
	quadramp_set_2nd_order_vars(&cobboard.shovel.qr, 80, 20);
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_UP);
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
