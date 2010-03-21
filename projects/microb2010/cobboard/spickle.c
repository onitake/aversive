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
#include "actuator.h"
#include "main.h"


#define OFF 0
#define WAIT_SENSOR 1
#define SENSOR_OK 2
#define WAIT_DOWN 3

static volatile uint8_t spickle_state = OFF;
static volatile uint32_t spickle_pos_up =  35000;
static volatile uint32_t spickle_pos_down = 0;
static volatile uint32_t spickle_delay_up = 500;
static volatile uint32_t spickle_delay_down = 2000;
static volatile uint32_t delay = 0;
static volatile int32_t spickle_k1 = 500, spickle_k2 = 20;
static volatile int32_t spickle_cmd = 0;

/* init spickle position at beginning */
static void spickle_autopos(void)
{
	pwm_ng_set(LEFT_SPICKLE_PWM, -500);
	wait_ms(3000);
	pwm_ng_set(LEFT_SPICKLE_PWM, 0);
	encoders_spi_set_value(LEFT_SPICKLE_ENCODER, 0);
}

/* set CS command for spickle */
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
		maxcmd = spickle_k1;
	else if (speed < 0 && cmd > 0)
		maxcmd = spickle_k1;
	else {
		speed = ABS(speed);
		maxcmd = spickle_k1 + spickle_k2 * speed;
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
	spickle_k1 = k1;
	spickle_k2 = k2;
}

void spickle_set_delays(uint32_t delay_up, uint32_t delay_down)
{
	spickle_delay_up = delay_up;
	spickle_delay_down = delay_down;
}

void spickle_set_pos(uint32_t pos_up, uint32_t pos_down)
{
	spickle_pos_up = pos_up;
	spickle_pos_down = pos_down;
}

void spickle_dump_params(void)
{
	printf_P(PSTR("coef %ld %ld\r\n"), spickle_k1, spickle_k2);
	printf_P(PSTR("pos %ld %ld\r\n"), spickle_pos_up, spickle_pos_down);
	printf_P(PSTR("delay %ld %ld\r\n"), spickle_delay_up, spickle_delay_down);
}

void spickle_up(void)
{
	spickle_state = 0;
	cs_set_consign(&cobboard.left_spickle.cs, spickle_pos_up);
}

void spickle_down(void)
{
	spickle_state = 0;
	cs_set_consign(&cobboard.left_spickle.cs, spickle_pos_down);
}

void spickle_stop(void)
{
	spickle_state = 0;
}

void spickle_auto(void)
{
	spickle_state = WAIT_SENSOR;
	cs_set_consign(&cobboard.left_spickle.cs, spickle_pos_up);
}

/* for spickle auto mode */
static void spickle_cb(__attribute__((unused)) void *dummy)
{
	static uint8_t prev = 0;
	uint8_t val;

	val = sensor_get(S_LCOB);

	switch (spickle_state) {
	case OFF:
		break;
	case WAIT_SENSOR:
		if (val && !prev) {
			delay = spickle_delay_up;
			spickle_state = SENSOR_OK;
		}
		break;
	case SENSOR_OK:
		if (delay-- == 0) {
			cs_set_consign(&cobboard.left_spickle.cs, spickle_pos_down);
			spickle_state = WAIT_DOWN;
			delay = spickle_delay_down;
		}
		break;
	case WAIT_DOWN:
		if (delay-- == 0) {
			cs_set_consign(&cobboard.left_spickle.cs, spickle_pos_up);
			spickle_state = WAIT_SENSOR;
		}
		break;
	default:
		break;
	}
	prev = val;
}

void spickle_init(void)
{
	spickle_autopos();

	scheduler_add_periodical_event_priority(spickle_cb, NULL, 
						1000L / SCHEDULER_UNIT, 
						SPICKLE_PRIO);
}
