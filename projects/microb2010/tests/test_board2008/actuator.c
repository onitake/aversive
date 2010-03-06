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
 *  Revision : $Id: actuator.c,v 1.3 2009-05-02 10:08:09 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <encoders_microb.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>

#include "sensor.h"
#include "main.h"

#define OFF 0
#define WAIT_SENSOR 1
#define SENSOR_OK 2
#define WAIT_DOWN 3

static volatile uint8_t fessor_state = OFF;
static volatile uint32_t fessor_pos_up =  35000;
static volatile uint32_t fessor_pos_down = 0;
static volatile uint32_t fessor_delay_up = 500;
static volatile uint32_t fessor_delay_down = 2000;
static volatile uint32_t delay = 0;
static volatile int32_t fessor_k1 = 500, fessor_k2 = 20;
static volatile int32_t fessor_cmd = 0;


static volatile uint8_t elevator_state = OFF;
static volatile uint32_t elevator_pos_up =  -8000;
static volatile uint32_t elevator_pos_down = 0;
static volatile uint32_t elevator_delay_up = 500;
static volatile uint32_t elevator_delay_down = 2000;
//static volatile uint32_t delay = 0;
static volatile int32_t elevator_k1 = 500, elevator_k2 = 20;
static volatile int32_t elevator_cmd = 0;

void pwm_set_and_save(void *pwm, int32_t val)
{
	/* we need to do the saturation here, before saving the
	 * value */
	if (val > 4095)
		val = 4095;
	if (val < -4095)
		val = -4095;
	
	if (pwm == LEFT_PWM)
		mainboard.pwm_l = val;
	else if (pwm == RIGHT_PWM)
		mainboard.pwm_r = val;
	pwm_ng_set(pwm, val);
}

void pickup_wheels_on(void)
{
	mainboard.enable_pickup_wheels = 1;
}

void pickup_wheels_off(void)
{
	mainboard.enable_pickup_wheels = 0;
}

/* init fessor position at beginning */
void fessor_autopos(void)
{
	pwm_ng_set(FESSOR_PWM, -500);
	wait_ms(3000);
	pwm_ng_set(FESSOR_PWM, 0);
}

/* set CS command for fessor */
void fessor_set(void *dummy, int32_t cmd)
{
	fessor_cmd = cmd;
}

void fessor_set_coefs(uint32_t k1, uint32_t k2)
{
	fessor_k1 = k1;
	fessor_k2 = k2;
}

void fessor_set_delays(uint32_t delay_up, uint32_t delay_down)
{
	fessor_delay_up = delay_up;
	fessor_delay_down = delay_down;
}

void fessor_set_pos(uint32_t pos_up, uint32_t pos_down)
{
	fessor_pos_up = pos_up;
	fessor_pos_down = pos_down;
}

void fessor_dump_params(void)
{
	printf_P(PSTR("coef %ld %ld\r\n"), fessor_k1, fessor_k2);
	printf_P(PSTR("pos %ld %ld\r\n"), fessor_pos_up, fessor_pos_down);
	printf_P(PSTR("delay %ld %ld\r\n"), fessor_delay_up, fessor_delay_down);
}

/* called by CS perdiodically (current limit) */
void fessor_manage(void)
{
	static int32_t oldpos = 0;
	int32_t pos, maxcmd, speed, cmd;
	
	cmd = fessor_cmd;
	pos = encoders_microb_get_value(FESSOR_ENC);
	speed = pos - oldpos;
	if (speed > 0 && cmd < 0)
		maxcmd = fessor_k1;
	else if (speed < 0 && cmd > 0)
		maxcmd = fessor_k1;
	else {
		speed = ABS(speed);
		maxcmd = fessor_k1 + fessor_k2 * speed;
	}
	if (cmd > maxcmd)
		cmd = maxcmd;
	else if (cmd < -maxcmd)
		cmd = -maxcmd;

	pwm_ng_set(FESSOR_PWM, cmd);

	oldpos = pos;
}

void fessor_up(void)
{
	fessor_state = 0;
	cs_set_consign(&mainboard.fessor.cs, fessor_pos_up);
}

void fessor_down(void)
{
	fessor_state = 0;
	cs_set_consign(&mainboard.fessor.cs, fessor_pos_down);
}

void fessor_stop(void)
{
	fessor_state = 0;
}

void fessor_auto(void)
{
	fessor_state = WAIT_SENSOR;
	cs_set_consign(&mainboard.fessor.cs, fessor_pos_up);
}

/* for fessor auto mode */
static void fessor_cb(void *dummy)
{
	static uint8_t prev = 0;
	uint8_t val;

	val = !sensor_get(0);

	switch (fessor_state) {
	case OFF:
		break;
	case WAIT_SENSOR:
		if (val && !prev) {
			delay = fessor_delay_up;
			fessor_state = SENSOR_OK;
		}
		break;
	case SENSOR_OK:
		if (delay-- == 0) {
			cs_set_consign(&mainboard.fessor.cs, fessor_pos_down);
			fessor_state = WAIT_DOWN;
			delay = fessor_delay_down;
		}
		break;
	case WAIT_DOWN:
		if (delay-- == 0) {
			cs_set_consign(&mainboard.fessor.cs, fessor_pos_up);
			fessor_state = WAIT_SENSOR;
		}
		break;
	default:
		break;
	}
	prev = val;
}

void fessor_init(void)
{
	scheduler_add_periodical_event_priority(fessor_cb, NULL, 
						1000L / SCHEDULER_UNIT, 
						FESSOR_PRIO);
}








/* init elevator position at beginning */
void elevator_autopos(void)
{
	pwm_ng_set(ELEVATOR_PWM, 300);
	wait_ms(4000);
	pwm_ng_set(ELEVATOR_PWM, 0);
}

/* set CS command for elevator */
void elevator_set(void *dummy, int32_t cmd)
{
	elevator_cmd = cmd;
}

void elevator_set_coefs(uint32_t k1, uint32_t k2)
{
	elevator_k1 = k1;
	elevator_k2 = k2;
}

void elevator_set_delays(uint32_t delay_up, uint32_t delay_down)
{
	elevator_delay_up = delay_up;
	elevator_delay_down = delay_down;
}

void elevator_set_pos(uint32_t pos_up, uint32_t pos_down)
{
	elevator_pos_up = pos_up;
	elevator_pos_down = pos_down;
}

void elevator_dump_params(void)
{
	printf_P(PSTR("coef %ld %ld\r\n"), elevator_k1, elevator_k2);
	printf_P(PSTR("pos %ld %ld\r\n"), elevator_pos_up, elevator_pos_down);
	printf_P(PSTR("delay %ld %ld\r\n"), elevator_delay_up, elevator_delay_down);
}

/* called by CS perdiodically (current limit) */
void elevator_manage(void)
{
	static int32_t oldpos = 0;
	int32_t pos, maxcmd, speed, cmd;
	
	cmd = elevator_cmd;
	pos = encoders_microb_get_value(ELEVATOR_ENC);
	speed = pos - oldpos;
	if (speed > 0 && cmd < 0)
		maxcmd = elevator_k1;
	else if (speed < 0 && cmd > 0)
		maxcmd = elevator_k1;
	else {
		speed = ABS(speed);
		maxcmd = elevator_k1 + elevator_k2 * speed;
	}
	if (cmd > maxcmd)
		cmd = maxcmd;
	else if (cmd < -maxcmd)
		cmd = -maxcmd;

	pwm_ng_set(ELEVATOR_PWM, cmd);

	oldpos = pos;
}

void elevator_up(void)
{
	elevator_state = 0;
	cs_set_consign(&mainboard.elevator.cs, elevator_pos_up);
}

void elevator_down(void)
{
	elevator_state = 0;
	cs_set_consign(&mainboard.elevator.cs, elevator_pos_down);
}

void elevator_stop(void)
{
	elevator_state = 0;
}

void elevator_auto(void)
{
	elevator_state = WAIT_SENSOR;
	cs_set_consign(&mainboard.elevator.cs, elevator_pos_down);
}

/* for elevator auto mode */
static void elevator_cb(void *dummy)
{
	static uint8_t prev = 0;
	uint8_t val;

	val = !sensor_get(0);

	switch (elevator_state) {
	case OFF:
		break;
	case WAIT_SENSOR:
		if (val && !prev) {
			delay = elevator_delay_up;
			elevator_state = SENSOR_OK;
		}
		break;
	case SENSOR_OK:
		if (delay-- == 0) {
			cs_set_consign(&mainboard.elevator.cs, elevator_pos_up);
			elevator_state = WAIT_DOWN;
			delay = elevator_delay_down;
		}
		break;
	case WAIT_DOWN:
		if (delay-- == 0) {
			cs_set_consign(&mainboard.elevator.cs, elevator_pos_down);
			elevator_state = WAIT_SENSOR;
		}
		break;
	default:
		break;
	}
	prev = val;
}

void elevator_init(void)
{
	scheduler_add_periodical_event_priority(elevator_cb, NULL, 
						1000L / SCHEDULER_UNIT, 
						ELEVATOR_PRIO);
}
