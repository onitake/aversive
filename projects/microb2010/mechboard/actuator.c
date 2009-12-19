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
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "main.h"

#define FINGER_DEBUG(args...) DEBUG(E_USER_FINGER, args)
#define FINGER_NOTICE(args...) NOTICE(E_USER_FINGER, args)
#define FINGER_ERROR(args...) ERROR(E_USER_FINGER, args)

struct finger {
	int8_t event;
	uint16_t destination;
};

struct finger finger;

static void finger_goto_cb(void *);

/* schedule a single event for the finger */
static void finger_schedule_event(struct finger *finger)
{
	uint8_t flags;
	int8_t ret;

	IRQ_LOCK(flags);
	ret = scheduler_add_event(SCHEDULER_SINGLE,
				  (void *)finger_goto_cb,
				  finger, 1, ARM_PRIO);
	if (ret == -1) {
		IRQ_UNLOCK(flags);
		FINGER_ERROR("Cannot load finger event");
		return;
	}
	finger->event = ret;
	IRQ_UNLOCK(flags);
}

static void finger_goto_cb(void *data)
{
	uint8_t flags;
	struct finger *finger = data;
	uint16_t position;

	IRQ_LOCK(flags);
	finger->event = -1;
	position = finger->destination;
	IRQ_UNLOCK(flags);
	FINGER_DEBUG("goto_cb %d", position);
	ax12_user_write_int(&gen.ax12, FINGER_AX12,
			    AA_GOAL_POSITION_L, position);
}

/* load an event that will move the ax12 for us */
void finger_goto(uint16_t position)
{
	uint8_t flags;
	FINGER_NOTICE("goto %d", position);

	IRQ_LOCK(flags);
	finger.destination = position;
	if (finger.event != -1) {
		IRQ_UNLOCK(flags);
		return; /* nothing to do, event already scheduled */
	}
	IRQ_UNLOCK(flags);
	finger_schedule_event(&finger);
}

static void finger_init(void)
{
	finger.event = -1;
	finger.destination = 0;
	/* XXX set pos ? */
}

uint16_t finger_get_goal_pos(void)
{
	return finger.destination;
}

uint8_t finger_get_side(void)
{
	if (finger.destination <= FINGER_CENTER)
		return I2C_LEFT_SIDE;
	return I2C_RIGHT_SIDE;
}

/**********/

#define SERVO_LEFT_OUT  400
#define SERVO_LEFT_1LIN 520
#define SERVO_LEFT_2LIN 485
#define SERVO_RIGHT_OUT  290
#define SERVO_RIGHT_1LIN 155
#define SERVO_RIGHT_2LIN 180

void servo_lintel_out(void)
{
	mechboard.servo_lintel_left = SERVO_LEFT_OUT;
	mechboard.servo_lintel_right = SERVO_RIGHT_OUT;
}

void servo_lintel_1lin(void)
{
	mechboard.servo_lintel_left = SERVO_LEFT_1LIN;
	mechboard.servo_lintel_right = SERVO_RIGHT_1LIN;
}

void servo_lintel_2lin(void)
{
	mechboard.servo_lintel_left = SERVO_LEFT_2LIN;
	mechboard.servo_lintel_right = SERVO_RIGHT_2LIN;
}

/**********/

void actuator_init(void)
{
	finger_init();
	servo_lintel_out();
}
