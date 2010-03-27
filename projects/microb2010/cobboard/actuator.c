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

#define COBROLLER_SPEED 800

#define SERVO_DOOR_PWM ((void *)&gen.servo2)
#define SERVO_DOOR_OPEN 250
#define SERVO_DOOR_CLOSED 470

void actuator_init(void);

void servo_carry_open(void)
{
	/* TODO */
}

void servo_carry_close(void)
{
	/* TODO */
}

void servo_door_open(void)
{
	pwm_ng_set(SERVO_DOOR_PWM, SERVO_DOOR_OPEN);
}

void servo_door_close(void)
{
	pwm_ng_set(SERVO_DOOR_PWM, SERVO_DOOR_CLOSED);
}

void left_cobroller_on(void)
{
	cobboard.left_cobroller_speed = COBROLLER_SPEED;
}

void right_cobroller_on(void)
{
	cobboard.right_cobroller_speed = COBROLLER_SPEED;
}

void left_cobroller_off(void)
{
	cobboard.left_cobroller_speed = 0;
}

void right_cobroller_off(void)
{
	cobboard.right_cobroller_speed = 0;
}

void actuator_init(void)
{

}
