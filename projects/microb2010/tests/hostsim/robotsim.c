/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: main.c,v 1.9.4.5 2007-06-01 09:37:22 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <aversive.h>
#include <aversive/error.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"

static int32_t l_pwm, r_pwm;
static int32_t l_enc, r_enc;

/* */
#define FILTER  98
#define FILTER2 (100-FILTER)

/* must be called periodically */
void robotsim_update(void)
{
	static int32_t l_speed, r_speed;

	/* XXX should lock */
	l_speed = ((l_speed * FILTER) / 100) + ((l_pwm * 1000 * FILTER2)/1000);
	l_enc += (l_speed / 1000);
	r_speed = ((r_speed * FILTER) / 100) + ((r_pwm * 1000 * FILTER2)/1000);
	r_enc += (r_speed / 1000);
}

void robotsim_pwm(void *arg, int32_t val)
{
	//	printf("%p, %d\n", arg, val);
	if (arg == LEFT_PWM)
		l_pwm = val;
	else if (arg == RIGHT_PWM)
		r_pwm = val;
}

int32_t robotsim_encoder_get(void *arg)
{
	if (arg == LEFT_ENCODER)
		return l_enc;
	else if (arg == RIGHT_ENCODER)
		return r_enc;
	return 0;
}
