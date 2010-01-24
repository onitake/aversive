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
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>

#include "main.h"
#include "robotsim.h"

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
#ifdef HOST_VERSION
	robotsim_pwm(pwm, val);
#else
	pwm_ng_set(pwm, val);
#endif
}

void pickup_wheels_on(void)
{
	mainboard.enable_pickup_wheels = 1;
}

void pickup_wheels_off(void)
{
	mainboard.enable_pickup_wheels = 0;
}

