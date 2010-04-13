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
 *  Revision : $Id: actuator.c,v 1.2 2009-04-24 19:30:42 zer0 Exp $
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

#define ROLLER_ON      -1200
#define ROLLER_OFF     0
#define ROLLER_REVERSE 1200

#define FORKROT_DEPLOYED -50000
#define FORKROT_PACKED   0

#define FORKTRANS_LEFT   0
#define FORKTRANS_MIDDLE 500
#define FORKTRANS_RIGHT  1000

void roller_on(void)
{
	cs_set_consign(&ballboard.roller.cs, ROLLER_ON);
}

void roller_off(void)
{
	cs_set_consign(&ballboard.roller.cs, ROLLER_OFF);
}

void roller_reverse(void)
{
	cs_set_consign(&ballboard.roller.cs, ROLLER_REVERSE);
}

void fork_deploy(void)
{
	cs_set_consign(&ballboard.forkrot.cs, FORKROT_DEPLOYED);
}

void fork_pack(void)
{
	cs_set_consign(&ballboard.forkrot.cs, FORKROT_PACKED);
}

void fork_left(void)
{
	cs_set_consign(&ballboard.forktrans.cs, FORKTRANS_LEFT);
}

void fork_right(void)
{
	cs_set_consign(&ballboard.forktrans.cs, FORKTRANS_RIGHT);
}

void fork_middle(void)
{
	cs_set_consign(&ballboard.forktrans.cs, FORKTRANS_MIDDLE);
}
