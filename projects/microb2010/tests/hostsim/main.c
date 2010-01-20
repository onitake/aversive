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

#include <aversive.h>
#include <aversive/error.h>
#include <aversive/wait.h>

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

#include <uart.h>
//#include <timer.h>
#include <hostsim.h>

#include "cs.h"
#include "main.h"

struct genboard gen;
struct mainboard mainboard;

int main(void)
{
#ifndef HOST_VERSION
	uart_init();
	fdevopen(uart0_dev_send, uart0_dev_recv);
	sei();
#else
	int i;
#endif

#ifdef CONFIG_MODULE_TIMER
	timer_init();
#endif
	scheduler_init();
	printf("init\n");

#ifdef HOST_VERSION
	hostsim_init();
	robotsim_init();
#endif
	time_init(TIME_PRIO);


	microb_cs_init();

 	gen.logs[0] = E_USER_CS;
 	gen.log_level = 5;

	mainboard.flags = DO_ENCODERS | DO_RS |
		DO_POS | DO_POWER | DO_BD | DO_CS;

	trajectory_d_rel(&mainboard.traj, 1000);
	time_wait_ms(2000);
	trajectory_goto_xy_abs(&mainboard.traj, 1500, 2000);
	time_wait_ms(2000);
	return 0;
}


