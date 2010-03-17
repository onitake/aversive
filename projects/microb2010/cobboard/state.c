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
 *  Revision : $Id: state.c,v 1.5 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
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
#include <vt100.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "state.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */
#define INIT               I2C_COBBOARD_MODE_INIT
#define MANUAL             I2C_COBBOARD_MODE_MANUAL
#define HARVEST            I2C_COBBOARD_MODE_HARVEST
#define EXIT               I2C_COBBOARD_MODE_EXIT

static struct i2c_cmd_cobboard_set_mode mainboard_command;
static struct vt100 local_vt100;
static volatile uint8_t prev_state;
static volatile uint8_t changed = 0;

uint8_t state_debug = 0;

void state_dump_sensors(void)
{
	STMCH_DEBUG("TODO\n");
}

void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_cobboard_set_mode *cmd)
{
	changed = 1;
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);
	return 0;
}

/* check that state is the one in parameter and that state did not
 * changed */
uint8_t state_check(uint8_t mode)
{
	int16_t c;
	if (mode != mainboard_command.mode)
		return 0;

	if (changed)
		return 0;

	/* force quit when CTRL-C is typed */
	c = cmdline_getchar();
	if (c == -1)
		return 1;
	if (vt100_parser(&local_vt100, c) == KEY_CTRL_C) {
		mainboard_command.mode = EXIT;
		return 0;
	}
	return 1;
}

uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}

/* manual mode, arm position is sent from mainboard */
static void state_do_manual(void)
{
	if (!state_check(MANUAL))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(MANUAL));
}

/* init mode */
static void state_do_init(void)
{
	if (!state_check(INIT))
		return;
	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(INIT));
}

/* harvest columns elts from area */
static void state_do_harvest(void)
{
	if (!state_check(HARVEST))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(HARVEST));
}
/* main state machine */
void state_machine(void)
{
	while (state_get_mode() != EXIT) {
		changed = 0;
		state_do_init();
		state_do_manual();
		state_do_harvest();
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	mainboard_command.mode = HARVEST;
	cobboard.cob_count = 0;
}
