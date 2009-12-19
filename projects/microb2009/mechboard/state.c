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
#include <time.h>

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
#include "arm_xy.h"
#include "arm_highlevel.h"
#include "state.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */
#define MANUAL             I2C_MECHBOARD_MODE_MANUAL
#define HARVEST            I2C_MECHBOARD_MODE_HARVEST
#define PREPARE_PICKUP     I2C_MECHBOARD_MODE_PREPARE_PICKUP
#define PICKUP             I2C_MECHBOARD_MODE_PICKUP
#define PREPARE_BUILD      I2C_MECHBOARD_MODE_PREPARE_BUILD
#define AUTOBUILD          I2C_MECHBOARD_MODE_AUTOBUILD
#define WAIT               I2C_MECHBOARD_MODE_WAIT
#define INIT               I2C_MECHBOARD_MODE_INIT
#define PREPARE_GET_LINTEL I2C_MECHBOARD_MODE_PREPARE_GET_LINTEL
#define GET_LINTEL         I2C_MECHBOARD_MODE_GET_LINTEL
#define PUT_LINTEL         I2C_MECHBOARD_MODE_PUT_LINTEL
#define PREPARE_EJECT      I2C_MECHBOARD_MODE_PREPARE_EJECT
#define EJECT              I2C_MECHBOARD_MODE_EJECT
#define CLEAR              I2C_MECHBOARD_MODE_CLEAR
#define LAZY_HARVEST       I2C_MECHBOARD_MODE_LAZY_HARVEST
#define LOADED             I2C_MECHBOARD_MODE_LOADED
#define PREPARE_INSIDE     I2C_MECHBOARD_MODE_PREPARE_INSIDE
#define STORE              I2C_MECHBOARD_MODE_STORE
#define LAZY_PICKUP        I2C_MECHBOARD_MODE_LAZY_PICKUP
#define MANIVELLE          I2C_MECHBOARD_MODE_MANIVELLE
#define PUSH_TEMPLE        I2C_MECHBOARD_MODE_PUSH_TEMPLE
#define PUSH_TEMPLE_DISC   I2C_MECHBOARD_MODE_PUSH_TEMPLE_DISC
#define EXIT               I2C_MECHBOARD_MODE_EXIT

static void state_do_eject(uint8_t arm_num, uint8_t pump_num, uint8_t old_mode);

static struct i2c_cmd_mechboard_set_mode mainboard_command;
static struct vt100 local_vt100;
static volatile uint8_t prev_state;
static uint8_t pickup_side;
static volatile uint8_t changed = 0;

uint8_t state_debug = 0;

void state_dump_sensors(void)
{
	uint16_t tmp = sensor_get_all();
	prog_char *front = PSTR("no_front");
	prog_char *left = PSTR("no_left");
	prog_char *right = PSTR("no_right");
	
	if (tmp & _BV(S_FRONT))
		front = PSTR("FRONT");
	if (tmp & _BV(S_LEFT)) {
		if (tmp & _BV(S_COL_LEFT))
			left = PSTR("LEFT(red)");
		else
			left = PSTR("LEFT(green)");
	}
	if (tmp & _BV(S_RIGHT)) {
		if (tmp & _BV(S_COL_RIGHT))
			right = PSTR("RIGHT(red)");
		else
			right = PSTR("RIGHT(green)");
	}
	
	STMCH_DEBUG("sensors = %S %S %S", front, left, right);
}

/* return 1 if column is there */
uint8_t arm_get_sensor(uint8_t arm_num)
{
	if (arm_num == ARM_LEFT_NUM) {
		return sensor_get(S_LEFT);
	}
	else if (arm_num == ARM_RIGHT_NUM) {
		return sensor_get(S_RIGHT);
	}
	return 0;
}

/* return 0 if color is correct, else return -1 */
int8_t arm_get_color_sensor(uint8_t arm_num)
{
	uint8_t col = 0;
	if (arm_num == ARM_LEFT_NUM) {
		col = sensor_get(S_COL_LEFT);
	}
	else if (arm_num == ARM_RIGHT_NUM) {
		col = sensor_get(S_COL_RIGHT);
	}

	/* if col != 0, column is red */
	if (col) {
		if (mechboard.our_color == I2C_COLOR_RED)
			return 0;
		return -1;
	}
	else {
		if (mechboard.our_color == I2C_COLOR_GREEN)
			return 0;
		return -1;
	}
}

void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_mechboard_set_mode *cmd)
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

void pump_reset_all(void)
{
	uint8_t i;
	for (i=0; i<4; i++) {
		pump_set(i, PUMP_OFF);
		pump_mark_free(i);
	}
}

void pump_check_all(void)
{
	if (pump_is_busy(PUMP_LEFT1_NUM) &&
	    mechboard.pump_left1_current < I2C_MECHBOARD_CURRENT_COLUMN) {
		STMCH_DEBUG("Mark l1 as free");
		pump_mark_free(PUMP_LEFT1_NUM);
		pump_set(PUMP_LEFT1_NUM, PUMP_OFF);
	}

	if (pump_is_busy(PUMP_LEFT2_NUM) &&
	    mechboard.pump_left2_current < I2C_MECHBOARD_CURRENT_COLUMN) {
		STMCH_DEBUG("Mark l2 as free");
		pump_mark_free(PUMP_LEFT2_NUM);
		pump_set(PUMP_LEFT2_NUM, PUMP_OFF);
	}

	if (pump_is_busy(PUMP_RIGHT1_NUM) &&
	    sensor_get_adc(ADC_CSENSE3) < I2C_MECHBOARD_CURRENT_COLUMN) {
		STMCH_DEBUG("Mark r1 as free");
		pump_mark_free(PUMP_RIGHT1_NUM);
		pump_set(PUMP_RIGHT1_NUM, PUMP_OFF);
	}

	if (pump_is_busy(PUMP_RIGHT2_NUM) &&
	    sensor_get_adc(ADC_CSENSE4) < I2C_MECHBOARD_CURRENT_COLUMN) {
		STMCH_DEBUG("Mark r2 as free");
		pump_mark_free(PUMP_RIGHT2_NUM);
		pump_set(PUMP_RIGHT2_NUM, PUMP_OFF);
	}
}

uint8_t get_free_pump_count(void)
{
	uint8_t i, free_pump_count = 0;
	for (i=0; i<4; i++) {
		if (pump_is_free(i))
			free_pump_count++;
	}
	return free_pump_count;
}

/* move finger if we are not in lazy harvest */
void state_finger_goto(uint8_t mode, uint16_t position)
{
	if (mode == LAZY_HARVEST)
		return;
	finger_goto(position);
}

void state_manivelle(int16_t step_deg)
{
	double add_h = 0.;
	double add_d = 160.;
	double l = 70.;
	double step = RAD(step_deg);
	microseconds us;
	double al = RAD(0);
	double ar = RAD(180);

	time_wait_ms(500);

	us = time_get_us2();
	while (1) {
		al += step;
		ar += step;
		arm_do_xy(&left_arm, add_d+l*sin(al), add_h+l*cos(al), 10);
		arm_do_xy(&right_arm, add_d+l*sin(ar), add_h+l*cos(ar), 10);
		time_wait_ms(25);
		if (time_get_us2() - us > (4000L * 1000L))
			break;
	}
}

static void state_do_manivelle(void)
{
	if (!state_check(MANIVELLE))
		return;
	state_manivelle(30);
	while (state_check(MANIVELLE));
}

/* common function for pickup/harvest */
static void state_pickup_or_harvest(uint8_t mode)
{
	int8_t arm_num, pump_num;
	int8_t other_arm_num, other_pump_num;
	struct arm *arm;
        microseconds us;
	uint8_t flags, bad_color = 0, have_2cols = 0;

	pump_check_all();

	/* get arm num */
	if (pickup_side == I2C_LEFT_SIDE) {
		arm_num = ARM_LEFT_NUM;
		other_arm_num = ARM_RIGHT_NUM;
	}
	else {
		arm_num = ARM_RIGHT_NUM;
		other_arm_num = ARM_LEFT_NUM;
	}

	pump_num = arm_get_free_pump(arm_num);
	other_pump_num = arm_get_free_pump(other_arm_num);
	
	/* pump is not free... skip to other arm */
	if (mode == HARVEST && pump_num == -1) {
		STMCH_DEBUG("%s no free pump", __FUNCTION__);
		if (arm_num == ARM_RIGHT_NUM) {
			state_finger_goto(mode, FINGER_CENTER_RIGHT);
			pickup_side = I2C_LEFT_SIDE;
		}
		else {
			state_finger_goto(mode, FINGER_CENTER_LEFT);
			pickup_side = I2C_RIGHT_SIDE;
		}
		return;
	}
	else if (mode == PICKUP && pump_num == -1) {
		/* or exit when we are in pickup mode */
		IRQ_LOCK(flags);		
		if (mainboard_command.mode == mode)
			mainboard_command.mode = WAIT;
		IRQ_UNLOCK(flags);		
	}

	us = time_get_us2();
	/* wait front sensor */
	if (mode == HARVEST || mode == LAZY_HARVEST) {
		STMCH_DEBUG("%s wait front", __FUNCTION__);

		while (1) {
			if (sensor_get(S_FRONT))
				break;
			if (state_check(mode) == 0)
				return;
			/* wait 500ms before reading other
			   sensors */
			if (time_get_us2() - us < (500 * 1000L))
				continue;
			if (arm_get_sensor(arm_num))
				break;
			if (arm_get_sensor(other_arm_num)) {
				uint8_t tmp;
				tmp = arm_num;
				arm_num = other_arm_num;
				other_arm_num = tmp;
				pump_num = arm_get_free_pump(arm_num);
				other_pump_num = arm_get_free_pump(other_arm_num);
				if (other_pump_num == -1)
					return; // XXX
				break;
			}
		}
	}


	STMCH_DEBUG("%s arm_num=%d pump_num=%d",
		    __FUNCTION__, arm_num, pump_num);

	/* when ready, move finger */
	if (arm_num == ARM_RIGHT_NUM)
		state_finger_goto(mode, FINGER_RIGHT);
	else
		state_finger_goto(mode, FINGER_LEFT);

	state_debug_wait_key_pressed();

	
	arm = arm_num2ptr(arm_num);

	/* prepare arm, should be already done */
	arm_goto_prepare_get(arm_num, pump_num);
	while (arm_test_traj_end(arm, ARM_TRAJ_ALL) &&
	       state_check(mode));
	
	STMCH_DEBUG("%s arm pos ok", __FUNCTION__);
	
	state_debug_wait_key_pressed();
	
	/* wait to see the column on the sensor */
        us = time_get_us2();
	while (1) {
		if (arm_get_sensor(arm_num))
			break;
		if (state_check(mode) == 0)
			return;
		if (mode == PICKUP) /* no timeout in pickup */
			continue;
		/* 500ms timeout in harvest, go back */
		if (time_get_us2() - us > 500*1000L) {
			STMCH_DEBUG("%s timeout", __FUNCTION__);
			
			if (arm_num == ARM_RIGHT_NUM)
				state_finger_goto(mode, FINGER_LEFT);
			else
				state_finger_goto(mode, FINGER_RIGHT);
			
			if (sensor_get(S_FRONT))
				time_wait_ms(500);

			pump_set(pump_num, PUMP_OFF);
			return;
		}
	}

	state_dump_sensors();

	pump_set(pump_num, PUMP_ON);
	/* bad color */
	if (arm_get_color_sensor(arm_num) == -1) {
		bad_color = 1;
		STMCH_DEBUG("%s prepare eject", __FUNCTION__);
		mainboard_command.mode = PREPARE_EJECT;
		state_do_eject(arm_num, pump_num, mode);
		return;
	}

	STMCH_DEBUG("%s sensor ok", __FUNCTION__);

	/* by the way, prepare the other arm */
	if (other_pump_num != -1)
		arm_goto_prepare_get(other_arm_num, other_pump_num);

	/* get the column */
	arm_goto_get_column(arm_num, pump_num);

	us = time_get_us2();
	while (1) {
		/* wait 50 ms */
		if (time_get_us2() - us > 50*1000L)
			break;
		if (mode != HARVEST)
			continue;
		/* if we still see the front sensor, it's because
		 * there are 2 columns instead of one or because there
		 * is another column, so send the arm on other
		 * side. */
		if (sensor_get(S_FRONT) && have_2cols == 0) {
			STMCH_DEBUG("%s 2 columns, release finger", __FUNCTION__);
			have_2cols = 1;
			if (finger_get_side() == I2C_LEFT_SIDE)
				state_finger_goto(mode, FINGER_RIGHT);
			else
				state_finger_goto(mode, FINGER_LEFT);
		}
	}
	
	if (mode == HARVEST && have_2cols == 0) {
		/* just release a bit of effort */
		if (finger_get_side() == I2C_LEFT_SIDE) {
			state_finger_goto(mode, FINGER_LEFT_RELAX);
		}
		else {
			state_finger_goto(mode, FINGER_RIGHT_RELAX);
		}
	}
	else if (mode == PICKUP) {
		/* no free pump on other arm */
		if (other_pump_num == -1) {
			if (finger_get_side() == I2C_LEFT_SIDE) {
				state_finger_goto(mode, FINGER_LEFT_RELAX);
			}
			else {
				state_finger_goto(mode, FINGER_RIGHT_RELAX);
			}
		}
		/* else send finger on the other side */
		else {
			if (finger_get_side() == I2C_LEFT_SIDE) {
				state_finger_goto(mode, FINGER_RIGHT);
			}
			else {
				state_finger_goto(mode, FINGER_LEFT);
			}
		}
	}
	
	us = time_get_us2();
	while (1) {
		/* wait 100 ms */
		if (time_get_us2() - us > 100*1000L)
			break;
		if (mode != HARVEST)
			continue;
		/* if we still see the front sensor, it's because
		 * there are 2 columns instead of one or because there
		 * is another column, so send the arm on other
		 * side. */
		if (sensor_get(S_FRONT) && have_2cols == 0) {
			STMCH_DEBUG("%s 2 columns, release finger", __FUNCTION__);
			have_2cols = 1;
			if (finger_get_side() == I2C_LEFT_SIDE)
				state_finger_goto(mode, FINGER_RIGHT);
			else
				state_finger_goto(mode, FINGER_LEFT);
		}
	}
	
	/* consider the column as taken */
	pump_mark_busy(pump_num);

	state_debug_wait_key_pressed();
	
	arm_goto_intermediate_get(arm_num, pump_num);
	arm_wait_traj_end(arm, ARM_TRAJ_ALL_NEAR);

	/* prepare next */
	pump_num = arm_get_free_pump(arm_num);
	if (pump_num == -1)
		arm_goto_loaded(arm_num);
	else
		arm_goto_intermediate_get(arm_num, pump_num);

	state_debug_wait_key_pressed();

	/* switch to wait state */
	if (get_free_pump_count() == 0) {
		IRQ_LOCK(flags);		
		if (mainboard_command.mode == mode)
			mainboard_command.mode = WAIT;
		IRQ_UNLOCK(flags);		
	}

	/* next pickup/harvest will be on the other side */
	if (pickup_side == I2C_LEFT_SIDE)
		pickup_side = I2C_RIGHT_SIDE;
	else
		pickup_side = I2C_LEFT_SIDE;
}


/* manual mode, arm position is sent from mainboard */
static void state_do_manual(void)
{
	if (!state_check(MANUAL))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(MANUAL));
}

/* wait mode */
static void state_do_wait(void)
{
	if (!state_check(WAIT))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(WAIT));
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

	if (get_free_pump_count() == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	state_pickup_or_harvest(HARVEST);
}

/* harvest columns elts from area without moving finger */
static void state_do_lazy_harvest(void)
{
	if (!state_check(LAZY_HARVEST))
		return;

	if (get_free_pump_count() == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	state_pickup_or_harvest(LAZY_HARVEST);
}

/* eject a column. always called from pickup mode. */
static void state_do_eject(uint8_t arm_num, uint8_t pump_num, uint8_t old_mode)
{
	struct arm *arm;
 	arm = arm_num2ptr(arm_num);

	if (finger_get_side() == I2C_LEFT_SIDE) {
		state_finger_goto(old_mode, FINGER_LEFT_RELAX);
	}
	else {
		state_finger_goto(old_mode, FINGER_RIGHT_RELAX);
	}
	
	/* wait mainboard to eject */
	while (state_check(PREPARE_EJECT));

	if (finger_get_side() == I2C_LEFT_SIDE) {
		state_finger_goto(old_mode, FINGER_CENTER_LEFT);
	}
	else {
		state_finger_goto(old_mode, FINGER_CENTER_RIGHT);
	}

	arm_goto_get_column(arm_num, pump_num);
	arm_wait_traj_end(arm, ARM_TRAJ_ALL);
	time_wait_ms(150);

	state_debug_wait_key_pressed();
	
	arm_goto_prepare_eject(arm_num, pump_num);
	arm_wait_traj_end(arm, ARM_TRAJ_ALL);

	state_debug_wait_key_pressed();

	if (finger_get_side() == I2C_LEFT_SIDE) {
		state_finger_goto(old_mode, FINGER_LEFT_RELAX);
	}
	else {
		state_finger_goto(old_mode, FINGER_RIGHT_RELAX);
	}

	state_debug_wait_key_pressed();

	time_wait_ms(300);
	arm_goto_eject(arm_num, pump_num);
	time_wait_ms(200);
	pump_set(pump_num, PUMP_REVERSE);
	arm_wait_traj_end(arm, ARM_TRAJ_ALL);
	
	arm_goto_intermediate_get(arm_num, pump_num);
	pump_set(pump_num, PUMP_OFF);
}


/* prepare pickup in a dispenser, or harvest */
static void state_do_prepare_pickup(void)
{
	uint8_t left_count = 0, right_count = 0;
	int8_t pump_l, pump_r;

	if (!state_check(PREPARE_PICKUP))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	pump_check_all();

	pump_l = arm_get_free_pump(ARM_LEFT_NUM);
	if (pump_l == -1) {
		arm_goto_loaded(ARM_LEFT_NUM);
	}
	else {
		arm_goto_intermediate_front_get(ARM_LEFT_NUM, pump_l);
	}

	pump_r = arm_get_free_pump(ARM_RIGHT_NUM);
	if (pump_r == -1) {
		arm_goto_loaded(ARM_RIGHT_NUM);
	}
	else {
		arm_goto_intermediate_front_get(ARM_RIGHT_NUM, pump_r);
	}

	arm_wait_both(ARM_TRAJ_ALL);

	if (pump_l != -1)
		arm_goto_prepare_get(ARM_LEFT_NUM, pump_l);
	if (pump_r != -1)
		arm_goto_prepare_get(ARM_RIGHT_NUM, pump_r);

	if (mainboard_command.prep_pickup.side == I2C_AUTO_SIDE) {
		left_count += pump_is_busy(PUMP_LEFT1_NUM);
		left_count += pump_is_busy(PUMP_LEFT2_NUM);
		right_count += pump_is_busy(PUMP_RIGHT1_NUM);
		right_count += pump_is_busy(PUMP_RIGHT2_NUM);
		if (left_count < right_count)
			finger_goto(FINGER_RIGHT);
		else
			finger_goto(FINGER_LEFT);
	}
	else if (mainboard_command.prep_pickup.side == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else if (mainboard_command.prep_pickup.side == I2C_RIGHT_SIDE)
		finger_goto(FINGER_RIGHT);
	else if (mainboard_command.prep_pickup.side == I2C_CENTER_SIDE)
		finger_goto(FINGER_CENTER_LEFT);

	/* try to know on which side we have to pickup */
	if (finger_get_side() == I2C_RIGHT_SIDE) {
		pickup_side = I2C_LEFT_SIDE;
	}
	else {
		pickup_side = I2C_RIGHT_SIDE;
	}

	arm_prepare_free_pumps();

	mainboard_command.mode = mainboard_command.prep_pickup.next_mode;

	while (state_check(PREPARE_PICKUP));
}

/* clear pickup zone, will switch to harvest if needed */
static void state_do_clear(void)
{
	uint8_t flags, err;

	if (!state_check(CLEAR))
		return;

	if (get_free_pump_count() == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	finger_goto(FINGER_LEFT);
	err = WAIT_COND_OR_TIMEOUT(sensor_get(S_LEFT), 500);
	if (err) {
		IRQ_LOCK(flags);
		if (mainboard_command.mode == CLEAR)
			mainboard_command.mode = I2C_MECHBOARD_MODE_HARVEST;
		IRQ_UNLOCK(flags);
		pickup_side = I2C_LEFT_SIDE;
		return;
	}

	finger_goto(FINGER_RIGHT);
	err = WAIT_COND_OR_TIMEOUT(sensor_get(S_RIGHT), 500);
	if (err) {
		IRQ_LOCK(flags);
		if (mainboard_command.mode == CLEAR)
			mainboard_command.mode = I2C_MECHBOARD_MODE_HARVEST;
		IRQ_UNLOCK(flags);
		pickup_side = I2C_RIGHT_SIDE;
		return;
	}

	IRQ_LOCK(flags);
	if (mainboard_command.mode == CLEAR)
		mainboard_command.mode = I2C_MECHBOARD_MODE_HARVEST;
	IRQ_UNLOCK(flags);
}

/* do a lazy pickup */
static void state_do_lazy_pickup(void)
{
	int8_t flags, arm_num, pump_num;
	uint32_t us;

	if (!state_check(LAZY_PICKUP))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	if (arm_get_sensor(ARM_LEFT_NUM) && 
	    arm_get_sensor(ARM_RIGHT_NUM)) {
		IRQ_LOCK(flags);
		if (mainboard_command.mode == LAZY_PICKUP) {
			mainboard_command.mode = WAIT;
		}
		IRQ_UNLOCK(flags);
		return;
	}

	if (finger_get_side() == I2C_RIGHT_SIDE) {
		finger_goto(FINGER_LEFT);
		arm_num = ARM_LEFT_NUM;
	}
	else {
		finger_goto(FINGER_RIGHT);
		arm_num = ARM_RIGHT_NUM;
	}

	us = time_get_us2();
	while(1) {
		if (state_check(LAZY_PICKUP) == 0)
			return;
		if (arm_get_sensor(arm_num))
			break;
		if (time_get_us2() - us > 500*1000L) {
			if (finger_get_side() == I2C_RIGHT_SIDE)
				finger_goto(FINGER_LEFT);
			else
				finger_goto(FINGER_RIGHT);
			return;
		}
	}

	if (arm_get_color_sensor(arm_num) == -1) {
		pump_num = arm_get_free_pump(arm_num);
		if (pump_num == -1)
			return; /* XXX */
		pump_set(pump_num, PUMP_ON);
		STMCH_DEBUG("%s prepare eject", __FUNCTION__);
		mainboard_command.mode = PREPARE_EJECT;
		state_do_eject(arm_num, pump_num, LAZY_PICKUP);
	}
}

/* pickup from a dispenser automatically */
static void state_do_pickup(void)
{
	if (!state_check(PICKUP))
		return;

	if (get_free_pump_count() == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	/* XXX check that finger is at correct place */

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	state_pickup_or_harvest(PICKUP);
}

/* store columns without using arms */
static void state_do_store(void)
{
	int8_t arm_num;
	int8_t other_arm_num;
        microseconds us;

	if (!state_check(STORE))
		return;

	if (get_free_pump_count() == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	/* get arm num */
	if (pickup_side == I2C_LEFT_SIDE) {
		arm_num = ARM_LEFT_NUM;
		other_arm_num = ARM_RIGHT_NUM;
	}
	else {
		arm_num = ARM_RIGHT_NUM;
		other_arm_num = ARM_LEFT_NUM;
	}

	while (1) {
		if (sensor_get(S_FRONT))
			break;
		if (state_check(STORE) == 0)
			return;
	}

	/* when ready, move finger */
	if (arm_num == ARM_RIGHT_NUM)
		finger_goto(FINGER_RIGHT);
	else
		finger_goto(FINGER_LEFT);

	/* wait to see the column on the sensor */
        us = time_get_us2();
	while (1) {
		if (arm_get_sensor(arm_num))
			break;
		if (state_check(STORE) == 0)
			return;
		/* 500ms timeout in harvest, go back */
		if (time_get_us2() - us > 500*1000L) {
			STMCH_DEBUG("%s timeout", __FUNCTION__);
			
			if (arm_num == ARM_RIGHT_NUM)
				finger_goto(FINGER_LEFT);
			else
				finger_goto(FINGER_RIGHT);
			return;
		}
	}

	if (arm_get_sensor(arm_num) && arm_get_sensor(other_arm_num)) {
		STMCH_DEBUG("%s full", __FUNCTION__);
		while (state_check(STORE));
		return;
	}

	/* next store will be on the other side */
	if (pickup_side == I2C_LEFT_SIDE)
		pickup_side = I2C_RIGHT_SIDE;
	else
		pickup_side = I2C_LEFT_SIDE;
}

/* prepare the building of a temple */
static void state_do_prepare_build(void)
{
	int8_t pump_num, level;
	if (!state_check(PREPARE_BUILD))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	pump_check_all();

	if (finger_get_side() == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else
		finger_goto(FINGER_RIGHT);

	pump_num = arm_get_busy_pump(ARM_LEFT_NUM);
	level = mainboard_command.prep_build.level_l;
	if (pump_num != -1 && level != -1)
		arm_goto_prepare_autobuild_outside(ARM_LEFT_NUM, pump_num,
						   level, I2C_AUTOBUILD_DEFAULT_DIST);
	
	pump_num = arm_get_busy_pump(ARM_RIGHT_NUM);
	level = mainboard_command.prep_build.level_r;
	if (pump_num != -1 && level != -1)
		arm_goto_prepare_autobuild_outside(ARM_RIGHT_NUM, pump_num,
						   level, I2C_AUTOBUILD_DEFAULT_DIST);
	
	while (state_check(PREPARE_BUILD));
}

/* prepare the building of a temple */
static void state_do_push_temple(void)
{
	uint8_t level;

	level = mainboard_command.push_temple.level;

	if (!state_check(PUSH_TEMPLE))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	if (finger_get_side() == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else
		finger_goto(FINGER_RIGHT);

	arm_goto_prepare_push_temple(ARM_LEFT_NUM);
	arm_goto_prepare_push_temple(ARM_RIGHT_NUM);
	arm_wait_both(ARM_TRAJ_ALL);

	arm_goto_push_temple(ARM_LEFT_NUM, level);
	arm_goto_push_temple(ARM_RIGHT_NUM, level);
	
	while (state_check(PUSH_TEMPLE));
}

/* prepare the building of a temple */
static void state_do_push_temple_disc(void)
{
	uint8_t side;
	struct arm *arm;

	if (!state_check(PUSH_TEMPLE_DISC))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	side = mainboard_command.push_temple_disc.side;

	if (side == I2C_LEFT_SIDE) {
		arm = arm_num2ptr(ARM_LEFT_NUM);
		arm_goto_prepare_push_temple_disc(ARM_LEFT_NUM);
		arm_wait_traj_end(arm, ARM_TRAJ_ALL);
		arm_goto_push_temple_disc(ARM_LEFT_NUM);
	}
	else {
		arm = arm_num2ptr(ARM_RIGHT_NUM);
		arm_goto_prepare_push_temple_disc(ARM_RIGHT_NUM);
		arm_wait_traj_end(arm, ARM_TRAJ_ALL);
		arm_goto_push_temple_disc(ARM_RIGHT_NUM);
	}
	
	while (state_check(PUSH_TEMPLE_DISC));
}

/* prepare the building of a temple (mainly for columns) */
static void state_do_prepare_inside(void)
{
	int8_t pump_num, level_l, level_r;
	if (!state_check(PREPARE_INSIDE))
		return;

	level_l = mainboard_command.prep_inside.level_l;
	level_r = mainboard_command.prep_inside.level_r;
	STMCH_DEBUG("%s mode=%d level_l=%d, level_r=%d", __FUNCTION__,
		    state_get_mode(), level_l, level_r);

	pump_check_all();

	if (finger_get_side() == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else
		finger_goto(FINGER_RIGHT);

	pump_num = arm_get_busy_pump(ARM_LEFT_NUM);
	if (pump_num == -1)
		pump_num = PUMP_LEFT1_NUM;
	if (level_l != -1)
		arm_goto_prepare_build_inside(ARM_LEFT_NUM, pump_num,
						  level_l);
	
	pump_num = arm_get_busy_pump(ARM_RIGHT_NUM);
	if (pump_num == -1)
		pump_num = PUMP_RIGHT1_NUM;
	if (level_r != -1)
		arm_goto_prepare_build_inside(ARM_RIGHT_NUM, pump_num,
						  level_r);
	
	while (state_check(PREPARE_INSIDE));
}

/* moving position */
static void state_do_loaded(void)
{
	if (!state_check(LOADED))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	pump_check_all();

	if (finger_get_side() == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else
		finger_goto(FINGER_RIGHT);

	arm_goto_loaded(ARM_LEFT_NUM);
	arm_goto_loaded(ARM_RIGHT_NUM);
	
	while (state_check(LOADED));
}

static void state_do_build_lintel(uint8_t level)
{
	STMCH_DEBUG("%s() level=%d have_lintel=%d",
		    __FUNCTION__, level, mechboard.lintel_count);

	servo_lintel_out();

	arm_goto_prepare_get_lintel_inside1();
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	pump_set(PUMP_LEFT1_NUM, PUMP_REVERSE);
	pump_set(PUMP_RIGHT1_NUM, PUMP_REVERSE);
	arm_goto_prepare_get_lintel_inside2(mechboard.lintel_count);
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	arm_goto_get_lintel_inside(mechboard.lintel_count);
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	time_wait_ms(150);
	arm_goto_prepare_build_lintel1();
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	arm_goto_prepare_build_lintel2(level);
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	arm_goto_prepare_build_lintel3(level);
	arm_wait_both(ARM_TRAJ_ALL);
	state_debug_wait_key_pressed();

	if (mechboard.lintel_count == 1)
		servo_lintel_1lin();
	else
		servo_lintel_2lin();

	arm_goto_build_lintel(level);
	arm_wait_both(ARM_TRAJ_ALL);
	time_wait_ms(170);
	pump_set(PUMP_LEFT1_NUM, PUMP_ON);
	time_wait_ms(50); /* right arm a bit after */
	pump_set(PUMP_RIGHT1_NUM, PUMP_ON);
	time_wait_ms(130);
	pump_set(PUMP_LEFT1_NUM, PUMP_OFF);
	pump_set(PUMP_RIGHT1_NUM, PUMP_OFF);

	mechboard.lintel_count --;
}

/* Build one level of column. If pump_r or pump_l is -1, don't build
 * with this arm. */
static void state_do_build_column(uint8_t level_l, int8_t pump_l,
				  uint8_t dist_l,
				  uint8_t level_r, int8_t pump_r,
				  uint8_t dist_r)
{
	STMCH_DEBUG("%s() level_l=%d pump_l=%d level_r=%d pump_r=%d",
		    __FUNCTION__, level_l, pump_l, level_r, pump_r);

	/* nothing to do */
	if (pump_l == -1 && pump_r == -1)
		return;

	/* go above the selected level */
	if (pump_l != -1)
		arm_goto_prepare_autobuild_outside(ARM_LEFT_NUM, pump_l, level_l, dist_l);
	if (pump_r != -1)
		arm_goto_prepare_autobuild_outside(ARM_RIGHT_NUM, pump_r, level_r, dist_r);
	STMCH_DEBUG("l=%d r=%d", arm_test_traj_end(&left_arm, ARM_TRAJ_ALL), 
		    arm_test_traj_end(&right_arm, ARM_TRAJ_ALL));
	arm_wait_select(pump_l != -1, pump_r != -1, ARM_TRAJ_ALL);
	STMCH_DEBUG("l=%d r=%d", arm_test_traj_end(&left_arm, ARM_TRAJ_ALL), 
		    arm_test_traj_end(&right_arm, ARM_TRAJ_ALL));
	
	state_debug_wait_key_pressed();
		
	/* drop columns of P2 */
	if (pump_l != -1)
		arm_goto_autobuild(ARM_LEFT_NUM, pump_l, level_l, dist_l);
	if (pump_r != -1)
		arm_goto_autobuild(ARM_RIGHT_NUM, pump_r, level_r, dist_r);
	arm_wait_select(pump_l != -1, pump_r != -1, ARM_TRAJ_ALL);
		
	state_debug_wait_key_pressed();
		
	time_wait_ms(150);
	if (pump_l != -1)
		pump_set(pump_l, PUMP_REVERSE);
	if (pump_r != -1)
		pump_set(pump_r, PUMP_REVERSE);
	time_wait_ms(150);
	if (pump_l != -1) {
		pump_set(pump_l, PUMP_OFF);
		pump_mark_free(pump_l);
	}
	if (pump_r != -1) {
		pump_set(pump_r, PUMP_OFF);
		pump_mark_free(pump_r);
	}
	
	state_debug_wait_key_pressed();
}

/* autobuild columns elts from area */
/* check level to avoid bad things ? */
/* check if enough cols ? */
static void state_do_autobuild(void)
{
	int8_t pump_l, pump_r;
	/* copy command into local data */
	int8_t level_l = mainboard_command.autobuild.level_left;
	int8_t level_r = mainboard_command.autobuild.level_right;
	uint8_t count_l = mainboard_command.autobuild.count_left;
	uint8_t count_r = mainboard_command.autobuild.count_right;
	uint8_t dist_l = mainboard_command.autobuild.distance_left;
	uint8_t dist_r = mainboard_command.autobuild.distance_right;
	uint8_t do_lintel = mainboard_command.autobuild.do_lintel;
	int8_t max_level = level_l;
	

	if (!state_check(AUTOBUILD))
		return;

	STMCH_DEBUG("%s mode=%d do_lintel=%d", __FUNCTION__,
		    state_get_mode(), do_lintel);
	STMCH_DEBUG("  left: level=%d count=%d", level_l, count_l);
	STMCH_DEBUG("  right: level=%d count=%d", level_r, count_r);

	/* 
	 * build the first level of column if needed
	 */

	/* don't build with this arm if no pump or if we don't ask to */
	pump_l = arm_get_busy_pump(ARM_LEFT_NUM);
	if (count_l == 0)
		pump_l = -1;
	pump_r = arm_get_busy_pump(ARM_RIGHT_NUM);
	if (count_r == 0)
		pump_r = -1;

	if  (pump_l == -1 && pump_r == -1)
		goto lintel_only;

	state_do_build_column(level_l, pump_l, dist_l,
			      level_r, pump_r, dist_r);
	
	/* one level up */
	if (pump_l != -1) {
		count_l --;
		level_l ++;
		max_level = level_l;
	}
	if (pump_r != -1) {
		count_r --;
		level_r ++;
		if (level_r > max_level)
			max_level = level_r;
	}

	/* 
	 * build the second level of column if needed
	 */

	/* don't build with this arm if no pump or if we don't ask to */
	pump_l = arm_get_busy_pump(ARM_LEFT_NUM);
	if (count_l == 0)
		pump_l = -1;
	pump_r = arm_get_busy_pump(ARM_RIGHT_NUM);
	if (count_r == 0)
		pump_r = -1;

	state_do_build_column(level_l, pump_l, dist_l,
			      level_r, pump_r, dist_r);
	
	/* one level up */
	if (pump_l != -1) {
		count_l --;
		level_l ++;
		max_level = level_l;
	}
	if (pump_r != -1) {
		count_r --;
		level_r ++;
		if (level_r > max_level)
			max_level = level_r;
	}

	state_debug_wait_key_pressed();

	if (mechboard.lintel_count != 0 && do_lintel != 0) {
		arm_goto_prepare_autobuild_outside(ARM_LEFT_NUM,
						   PUMP_LEFT1_NUM,
						   max_level,
						   I2C_AUTOBUILD_DEFAULT_DIST);
		arm_goto_prepare_autobuild_outside(ARM_RIGHT_NUM,
						   PUMP_RIGHT1_NUM,
						   max_level,
						   I2C_AUTOBUILD_DEFAULT_DIST);
		arm_wait_both(ARM_TRAJ_ALL_NEAR);
		state_debug_wait_key_pressed();

		arm_goto_prepare_autobuild_inside(ARM_LEFT_NUM,
						  PUMP_LEFT1_NUM,
						  max_level);
		arm_goto_prepare_autobuild_inside(ARM_RIGHT_NUM,
						  PUMP_RIGHT1_NUM,
						  max_level);
		arm_wait_both(ARM_TRAJ_ALL_NEAR);
		state_debug_wait_key_pressed();
	}

 lintel_only:
	if (mechboard.lintel_count == 0 || do_lintel == 0) {
		mainboard_command.mode = WAIT;
		return;
	}

	state_do_build_lintel(max_level);
	mainboard_command.mode = WAIT;
}

/* prepare to get the lintel */
static void state_do_prepare_get_lintel(void)
{
	if (!state_check(PREPARE_GET_LINTEL))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	arm_goto_prepare_get_lintel_disp();
	arm_wait_both(ARM_TRAJ_ALL);

	pump_set(PUMP_LEFT1_NUM, PUMP_OFF);
	pump_set(PUMP_RIGHT1_NUM, PUMP_OFF);

	/* go fully left or right */
	if (finger_get_side() == I2C_LEFT_SIDE)
		finger_goto(FINGER_LEFT);
	else
		finger_goto(FINGER_RIGHT);

	while (state_check(PREPARE_GET_LINTEL));
}

/* get the lintel from the dispenser */
static void state_do_get_lintel(void)
{
	if (!state_check(GET_LINTEL))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	
	pump_set(PUMP_LEFT1_NUM, PUMP_REVERSE);
	pump_set(PUMP_RIGHT1_NUM, PUMP_REVERSE);

	arm_goto_get_lintel_disp();
	arm_wait_both(ARM_TRAJ_ALL_NEAR);
	
	time_wait_ms(200);

	STMCH_DEBUG("%s left1=%d left2=%d", __FUNCTION__,
		    mechboard.pump_left1_current,
		    sensor_get_adc(ADC_CSENSE3));

	while (state_check(GET_LINTEL));
	
	/* mainboard asked to release lintel, so release pump first */
	if (state_get_mode() == PREPARE_GET_LINTEL) {
		pump_set(PUMP_LEFT1_NUM, PUMP_ON);
		pump_set(PUMP_RIGHT1_NUM, PUMP_ON);
		time_wait_ms(200);
		pump_set(PUMP_LEFT1_NUM, PUMP_OFF);
		pump_set(PUMP_RIGHT1_NUM, PUMP_OFF);
	}
}

/* put the lintel inside the robot */
static void state_do_put_lintel(void)
{
	uint8_t prev_lin_count;

	if (!state_check(PUT_LINTEL))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	prev_lin_count = mechboard.lintel_count;
	mechboard.lintel_count ++;

	arm_goto_prepare_get_lintel_disp();
	arm_wait_both(ARM_TRAJ_ALL);

	servo_lintel_out();

	arm_goto_prepare_put_lintel();
	arm_wait_both(ARM_TRAJ_ALL_NEAR);
	
	arm_goto_put_lintel(prev_lin_count);
	arm_wait_both(ARM_TRAJ_ALL);
	
	pump_set(PUMP_LEFT1_NUM, PUMP_ON);
	pump_set(PUMP_RIGHT1_NUM, PUMP_ON);

	if (mechboard.lintel_count == 1)
		servo_lintel_1lin();
	else
		servo_lintel_2lin();

	time_wait_ms(300);
	
	pump_set(PUMP_LEFT1_NUM, PUMP_OFF);
	pump_set(PUMP_RIGHT1_NUM, PUMP_OFF);

	arm_goto_prepare_put_lintel();
	arm_wait_both(ARM_TRAJ_ALL_NEAR);

	while (state_check(PUT_LINTEL));
}

/* main state machine */
void state_machine(void)
{
	while (state_get_mode() != EXIT) {
		changed = 0;
		state_do_init();
		state_do_manual();
		state_do_harvest();
		state_do_lazy_harvest();
		state_do_prepare_pickup();
		state_do_pickup();
		state_do_prepare_inside();
		state_do_prepare_build();
		state_do_autobuild();
		state_do_prepare_get_lintel();
		state_do_get_lintel();
		state_do_put_lintel();
		state_do_loaded();
		state_do_clear();
		state_do_lazy_pickup();
		state_do_wait();
		state_do_store();
		state_do_manivelle();
		state_do_push_temple();
		state_do_push_temple_disc();
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	mainboard_command.mode = WAIT;
	pump_reset_all();
	mechboard.lintel_count = 1;
	mechboard.column_flags = 0;
	servo_lintel_1lin();
	finger_goto(FINGER_LEFT);
}
