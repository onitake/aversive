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
 *  Revision : $Id: arm_highlevel.c,v 1.4 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>
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
#include "main.h"
#include "arm_xy.h"
#include "arm_highlevel.h"

#define WRIST_ANGLE_PUMP1 -2  /* in degree */
#define WRIST_ANGLE_PUMP2 103 /* in degree */

struct arm *arm_num2ptr(uint8_t arm_num)
{
	switch (arm_num) {
	case ARM_LEFT_NUM:
		return &left_arm;
	case ARM_RIGHT_NUM:
		return &right_arm;
	default:
		return NULL;
	}
}

#define ARM_MAX_H 110
#define ARM_STRAIGHT_D 254
#define ARM_STRAIGHT_H 0

void arm_goto_straight(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_STRAIGHT_D, ARM_STRAIGHT_H, angle);
}

/* position to get a column */
#define ARM_GET_D 60
#define ARM_GET_H -140

void arm_goto_get_column(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_GET_D, ARM_GET_H, angle);
}

/* position to get a column */
#define ARM_PREPARE_D 62
#define ARM_PREPARE_H -133

void arm_goto_prepare_get(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_PREPARE_D, ARM_PREPARE_H, angle);
}

#define ARM_INTERMEDIATE_D (65)
#define ARM_INTERMEDIATE_H (-115)

void arm_goto_intermediate_get(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_INTERMEDIATE_D, ARM_INTERMEDIATE_H, angle);
}

/* used in prepare pickup */
#define ARM_INTERMEDIATE_FRONT_D (90)
#define ARM_INTERMEDIATE_FRONT_H (-105)

void arm_goto_intermediate_front_get(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_INTERMEDIATE_FRONT_D,
		  ARM_INTERMEDIATE_FRONT_H, angle);
}

/* ****** */

#define ARM_PREPARE_EJECT_D (70)
#define ARM_PREPARE_EJECT_H (-50)

void arm_goto_prepare_eject(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_PREPARE_EJECT_D, ARM_PREPARE_EJECT_H, angle);
}

#define ARM_EJECT_D (200)
#define ARM_EJECT_H (30)

void arm_goto_eject(uint8_t arm_num, uint8_t pump_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	arm_do_xy(arm, ARM_EJECT_D, ARM_EJECT_H, angle);
}

/* ****** */

#define ARM_PREPARE_GET_LINTEL_INSIDE1_D 90
#define ARM_PREPARE_GET_LINTEL_INSIDE1_H -75
#define ARM_PREPARE_GET_LINTEL_INSIDE1_A -30
void arm_goto_prepare_get_lintel_inside1(void)
{
	arm_do_xy(&left_arm, ARM_PREPARE_GET_LINTEL_INSIDE1_D,
		  ARM_PREPARE_GET_LINTEL_INSIDE1_H,
		  ARM_PREPARE_GET_LINTEL_INSIDE1_A);
	arm_do_xy(&right_arm, ARM_PREPARE_GET_LINTEL_INSIDE1_D,
		  ARM_PREPARE_GET_LINTEL_INSIDE1_H,
		  ARM_PREPARE_GET_LINTEL_INSIDE1_A);
}

#define ARM_PREPARE_GET_LINTEL_INSIDE2_D 30
#define ARM_PREPARE_GET_LINTEL_INSIDE2_H -75
#define ARM_PREPARE_GET_LINTEL_INSIDE2_A -30
void arm_goto_prepare_get_lintel_inside2(uint8_t lintel_count)
{
	uint16_t d;
	d = ARM_PREPARE_GET_LINTEL_INSIDE2_D;
	if (lintel_count == 2)
		d += 34;
	arm_do_xy(&left_arm, d,
		  ARM_PREPARE_GET_LINTEL_INSIDE2_H,
		  ARM_PREPARE_GET_LINTEL_INSIDE2_A);
	arm_do_xy(&right_arm, d,
		  ARM_PREPARE_GET_LINTEL_INSIDE2_H,
		  ARM_PREPARE_GET_LINTEL_INSIDE2_A);
}

#define ARM_GET_LINTEL_INSIDE_D 10
#define ARM_GET_LINTEL_INSIDE_H -75
#define ARM_GET_LINTEL_INSIDE_A -30
void arm_goto_get_lintel_inside(uint8_t lintel_count)
{
	uint16_t d;
	d = ARM_GET_LINTEL_INSIDE_D;
	if (lintel_count == 2)
		d += 34;
	arm_do_xy(&left_arm, d,
		  ARM_GET_LINTEL_INSIDE_H,
		  ARM_GET_LINTEL_INSIDE_A);
	arm_do_xy(&right_arm, d,
		  ARM_GET_LINTEL_INSIDE_H,
		  ARM_GET_LINTEL_INSIDE_A);
}

#define ARM_PREPARE_BUILD_LINTEL1_D 30
#define ARM_PREPARE_BUILD_LINTEL1_H -50
#define ARM_PREPARE_BUILD_LINTEL1_A -30
void arm_goto_prepare_build_lintel1(void)
{
	arm_do_xy(&left_arm, ARM_PREPARE_BUILD_LINTEL1_D,
		  ARM_PREPARE_BUILD_LINTEL1_H,
		  ARM_PREPARE_BUILD_LINTEL1_A);
	arm_do_xy(&right_arm, ARM_PREPARE_BUILD_LINTEL1_D,
		  ARM_PREPARE_BUILD_LINTEL1_H,
		  ARM_PREPARE_BUILD_LINTEL1_A);
}

#define ARM_PREPARE_BUILD_LINTEL2_D 80
#define ARM_PREPARE_BUILD_LINTEL2_H -110
#define ARM_PREPARE_BUILD_LINTEL2_A 60
void arm_goto_prepare_build_lintel2(uint8_t level)
{
	int16_t h;
	if (level < 3)
		level = 3;
	h = (int16_t)level * 30 + ARM_PREPARE_BUILD_LINTEL2_H;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(&left_arm, ARM_PREPARE_BUILD_LINTEL2_D,
		  h, ARM_PREPARE_BUILD_LINTEL2_A);
	arm_do_xy(&right_arm, ARM_PREPARE_BUILD_LINTEL2_D,
		  h, ARM_PREPARE_BUILD_LINTEL2_A);
}

#define ARM_PREPARE_BUILD_LINTEL3_D 205
#define ARM_PREPARE_BUILD_LINTEL3_H -100
#define ARM_PREPARE_BUILD_LINTEL3_A 50
void arm_goto_prepare_build_lintel3(uint8_t level)
{
	int16_t h;
	if (level < 2)
		level = 2;
	h = (int16_t)level * 30 + ARM_PREPARE_BUILD_LINTEL3_H;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(&left_arm, ARM_PREPARE_BUILD_LINTEL3_D,
		  h, ARM_PREPARE_BUILD_LINTEL3_A);
	arm_do_xy(&right_arm, ARM_PREPARE_BUILD_LINTEL3_D,
		  h, ARM_PREPARE_BUILD_LINTEL3_A);
}

#define ARM_BUILD_LINTEL_D 205
#define ARM_BUILD_LINTEL_H -128
#define ARM_BUILD_LINTEL_A 50
void arm_goto_build_lintel(uint8_t level)
{
	int16_t h;
	h = (int16_t)level * 30 + ARM_BUILD_LINTEL_H;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(&left_arm, ARM_BUILD_LINTEL_D,
		  h, ARM_BUILD_LINTEL_A);
	arm_do_xy(&right_arm, ARM_BUILD_LINTEL_D,
		  h, ARM_BUILD_LINTEL_A);
}

/* ****** */

#define ARM_PREPARE_GET_LINTEL_DISP_D 190
#define ARM_PREPARE_GET_LINTEL_DISP_H -40
#define ARM_PREPARE_GET_LINTEL_DISP_A 50
void arm_goto_prepare_get_lintel_disp(void)
{
	arm_do_xy(&left_arm, ARM_PREPARE_GET_LINTEL_DISP_D,
		  ARM_PREPARE_GET_LINTEL_DISP_H,
		  ARM_PREPARE_GET_LINTEL_DISP_A);
	arm_do_xy(&right_arm, ARM_PREPARE_GET_LINTEL_DISP_D,
		  ARM_PREPARE_GET_LINTEL_DISP_H,
		  ARM_PREPARE_GET_LINTEL_DISP_A);
}

#define ARM_GET_LINTEL_DISP_D 190
#define ARM_GET_LINTEL_DISP_H -70
#define ARM_GET_LINTEL_DISP_A 40
void arm_goto_get_lintel_disp(void)
{
	arm_do_xy(&left_arm, ARM_GET_LINTEL_DISP_D,
		  ARM_GET_LINTEL_DISP_H,
		  ARM_GET_LINTEL_DISP_A);
	arm_do_xy(&right_arm, ARM_GET_LINTEL_DISP_D,
		  ARM_GET_LINTEL_DISP_H,
		  ARM_GET_LINTEL_DISP_A);
}

#define ARM_PREPARE_PUT_LINTEL_DISP_D 130
#define ARM_PREPARE_PUT_LINTEL_DISP_H 0
#define ARM_PREPARE_PUT_LINTEL_DISP_A 0
void arm_goto_prepare_put_lintel(void)
{
	arm_do_xy(&left_arm, ARM_PREPARE_PUT_LINTEL_DISP_D,
		  ARM_PREPARE_PUT_LINTEL_DISP_H,
		  ARM_PREPARE_PUT_LINTEL_DISP_A);
	arm_do_xy(&right_arm, ARM_PREPARE_PUT_LINTEL_DISP_D,
		  ARM_PREPARE_PUT_LINTEL_DISP_H,
		  ARM_PREPARE_PUT_LINTEL_DISP_A);
}

#define ARM_PUT_LINTEL_DISP_D 30
#define ARM_PUT_LINTEL_DISP_H -60
#define ARM_PUT_LINTEL_DISP_A -30
void arm_goto_put_lintel(uint8_t lintel_count)
{
	arm_do_xy(&left_arm,
		  ARM_PUT_LINTEL_DISP_D + lintel_count * 30,
		  ARM_PUT_LINTEL_DISP_H,
		  ARM_PUT_LINTEL_DISP_A);
	arm_do_xy(&right_arm,
		  ARM_PUT_LINTEL_DISP_D + lintel_count * 30,
		  ARM_PUT_LINTEL_DISP_H,
		  ARM_PUT_LINTEL_DISP_A);
}

/* ****** */


#define ARM_LOADED_D 100
#define ARM_LOADED_H 0

void arm_goto_loaded(uint8_t arm_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	arm_do_xy(arm, ARM_LOADED_D, ARM_LOADED_H, WRIST_ANGLE_PUMP2);
}


/* for columns */
#define ARM_PREPARE_BUILD_INSIDE_D  90
#define ARM_PREPARE_BUILD_INSIDE_H -45

void arm_goto_prepare_build_inside(uint8_t arm_num, uint8_t pump_num, uint8_t level)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	int16_t h;
	if (level < 2)
		level = 2;
	h = (int16_t)level * 30 + ARM_PREPARE_BUILD_INSIDE_H;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(arm, ARM_PREPARE_BUILD_INSIDE_D, h, angle);
}

#define ARM_PREPARE_AUTOBUILD_INSIDE_D  90
#define ARM_PREPARE_AUTOBUILD_INSIDE_H -70

void arm_goto_prepare_autobuild_inside(uint8_t arm_num, uint8_t pump_num, uint8_t level)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	int16_t h;
	if (level < 2)
		level = 2;
	h = (int16_t)level * 30 + ARM_PREPARE_AUTOBUILD_INSIDE_H;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(arm, ARM_PREPARE_AUTOBUILD_INSIDE_D, h, angle);
}

#define ARM_PREPARE_AUTOBUILD_OUTSIDE_D  210 /* not used, see dist below */
#define ARM_PREPARE_AUTOBUILD_OUTSIDE_H_P1 (-110)
#define ARM_PREPARE_AUTOBUILD_OUTSIDE_H_P2 (-90)

void arm_goto_prepare_autobuild_outside(uint8_t arm_num, uint8_t pump_num,
					uint8_t level, uint8_t dist)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	int16_t h;
	if (pump_num == PUMP_LEFT1_NUM || pump_num == PUMP_RIGHT1_NUM)
		h = (int16_t)level * 30 + ARM_PREPARE_AUTOBUILD_OUTSIDE_H_P1;
	else 
		h = (int16_t)level * 30 + ARM_PREPARE_AUTOBUILD_OUTSIDE_H_P2;
	if (h > ARM_MAX_H)
		h = ARM_MAX_H;
	arm_do_xy(arm, dist, h, angle);
}

#define ARM_AUTOBUILD_D_P1  208 /* not used, see dist below */
#define ARM_AUTOBUILD_D_P2  210 /* not used, see dist below */
#define ARM_AUTOBUILD_D_P1_OFFSET  (-2)
#define ARM_AUTOBUILD_D_P2_OFFSET  (0)
#define ARM_AUTOBUILD_H_P1 (-133)
#define ARM_AUTOBUILD_H_P2 (-130)

void arm_goto_autobuild(uint8_t arm_num, uint8_t pump_num, 
			uint8_t level, uint8_t dist)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t angle = pump_num2angle(pump_num);
	int16_t h;
	if (pump_num == PUMP_LEFT1_NUM || pump_num == PUMP_RIGHT1_NUM) {
		h = (int16_t)level * 30 + ARM_AUTOBUILD_H_P1;
		if (h > ARM_MAX_H)
			h = ARM_MAX_H;
		arm_do_xy(arm, dist + ARM_AUTOBUILD_D_P1_OFFSET, h, angle);
	}
	else {
		h = (int16_t)level * 30 + ARM_AUTOBUILD_H_P2;
		if (h > ARM_MAX_H)
			h = ARM_MAX_H;
		arm_do_xy(arm, dist + ARM_AUTOBUILD_D_P2_OFFSET, h, angle);
	}
}

#define ARM_PUSH_TEMPLE_D 170
#define ARM_PUSH_TEMPLE_H -165

void arm_goto_push_temple(uint8_t arm_num, uint8_t level)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int16_t h = ARM_PUSH_TEMPLE_H;

	/* level can be 0 or 1 */
	if (level)
		h += 30;
	arm_do_xy(arm, ARM_PUSH_TEMPLE_D, h, WRIST_ANGLE_PUMP1);
}

#define ARM_PREPARE_PUSH_TEMPLE_D 120
#define ARM_PREPARE_PUSH_TEMPLE_H -60

void arm_goto_prepare_push_temple(uint8_t arm_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	arm_do_xy(arm, ARM_PREPARE_PUSH_TEMPLE_D,
		  ARM_PREPARE_PUSH_TEMPLE_H, WRIST_ANGLE_PUMP1);
}

#define ARM_PUSH_TEMPLE_DISC_D1 215
#define ARM_PUSH_TEMPLE_DISC_H1 -100
#define ARM_PUSH_TEMPLE_DISC_D2 190
#define ARM_PUSH_TEMPLE_DISC_H2 -65

void arm_goto_push_temple_disc(uint8_t arm_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	int8_t pump_num;

	pump_num = arm_get_busy_pump(arm_num);
	if (pump_num == -1)
		arm_do_xy(arm, ARM_PUSH_TEMPLE_DISC_D1,
			  ARM_PUSH_TEMPLE_DISC_H1, WRIST_ANGLE_PUMP1);
	else
		arm_do_xy(arm, ARM_PUSH_TEMPLE_DISC_D2,
			  ARM_PUSH_TEMPLE_DISC_H2, WRIST_ANGLE_PUMP1);
}

#define ARM_PREPARE_PUSH_TEMPLE_DISC_D 100
#define ARM_PREPARE_PUSH_TEMPLE_DISC_H -80

void arm_goto_prepare_push_temple_disc(uint8_t arm_num)
{
	struct arm *arm = arm_num2ptr(arm_num);
	arm_do_xy(arm, ARM_PREPARE_PUSH_TEMPLE_DISC_D,
		  ARM_PREPARE_PUSH_TEMPLE_DISC_H, WRIST_ANGLE_PUMP1);
}

void arm_prepare_free_pumps(void)
{
	int8_t pump_num;

	pump_num = arm_get_free_pump(ARM_LEFT_NUM);
	if (pump_num != -1)
		arm_goto_prepare_get(ARM_LEFT_NUM, pump_num);
	pump_num = arm_get_free_pump(ARM_RIGHT_NUM);
	if (pump_num != -1)
		arm_goto_prepare_get(ARM_RIGHT_NUM, pump_num);
}


/* return the id of a free pump on this arm */
int8_t arm_get_free_pump(uint8_t arm_num)
{
	switch (arm_num) {
	case ARM_LEFT_NUM:
		if (pump_is_free(PUMP_LEFT1_NUM) && 
		    pump_is_free(PUMP_LEFT2_NUM))
			return PUMP_LEFT1_NUM;
		else if (pump_is_free(PUMP_LEFT2_NUM))
			return PUMP_LEFT2_NUM;
		return -1;
	case ARM_RIGHT_NUM:
		if (pump_is_free(PUMP_RIGHT1_NUM) && 
		    pump_is_free(PUMP_RIGHT2_NUM))
			return PUMP_RIGHT1_NUM;
		else if (pump_is_free(PUMP_RIGHT2_NUM))
			return PUMP_RIGHT2_NUM;
		return -1;
	default:
		return -1;
	}
}

/* return the id of a busy pump on this arm */
int8_t arm_get_busy_pump(uint8_t arm_num)
{
	switch (arm_num) {
	case ARM_LEFT_NUM:
		if (pump_is_busy(PUMP_LEFT2_NUM))
			return PUMP_LEFT2_NUM;
		else if (pump_is_busy(PUMP_LEFT1_NUM))
			return PUMP_LEFT1_NUM;
		return -1;
	case ARM_RIGHT_NUM:
		if (pump_is_busy(PUMP_RIGHT2_NUM))
			return PUMP_RIGHT2_NUM;
		else if (pump_is_busy(PUMP_RIGHT1_NUM))
			return PUMP_RIGHT1_NUM;
		return -1;
	default:
		return -1;
	}
}

uint8_t arm_wait_both(uint8_t mask)
{
	uint8_t ret;
	ret = arm_wait_traj_end(&left_arm, mask);
	if (ret != ARM_TRAJ_END && ret != ARM_TRAJ_NEAR)
		return ret;
	return arm_wait_traj_end(&right_arm, mask);
}

uint8_t arm_wait_select(uint8_t left, uint8_t right, uint8_t mask)
{
	if (left && right)
		return arm_wait_both(mask);
	if (left)
		return arm_wait_traj_end(&left_arm, mask);
	if (right)
		return arm_wait_traj_end(&right_arm, mask);
	return ARM_TRAJ_END;
}

/*********************/

int16_t *pump_num2ptr(uint8_t pump_num)
{
	switch (pump_num) {
	case PUMP_LEFT1_NUM:
		return &mechboard.pump_left1;
	case PUMP_RIGHT1_NUM:
		return &mechboard.pump_right1;
	case PUMP_LEFT2_NUM:
		return &mechboard.pump_left2;
	case PUMP_RIGHT2_NUM:
		return &mechboard.pump_right2;
	default:
		return NULL;
	}
}

void pump_set(uint8_t pump_num, int16_t val)
{
	int16_t *pump_ptr = pump_num2ptr(pump_num);

	*pump_ptr = val;

	switch (pump_num) {
	case PUMP_RIGHT1_NUM:
		pwm_ng_set(RIGHT_PUMP1_PWM, val);
		break;
	case PUMP_RIGHT2_NUM:
		pwm_ng_set(RIGHT_PUMP2_PWM, val);
		break;

	/* no pwm, it's remote */
	case PUMP_LEFT1_NUM:
	case PUMP_LEFT2_NUM:
	default:
		break;
	}
}

int16_t pump_num2angle(uint8_t pump_num)
{
	switch (pump_num) {
	case PUMP_LEFT1_NUM:
	case PUMP_RIGHT1_NUM:
		return WRIST_ANGLE_PUMP1;
	case PUMP_LEFT2_NUM:
	case PUMP_RIGHT2_NUM:
		return WRIST_ANGLE_PUMP2;
	default:
		return 0;
	}
}

void pump_mark_busy(uint8_t pump_num)
{
	switch (pump_num) {
	case PUMP_LEFT1_NUM:
		mechboard.column_flags |= I2C_MECHBOARD_COLUMN_L1;
		break;
	case PUMP_RIGHT1_NUM:
		mechboard.column_flags |= I2C_MECHBOARD_COLUMN_R1;
		break;
	case PUMP_LEFT2_NUM:
		mechboard.column_flags |= I2C_MECHBOARD_COLUMN_L2;
		break;
	case PUMP_RIGHT2_NUM:
		mechboard.column_flags |= I2C_MECHBOARD_COLUMN_R2;
		break;
	default:
		break;
	}

}

void pump_mark_free(uint8_t pump_num)
{
	switch (pump_num) {
	case PUMP_LEFT1_NUM:
		mechboard.column_flags &= (~I2C_MECHBOARD_COLUMN_L1);
		break;
	case PUMP_RIGHT1_NUM:
		mechboard.column_flags &= (~I2C_MECHBOARD_COLUMN_R1);
		break;
	case PUMP_LEFT2_NUM:
		mechboard.column_flags &= (~I2C_MECHBOARD_COLUMN_L2);
		break;
	case PUMP_RIGHT2_NUM:
		mechboard.column_flags &= (~I2C_MECHBOARD_COLUMN_R2);
		break;
	default:
		break;
	}

}

uint8_t pump_is_free(uint8_t pump_num)
{
	switch (pump_num) {
	case PUMP_LEFT1_NUM:
		return !(mechboard.column_flags & I2C_MECHBOARD_COLUMN_L1);
	case PUMP_RIGHT1_NUM:
		return !(mechboard.column_flags & I2C_MECHBOARD_COLUMN_R1);
	case PUMP_LEFT2_NUM:
		return !(mechboard.column_flags & I2C_MECHBOARD_COLUMN_L2);
	case PUMP_RIGHT2_NUM:
		return !(mechboard.column_flags & I2C_MECHBOARD_COLUMN_R2);
	default:
		return 0;
	}
}

uint8_t pump_is_busy(uint8_t pump_num)
{
	return !pump_is_free(pump_num);
}
