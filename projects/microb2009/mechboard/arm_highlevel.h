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
 *  Revision : $Id: arm_highlevel.h,v 1.4 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#define ARM_LEFT_NUM 0
#define ARM_RIGHT_NUM 1

#define PUMP_LEFT1_NUM  0
#define PUMP_RIGHT1_NUM 1
#define PUMP_LEFT2_NUM  2
#define PUMP_RIGHT2_NUM 3

struct arm *arm_num2ptr(uint8_t arm_num);

void arm_goto_straight(uint8_t arm_num, uint8_t pump_num);
void arm_goto_get_column(uint8_t arm_num, uint8_t pump_num);
void arm_goto_prepare_get(uint8_t arm_num, uint8_t pump_num);
void arm_goto_intermediate_get(uint8_t arm_num, uint8_t pump_num);
void arm_goto_intermediate_front_get(uint8_t arm_num, uint8_t pump_num);
void arm_goto_prepare_eject(uint8_t arm_num, uint8_t pump_num);
void arm_goto_eject(uint8_t arm_num, uint8_t pump_num);
void arm_goto_loaded(uint8_t arm_num);
void arm_goto_prepare_build_inside(uint8_t arm_num, uint8_t pump_num,
				   uint8_t level);
void arm_goto_prepare_autobuild_inside(uint8_t arm_num, uint8_t pump_num,
				       uint8_t level);
void arm_goto_prepare_autobuild_outside(uint8_t arm_num, uint8_t pump_num,
					uint8_t level, uint8_t dist);
void arm_goto_autobuild(uint8_t arm_num, uint8_t pump_num,
					uint8_t level, uint8_t dist);

void arm_goto_prepare_get_lintel_inside1(void);
void arm_goto_prepare_get_lintel_inside2(uint8_t lintel_count);
void arm_goto_get_lintel_inside(uint8_t lintel_count);
void arm_goto_prepare_build_lintel1(void);
void arm_goto_prepare_build_lintel2(uint8_t level);
void arm_goto_prepare_build_lintel3(uint8_t level);
void arm_goto_build_lintel(uint8_t level);

void arm_goto_prepare_get_lintel_disp(void);
void arm_goto_get_lintel_disp(void);

void arm_goto_prepare_put_lintel(void);
void arm_goto_put_lintel(uint8_t lintel_count);
void arm_goto_prepare_push_temple(uint8_t arm_num);
void arm_goto_push_temple(uint8_t arm_num, uint8_t level);
void arm_goto_prepare_push_temple_disc(uint8_t arm_num);
void arm_goto_push_temple_disc(uint8_t arm_num);

void arm_prepare_free_pumps(void);
uint8_t arm_wait_both(uint8_t mask);
uint8_t arm_wait_select(uint8_t left, uint8_t right, uint8_t mask);

/* return the id of the free pump for the arm, or return -1 */
int8_t arm_get_free_pump(uint8_t arm_num);
int8_t arm_get_busy_pump(uint8_t arm_num);

#define PUMP_ON      3400
#define PUMP_OFF     0
#define PUMP_REVERSE -3400

void pump_set(uint8_t pump_num, int16_t val);
int16_t pump_num2angle(uint8_t pump_num);
void pump_mark_busy(uint8_t pump_num);
void pump_mark_free(uint8_t pump_num);
uint8_t pump_is_free(uint8_t pump_num);
uint8_t pump_is_busy(uint8_t pump_num);
