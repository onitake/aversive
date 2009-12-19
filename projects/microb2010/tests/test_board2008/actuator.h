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
 *  Revision : $Id: actuator.h,v 1.2 2009-04-24 19:30:41 zer0 Exp $
 *
 */

void pwm_set_and_save(void *pwm, int32_t val);
void pickup_wheels_on(void);
void pickup_wheels_off(void);

void fessor_set(void *dummy, int32_t cmd);
void fessor_manage(void);
void fessor_up(void);
void fessor_down(void);
void fessor_stop(void);
void fessor_auto(void);
void fessor_init(void);
void fessor_autopos(void);
void fessor_dump_params(void);

void fessor_set_coefs(uint32_t k1, uint32_t k2);
void fessor_set_delays(uint32_t delay_up, uint32_t delay_down);
void fessor_set_pos(uint32_t pos_up, uint32_t pos_down);


void elevator_set(void *dummy, int32_t cmd);
void elevator_manage(void);
void elevator_up(void);
void elevator_down(void);
void elevator_stop(void);
void elevator_auto(void);
void elevator_init(void);
void elevator_autopos(void);
void elevator_dump_params(void);

void elevator_set_coefs(uint32_t k1, uint32_t k2);
void elevator_set_delays(uint32_t delay_up, uint32_t delay_down);
void elevator_set_pos(uint32_t pos_up, uint32_t pos_down);
