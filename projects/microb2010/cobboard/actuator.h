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
 *  Revision : $Id: actuator.h,v 1.6 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

void actuator_init(void);

void servo_carry_open(void);
void servo_carry_close(void);

void servo_door_open(void);
void servo_door_close(void);
void servo_door_block(void);

void cobroller_on(uint8_t side);
void cobroller_off(uint8_t side);
void cobroller_reverse(uint8_t side);

#endif

