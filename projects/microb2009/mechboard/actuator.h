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

#define FINGER_RIGHT        666
#define FINGER_RIGHT_RELAX  621
#define FINGER_CENTER_RIGHT 491
#define FINGER_CENTER       490
#define FINGER_CENTER_LEFT  489
#define FINGER_LEFT         340
#define FINGER_LEFT_RELAX   385

void finger_goto(uint16_t position);
uint16_t finger_get_goal_pos(void);
uint8_t finger_get_side(void);

void servo_lintel_out(void);
void servo_lintel_1lin(void);
void servo_lintel_2lin(void);

void actuator_init(void);

