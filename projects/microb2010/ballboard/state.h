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
 *  Revision : $Id: state.h,v 1.5 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#ifndef _STATE_H_
#define _STATE_H_

/* set a new state, return 0 on success */
int8_t state_set_mode(uint8_t mode);

/* get current state */
uint8_t state_get_status(void);

uint8_t state_get_ball_count(void);

/* launch state machine */
void state_machine(void);

void state_init(void);

extern uint8_t state_debug;

#endif
