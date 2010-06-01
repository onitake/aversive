/*
 *  Copyright Droids Corporation (2010)
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
 *  Revision : $Id: actuator.h,v 1.2 2009-04-24 19:30:42 zer0 Exp $
 *
 */

#define ROLLER_SPEED 1200

void roller_on(void);
void roller_off(void);
void roller_reverse(void);

void fork_deploy(void);
void fork_pack(void);
void fork_mid1(void);
void fork_mid2(void);
void fork_eject(void);
uint8_t fork_is_packed(void);

void actuator_init(void);
