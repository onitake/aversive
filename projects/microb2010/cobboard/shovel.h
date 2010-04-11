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
 *  Revision : $Id: actuator.c,v 1.4 2009-04-24 19:30:41 zer0 Exp $
 *
 */

#ifndef _SHOVEL_H_
#define _SHOVEL_H_

#define SHOVEL_DOWN 100
#define SHOVEL_MID  4900
#define SHOVEL_UP   10000

void shovel_init(void);

static inline void shovel_down(void)
{
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_DOWN);
}

static inline void shovel_mid(void)
{
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_MID);
}

static inline void shovel_up(void)
{
	cs_set_consign(&cobboard.shovel.cs, SHOVEL_UP);
}

uint8_t shovel_is_up(void);
uint8_t shovel_is_down(void);

#endif /* _SHOVEL_H_ */
