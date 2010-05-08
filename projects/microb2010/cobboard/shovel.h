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

void shovel_init(void);

void shovel_set_current_limit_coefs(int32_t k1, int32_t k2);
uint8_t shovel_get_current_limit_coefs(int32_t *k1, int32_t *k2);
void shovel_current_limit_enable(uint8_t enable);
void shovel_set(void *mot, int32_t cmd);

void shovel_down(void);
void shovel_mid(void);
void shovel_up(void);
void shovel_kickstand(void);

uint8_t shovel_is_up(void);
uint8_t shovel_is_down(void);

#endif /* _SHOVEL_H_ */
