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

#ifndef _SPICKLE_H_
#define _SPICKLE_H_

void spickle_set(void *dummy, int32_t cmd);
void spickle_set_scoefs(uint32_t k1, uint32_t k2);
void spickle_set_wcoefs(uint32_t k1, uint32_t k2);
void spickle_set_pos(uint8_t side, int32_t pos_pack,
		     int32_t pos_mid, int32_t pos_deployed);

void spickle_dump_params(void);

uint8_t spickle_is_packed(void);
uint8_t spickle_is_deployed(void);

void spickle_deploy(uint8_t side);
void spickle_mid(uint8_t side);
void spickle_pack(uint8_t side);

void spickle_init(void);

#endif
