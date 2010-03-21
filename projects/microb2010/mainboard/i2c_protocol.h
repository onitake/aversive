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
 *  Revision : $Id: i2c_protocol.h,v 1.6 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#ifndef _I2C_PROTOCOL_H_
#define _I2C_PROTOCOL_H_

void i2c_protocol_init(void);
void i2c_protocol_debug(void);

void i2cproto_wait_update(void);
void i2c_poll_slaves(void *dummy);

void i2c_recvevent(uint8_t *buf, int8_t size);
void i2c_recvbyteevent(uint8_t hwstatus, uint8_t i, uint8_t c);
void i2c_sendevent(int8_t size);

int8_t i2c_set_color(uint8_t addr, uint8_t color);
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state);

int8_t i2c_cobboard_mode_manual(void);
int8_t i2c_cobboard_mode_harvest(void);
int8_t i2c_cobboard_mode_init(void);

#endif
