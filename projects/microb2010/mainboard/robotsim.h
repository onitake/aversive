/*
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: main.c,v 1.9.4.5 2007-06-01 09:37:22 zer0 Exp $
 *
 */

extern uint8_t robotsim_blocking;

int8_t robotsim_i2c(uint8_t addr, uint8_t *buf, uint8_t size);
void robotsim_update(void);
void robotsim_pwm(void *arg, int32_t val);
int32_t robotsim_encoder_get(void *arg);
int robotsim_init(void);
void robotsim_dump(void);
int8_t robotsim_i2c_cobboard_set_mode(uint8_t mode);
int8_t robotsim_i2c_cobboard_set_spickles(uint8_t side, uint8_t flags);
