/*  
 *  Copyright Droids Corporation, Microb Technology (2008)
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
 *  Revision : $Id: action.c,v 1.5 2008-03-31 22:22:52 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <aversive.h>

#include <uart.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "action.h"

void action_init(void)
{
}

/* 
 * Protocol:
 *   0000 XXXX select servo XXXX
 *   01XX XXXX set LSB of servo
 *   10XX XXXX set MSB of servo and validate value
 */
void action_servo_set(uint8_t num, uint16_t us)
{
	uart1_send(num);
	uart1_send(0x40 | (us&0x3F));
	uart1_send(0x80 | ((us>>6)&0x3F));
}
