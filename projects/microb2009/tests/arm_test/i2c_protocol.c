/*
 *  Copyright Droids Corporation (2007)
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
 *  Revision : $Id: i2c_protocol.c,v 1.1 2009-01-30 20:38:49 zer0 Exp $
 *
 */


#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <uart.h>
#include <i2c.h>
#include <time.h>

#include "../common/i2c_commands.h"
#include "strat_base.h"
#include "main.h"
#include "i2c_protocol.h"

//#define DEBUG_FROM_EXT

#define I2C_TIMEOUT 100 /* ms */
#define I2C_MAX_ERRORS 40

/* used for commands */
uint8_t command_buf[I2C_SEND_BUFFER_SIZE];

void i2c_protocol_init(void)
{
}

void i2c_poll_slaves(void * dummy)
{
}

/* called when the xmit is finished */
void i2c_sendevent(int8_t size)
{
	if (size > 0) {
	}
	else {
		i2c_errors++;
		NOTICE(E_USER_I2C_PROTO, "send error size=%d", size);
	}
}

/* called rx event */
void i2c_recvevent(uint8_t * buf, int8_t size)
{
	if (size < 0) {
		goto error;
	}

	for (i = 0; i<size; i++) {
		printf("%x ", buf[1]);
	}
	printf("\r\n");

	return;
 error:
	NOTICE(E_USER_I2C_PROTO, "recv error size=%d", size);
}
	
void i2c_recvbyteevent(uint8_t hwstatus, uint8_t i, uint8_t c)
{
}


