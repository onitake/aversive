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
 *  Revision : $Id: i2c_protocol.c,v 1.6 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#include <string.h>

#include <aversive.h>
#include <aversive/list.h>
#include <aversive/error.h>

#include <i2c.h>
#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "state.h"

void i2c_protocol_init(void)
{
}

/*** LED CONTROL ***/
void i2c_led_control(uint8_t l, uint8_t state)
{
	switch(l) {
	case 1:
		state? LED1_ON():LED1_OFF();
		break;
	case 2:
		state? LED2_ON():LED2_OFF();
		break;
	case 3:
		state? LED3_ON():LED3_OFF();
		break;
	case 4:
		state? LED4_ON():LED4_OFF();
		break;
	default:
		break;
	}
}

#ifdef notyet
static void i2c_test(uint16_t val)
{
	static uint16_t prev=0;

	if ( (val-prev) != 1 ) {
		WARNING(E_USER_I2C_PROTO, "Dupplicated msg %d", val);
	}
	prev = val;
	ext.nb_test_cmd ++;
}
#endif

static void i2c_send_status(void)
{
	struct i2c_ans_cobboard_status ans;
	i2c_flush();
	ans.hdr.cmd =  I2C_ANS_COBBOARD_STATUS;

	/* status */
	ans.mode = state_get_mode();
	ans.status = state_get_status();

	ans.left_cobroller_speed = cobboard.left_cobroller_speed;
	ans.right_cobroller_speed = cobboard.right_cobroller_speed;

	ans.cob_count = state_get_cob_count();
;
	i2c_send(I2C_ADD_MASTER, (uint8_t *) &ans,
		 sizeof(ans), I2C_CTRL_GENERIC);
}

static int8_t i2c_set_mode(struct i2c_cmd_cobboard_set_mode *cmd)
{
	state_set_mode(cmd->mode);
	return 0;
}

void i2c_recvevent(uint8_t * buf, int8_t size)
{
	void *void_cmd = buf;

	static uint8_t a = 0;

	a++;
	if (a & 0x10)
		LED2_TOGGLE();

	if (size <= 0) {
		goto error;
	}

	switch (buf[0]) {

	/* Commands (no answer needed) */
	case I2C_CMD_GENERIC_LED_CONTROL:
		{
			struct i2c_cmd_led_control *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;
			i2c_led_control(cmd->led_num, cmd->state);
			break;
		}

	case I2C_CMD_COBBOARD_SET_MODE:
		{
			struct i2c_cmd_cobboard_set_mode *cmd = void_cmd;
			if (size != sizeof(struct i2c_cmd_cobboard_set_mode))
				goto error;
			i2c_set_mode(cmd);
			break;
		}

	case I2C_CMD_GENERIC_SET_COLOR:
		{
			struct i2c_cmd_generic_color *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;
			cobboard.our_color = cmd->color;
			break;
		}

#ifdef notyet
	case I2C_CMD_EXTENSION_TEST: 
		{
			struct i2c_cmd_extension_test *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;
			i2c_test(cmd->val);
			break;
		}
#endif
		
	/* Add other commands here ...*/


	case I2C_REQ_COBBOARD_STATUS:
		{
			//struct i2c_req_cobboard_status *cmd = void_cmd;
			if (size != sizeof (struct i2c_req_cobboard_status))
				goto error;
			
			i2c_send_status();
			break;
		}

	default:
		goto error;
	}

 error:
	/* log error on a led ? */
	return;
}

void i2c_recvbyteevent(__attribute__((unused)) uint8_t hwstatus,
		       __attribute__((unused)) uint8_t i, 
		       __attribute__((unused)) int8_t c)
{
}

void i2c_sendevent(__attribute__((unused)) int8_t size)
{
}


