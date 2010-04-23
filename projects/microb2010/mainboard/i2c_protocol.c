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
 *  Revision : $Id: i2c_protocol.c,v 1.8 2009-11-08 17:24:33 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <i2c.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "i2c_protocol.h"
#ifdef HOST_VERSION
#include "robotsim.h"
#endif

#define I2C_STATE_MAX 4

#define I2C_TIMEOUT 100 /* ms */
#define I2C_MAX_ERRORS 40

static volatile uint8_t i2c_poll_num = 0;
static volatile uint8_t i2c_state = 0;
static volatile uint16_t i2c_errors = 0;

#define OP_READY 0 /* no i2c op running */
#define OP_POLL  1 /* a user command is running */
#define OP_CMD   2 /* a polling (req / ans) is running */

static volatile uint8_t running_op = OP_READY;

#define I2C_MAX_LOG 3
#ifndef HOST_VERSION
static uint8_t error_log = 0;

static int8_t i2c_req_cobboard_status(void);
static int8_t i2c_req_ballboard_status(void);

#endif

/* used for commands */
uint8_t command_buf[I2C_SEND_BUFFER_SIZE];
volatile int8_t command_dest=-1;
volatile uint8_t command_size=0;

#define I2C_ERROR(args...) do {						\
		if (error_log < I2C_MAX_LOG) {				\
			ERROR(E_USER_I2C_PROTO, args);			\
			error_log ++;					\
			if (error_log == I2C_MAX_LOG) {			\
				ERROR(E_USER_I2C_PROTO,			\
				      "i2c logs are now warnings");	\
			}						\
		}							\
		else							\
			WARNING(E_USER_I2C_PROTO, args);		\
	} while(0)

void i2c_protocol_init(void)
{
}

void i2c_protocol_debug(void)
{
#ifdef HOST_VERSION
	return;
#else
	printf_P(PSTR("I2C protocol debug infos:\r\n"));
	printf_P(PSTR("  i2c_state=%d\r\n"), i2c_state);
	printf_P(PSTR("  i2c_errors=%d\r\n"), i2c_errors);
	printf_P(PSTR("  running_op=%d\r\n"), running_op);
	printf_P(PSTR("  command_size=%d\r\n"), command_size);
	printf_P(PSTR("  command_dest=%d\r\n"), command_dest);
	printf_P(PSTR("  i2c_status=%x\r\n"), i2c_status());
#endif
}

#ifndef HOST_VERSION
static void i2cproto_next_state(uint8_t inc)
{
	i2c_state += inc;
	if (i2c_state >= I2C_STATE_MAX) {
		i2c_state = 0;
		i2c_poll_num ++;
	}
}

void i2cproto_wait_update(void)
{
	uint8_t poll_num;
	poll_num = i2c_poll_num;
	WAIT_COND_OR_TIMEOUT((i2c_poll_num-poll_num) > 1, 150);
}

/* called periodically : the goal of this 'thread' is to send requests
 * and read answers on i2c slaves in the correct order. */
void i2c_poll_slaves(void *dummy)
{
	uint8_t flags;
	int8_t err;
	static uint8_t a = 0;

	a++;
	if (a & 0x4)
		LED2_TOGGLE();

	/* already running */
	IRQ_LOCK(flags);
	if (running_op != OP_READY) {
		IRQ_UNLOCK(flags);
		return;
	}

	/* if a command is ready to be sent, so send it */
	if (command_size) {
		running_op = OP_CMD;
		err = i2c_send(command_dest, command_buf, command_size,
			     I2C_CTRL_GENERIC);
		if (err < 0)
			goto error;
		IRQ_UNLOCK(flags);
		return;
	}

	/* no command, so do the polling */
	running_op = OP_POLL;

	switch(i2c_state) {

	/* poll status of cobboard */
#define I2C_REQ_COBBOARD 0
	case I2C_REQ_COBBOARD:
		if ((err = i2c_req_cobboard_status()))
			goto error;
		break;

#define I2C_ANS_COBBOARD 1
	case I2C_ANS_COBBOARD:
		if ((err = i2c_recv(I2C_COBBOARD_ADDR,
				    sizeof(struct i2c_ans_cobboard_status),
				    I2C_CTRL_GENERIC)))
			goto error;
		break;

	/* poll status of ballboard */
#define I2C_REQ_BALLBOARD 2
	case I2C_REQ_BALLBOARD:
		if ((err = i2c_req_ballboard_status()))
			goto error;
		break;

#define I2C_ANS_BALLBOARD 3
	case I2C_ANS_BALLBOARD:
		if ((err = i2c_recv(I2C_BALLBOARD_ADDR,
				    sizeof(struct i2c_ans_ballboard_status),
				    I2C_CTRL_GENERIC)))
			goto error;
		break;

	/* nothing, go to the first request */
	default:
		i2c_state = 0;
		running_op = OP_READY;
	}
	IRQ_UNLOCK(flags);

	return;

 error:
	running_op = OP_READY;
	IRQ_UNLOCK(flags);
	i2c_errors++;
	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C send is_cmd=%d proto_state=%d "
		      "err=%d i2c_status=%x", !!command_size, i2c_state, err, i2c_status());
		i2c_reset();
		i2c_errors = 0;
	}
}

/* called when the xmit is finished */
void i2c_sendevent(int8_t size)
{
	if (size > 0) {
		if (running_op == OP_POLL) {
			i2cproto_next_state(1);
		}
		else
			command_size = 0;
	}
	else {
		i2c_errors++;
		NOTICE(E_USER_I2C_PROTO, "send error state=%d size=%d "
			"op=%d", i2c_state, size, running_op);
		if (i2c_errors > I2C_MAX_ERRORS) {
			I2C_ERROR("I2C error, slave not ready");
			i2c_reset();
			i2c_errors = 0;
		}

		if (running_op == OP_POLL) {
			/* skip associated answer */
			i2cproto_next_state(2);
		}
	}
	running_op = OP_READY;
}

/* called rx event */
void i2c_recvevent(uint8_t * buf, int8_t size)
{
	if (running_op == OP_POLL)
		i2cproto_next_state(1);

	/* recv is only trigged after a poll */
	running_op = OP_READY;

	if (size < 0) {
		goto error;
	}

	switch (buf[0]) {

	case I2C_ANS_COBBOARD_STATUS: {
		struct i2c_ans_cobboard_status * ans =
			(struct i2c_ans_cobboard_status *)buf;

		if (size != sizeof (*ans))
			goto error;

		/* status */
		cobboard.mode = ans->mode;
		cobboard.status = ans->status;
		cobboard.left_cobroller_speed = ans->left_cobroller_speed;
		cs_set_consign(&mainboard.left_cobroller.cs, cobboard.left_cobroller_speed);
		cobboard.right_cobroller_speed = ans->right_cobroller_speed;
		cs_set_consign(&mainboard.right_cobroller.cs, cobboard.right_cobroller_speed);

		break;
	}

	case I2C_ANS_BALLBOARD_STATUS: {
		struct i2c_ans_ballboard_status * ans =
			(struct i2c_ans_ballboard_status *)buf;

		if (size != sizeof (*ans))
			goto error;
		ballboard.mode = ans->mode;
		ballboard.status = ans->status;
		ballboard.ball_count = ans->ball_count;
		break;
	}

	default:
		break;
	}

	return;
 error:
	i2c_errors++;
	NOTICE(E_USER_I2C_PROTO, "recv error state=%d op=%d",
	       i2c_state, running_op);
	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C error, slave not ready");
		i2c_reset();
		i2c_errors = 0;
	}
}

void i2c_recvbyteevent(uint8_t hwstatus, uint8_t i, uint8_t c)
{
}
#endif /* !HOST_VERSION */

/* ******** ******** ******** ******** */
/* commands */
/* ******** ******** ******** ******** */


static int8_t
i2c_send_command(uint8_t addr, uint8_t * buf, uint8_t size)
{
#ifdef HOST_VERSION
	return robotsim_i2c(addr, buf, size);
#else
	uint8_t flags;
        microseconds us = time_get_us2();

	while ((time_get_us2() - us) < (I2C_TIMEOUT)*1000L) {
		IRQ_LOCK(flags);
		if (command_size == 0) {
			memcpy(command_buf, buf, size);
			command_size = size;
			command_dest = addr;
			IRQ_UNLOCK(flags);
			return 0;
		}
		IRQ_UNLOCK(flags);
	}
	/* this should not happen... except if we are called from an
	 * interrupt context, but it's forbidden */
	I2C_ERROR("I2C command send failed");
	return -EBUSY;
#endif
}

#ifndef HOST_VERSION
static int8_t i2c_req_cobboard_status(void)
{
	struct i2c_req_cobboard_status buf;
	int8_t err;

	buf.hdr.cmd = I2C_REQ_COBBOARD_STATUS;
	buf.lspickle = cobboard.lspickle;
	buf.rspickle = cobboard.rspickle;
	err = i2c_send(I2C_COBBOARD_ADDR, (uint8_t*)&buf,
			sizeof(buf), I2C_CTRL_GENERIC);

	return err;
}

static int8_t i2c_req_ballboard_status(void)
{
	struct i2c_req_ballboard_status buf;

	buf.hdr.cmd = I2C_REQ_BALLBOARD_STATUS;
	return i2c_send(I2C_BALLBOARD_ADDR, (uint8_t*)&buf,
			sizeof(buf), I2C_CTRL_GENERIC);
}
#endif /* !HOST_VERSION */

int8_t i2c_set_color(uint8_t addr, uint8_t color)
{
	struct i2c_cmd_generic_color buf;

	if (addr == I2C_BALLBOARD_ADDR)
		return 0; /* XXX disabled for now */
	buf.hdr.cmd = I2C_CMD_GENERIC_SET_COLOR;
	buf.color = color;
	return i2c_send_command(addr, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state)
{
	struct i2c_cmd_led_control buf;
	buf.hdr.cmd = I2C_CMD_GENERIC_LED_CONTROL;
	buf.led_num = led;
	buf.state = state;
	return i2c_send_command(addr, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_cobboard_set_mode(uint8_t mode)
{
#ifdef HOST_VERSION
	return robotsim_i2c_cobboard_set_mode(mode);
#else
	struct i2c_cmd_cobboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_COBBOARD_SET_MODE;
	buf.mode = mode;
	return i2c_send_command(I2C_COBBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
#endif
}

static int8_t i2c_cobboard_set_spickle(uint8_t side, uint8_t flags)
{
	if (side == I2C_LEFT_SIDE)
		cobboard.lspickle = flags;
	else
		cobboard.rspickle = flags;
	return 0;
}

int8_t i2c_cobboard_pack(uint8_t side)
{
	return i2c_cobboard_set_spickle(side, 0);
}

int8_t i2c_cobboard_harvest(uint8_t side)
{
	return i2c_cobboard_set_spickle(side,
					I2C_COBBOARD_SPK_DEPLOY |
					I2C_COBBOARD_SPK_AUTOHARVEST);
}

int8_t i2c_cobboard_deploy(uint8_t side)
{
	return i2c_cobboard_set_spickle(side, I2C_COBBOARD_SPK_DEPLOY);
}

int8_t i2c_ballboard_set_mode(uint8_t mode)
{
	struct i2c_cmd_ballboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_BALLBOARD_SET_MODE;
	buf.mode = mode;
	return i2c_send_command(I2C_BALLBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

