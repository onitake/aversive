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
#include <time.h>

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
static uint8_t error_log = 0;

/* used for commands */
uint8_t command_buf[I2C_SEND_BUFFER_SIZE];
volatile int8_t command_dest=-1;
volatile uint8_t command_size=0;

static int8_t i2c_req_mechboard_status(void);
static int8_t i2c_req_sensorboard_status(void);

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
	printf_P(PSTR("I2C protocol debug infos:\r\n"));
	printf_P(PSTR("  i2c_state=%d\r\n"), i2c_state);
	printf_P(PSTR("  i2c_errors=%d\r\n"), i2c_errors);
	printf_P(PSTR("  running_op=%d\r\n"), running_op);
	printf_P(PSTR("  command_size=%d\r\n"), command_size);
	printf_P(PSTR("  command_dest=%d\r\n"), command_dest);
	printf_P(PSTR("  i2c_status=%x\r\n"), i2c_status());
}

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

	/* poll status of mechboard */
#define I2C_REQ_MECHBOARD 0
	case I2C_REQ_MECHBOARD:
		if ((err = i2c_req_mechboard_status()))
			goto error;
		break;

#define I2C_ANS_MECHBOARD 1
	case I2C_ANS_MECHBOARD:
		if ((err = i2c_recv(I2C_MECHBOARD_ADDR, 
				    sizeof(struct i2c_ans_mechboard_status), 
				    I2C_CTRL_GENERIC)))
			goto error;
		break;

	/* poll status of sensorboard */
#define I2C_REQ_SENSORBOARD 2
	case I2C_REQ_SENSORBOARD:
		if ((err = i2c_req_sensorboard_status()))
			goto error;
		break;

#define I2C_ANS_SENSORBOARD 3
	case I2C_ANS_SENSORBOARD:
		if ((err = i2c_recv(I2C_SENSORBOARD_ADDR, 
				    sizeof(struct i2c_ans_sensorboard_status), 
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

	case I2C_ANS_MECHBOARD_STATUS: {
		struct i2c_ans_mechboard_status * ans = 
			(struct i2c_ans_mechboard_status *)buf;
		
		if (size != sizeof (*ans))
			goto error;

		/* status */
		mechboard.mode = ans->mode;
		mechboard.status = ans->status;
		mechboard.lintel_count = ans->lintel_count;
		mechboard.column_flags = ans->column_flags;
		/* pumps pwm */
		mechboard.pump_left1 = ans->pump_left1;
		mechboard.pump_left2 = ans->pump_left2;
		mechboard.pump_right1 = ans->pump_right1;
		mechboard.pump_right2 = ans->pump_right2;
		pwm_ng_set(LEFT_PUMP1_PWM, mechboard.pump_left1);
		pwm_ng_set(LEFT_PUMP2_PWM, mechboard.pump_left2);
		/* pumps current */
		mechboard.pump_right1_current = ans->pump_right1_current;
		mechboard.pump_right2_current = ans->pump_right2_current;
		/* servos */
		mechboard.servo_lintel_left = ans->servo_lintel_left;
		mechboard.servo_lintel_right = ans->servo_lintel_right;
		pwm_ng_set(&gen.servo2, mechboard.servo_lintel_right);
		pwm_ng_set(&gen.servo3, mechboard.servo_lintel_left);

		break;
	}
		
	case I2C_ANS_SENSORBOARD_STATUS: {
		struct i2c_ans_sensorboard_status * ans = 
			(struct i2c_ans_sensorboard_status *)buf;
		
		if (size != sizeof (*ans))
			goto error;
		sensorboard.status = ans->status;
		sensorboard.opponent_x = ans->opponent_x;
		sensorboard.opponent_y = ans->opponent_y;
		sensorboard.opponent_a = ans->opponent_a;
		sensorboard.opponent_d = ans->opponent_d;

		sensorboard.scan_status = ans->scan_status;
		sensorboard.dropzone_h = ans->dropzone_h;
		sensorboard.dropzone_x = ans->dropzone_x;
		sensorboard.dropzone_y = ans->dropzone_y;
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



/* ******** ******** ******** ******** */
/* commands */
/* ******** ******** ******** ******** */


static int8_t
i2c_send_command(uint8_t addr, uint8_t * buf, uint8_t size) 
{
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
}

static int8_t i2c_req_mechboard_status(void)
{
	struct i2c_req_mechboard_status buf;
	int8_t err;

	buf.hdr.cmd = I2C_REQ_MECHBOARD_STATUS;
	buf.pump_left1_current = sensor_get_adc(ADC_CSENSE3);
	buf.pump_left2_current = sensor_get_adc(ADC_CSENSE4);
	err = i2c_send(I2C_MECHBOARD_ADDR, (uint8_t*)&buf,
			sizeof(buf), I2C_CTRL_GENERIC);

	return err;
}

static int8_t i2c_req_sensorboard_status(void)
{
	struct i2c_req_sensorboard_status buf;
	
	buf.hdr.cmd = I2C_REQ_SENSORBOARD_STATUS;
	/* robot position */
	buf.x = position_get_x_s16(&mainboard.pos);
	buf.y = position_get_y_s16(&mainboard.pos);
	buf.a = position_get_a_deg_s16(&mainboard.pos);
	/* pickup wheels */
	buf.enable_pickup_wheels = mainboard.enable_pickup_wheels;
	
	return i2c_send(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf,
			sizeof(buf), I2C_CTRL_GENERIC);
}

int8_t i2c_set_color(uint8_t addr, uint8_t color)
{
	struct i2c_cmd_generic_color buf;

	if (addr == I2C_SENSORBOARD_ADDR)
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

int8_t i2c_mechboard_mode_manual(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_MANUAL;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_harvest(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_HARVEST;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_lazy_harvest(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_LAZY_HARVEST;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_pickup(uint8_t side)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
	buf.prep_pickup.next_mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
	buf.prep_pickup.side = side;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_push_temple_disc(uint8_t side)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PUSH_TEMPLE_DISC;
	buf.prep_pickup.side = side;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_pickup_next(uint8_t side, uint8_t next_mode)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
	buf.prep_pickup.next_mode = next_mode;
	buf.prep_pickup.side = side;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_pickup(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PICKUP;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_eject(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_EJECT;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_manivelle(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_MANIVELLE;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_push_temple(uint8_t level)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PUSH_TEMPLE;
	buf.push_temple.level = level;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf,sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_build_both(uint8_t level)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_BUILD;
	buf.prep_build.level_l = level;
	buf.prep_build.level_r = level;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_build_select(int8_t level_l, int8_t level_r)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_BUILD;
	buf.prep_build.level_l = level_l;
	buf.prep_build.level_r = level_r;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_inside_both(uint8_t level)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_INSIDE;
	buf.prep_inside.level_l = level;
	buf.prep_inside.level_r = level;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_inside_select(int8_t level_l, int8_t level_r)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_INSIDE;
	buf.prep_inside.level_l = level_l;
	buf.prep_inside.level_r = level_r;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_simple_autobuild(uint8_t level)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
	buf.autobuild.level_left = level;
	buf.autobuild.level_right = level;
	buf.autobuild.count_left = 2;
	buf.autobuild.count_right = 2;
	buf.autobuild.do_lintel = 1;
	buf.autobuild.distance_left = 210;
	buf.autobuild.distance_right = 210;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_autobuild(uint8_t level_l, uint8_t count_l,
				    uint8_t dist_l,
				    uint8_t level_r, uint8_t count_r,
				    uint8_t dist_r,
				    uint8_t do_lintel)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
	buf.autobuild.level_left = level_l;
	buf.autobuild.level_right = level_r;
	buf.autobuild.count_left = count_l;
	buf.autobuild.count_right = count_r;
	buf.autobuild.distance_left = dist_l;
	buf.autobuild.distance_right = dist_r;
	buf.autobuild.do_lintel = do_lintel;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_init(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_INIT;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_prepare_get_lintel(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PREPARE_GET_LINTEL;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_get_lintel(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_GET_LINTEL;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_put_lintel(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_PUT_LINTEL;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_clear(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_CLEAR;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_loaded(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_LOADED;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_store(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_STORE;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_mechboard_mode_lazy_pickup(void)
{
	struct i2c_cmd_mechboard_set_mode buf;
	buf.hdr.cmd = I2C_CMD_MECHBOARD_SET_MODE;
	buf.mode = I2C_MECHBOARD_MODE_LAZY_PICKUP;
	return i2c_send_command(I2C_MECHBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_set_beacon(uint8_t enable)
{
	struct i2c_cmd_sensorboard_start_beacon buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_SET_BEACON;
	buf.enable = enable;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_scanner_set(uint8_t mode)
{
	struct i2c_cmd_sensorboard_scanner buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_SET_SCANNER;
	buf.mode = mode;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_scanner_calib(void)
{
	struct i2c_cmd_sensorboard_calib_scanner buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_CALIB_SCANNER;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_scanner_algo_column(uint8_t zone,
					   int16_t x, int16_t y)
{
	struct i2c_cmd_sensorboard_scanner_algo buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_SCANNER_ALGO;
	buf.algo = I2C_SCANNER_ALGO_COLUMN_DROPZONE;
	buf.drop_zone.working_zone = zone;
	buf.drop_zone.center_x = x;
	buf.drop_zone.center_y = y;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_scanner_algo_temple(uint8_t zone,
					   int16_t x, int16_t y)
{
	struct i2c_cmd_sensorboard_scanner_algo buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_SCANNER_ALGO;
	buf.algo = I2C_SCANNER_ALGO_TEMPLE_DROPZONE;
	buf.drop_zone.working_zone = zone;
	buf.drop_zone.center_x = x;
	buf.drop_zone.center_y = y;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_sensorboard_scanner_algo_check(uint8_t level,
					  int16_t x, int16_t y)
{
	struct i2c_cmd_sensorboard_scanner_algo buf;
	buf.hdr.cmd = I2C_CMD_SENSORBOARD_SCANNER_ALGO;
	buf.algo = I2C_SCANNER_ALGO_CHECK_TEMPLE;
	buf.check_temple.level = level;
	buf.check_temple.temple_x = x;
	buf.check_temple.temple_y = y;
	return i2c_send_command(I2C_SENSORBOARD_ADDR, (uint8_t*)&buf, sizeof(buf));
}
