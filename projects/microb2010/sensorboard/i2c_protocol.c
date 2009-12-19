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
 *  Revision : $Id: i2c_protocol.c,v 1.3 2009-05-27 20:04:07 zer0 Exp $
 *
 */

#include <string.h>

#include <aversive.h>
#include <aversive/list.h>
#include <aversive/error.h>

#include <scheduler.h>

#include <i2c.h>
#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <time.h>
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
#include "actuator.h"
#include "beacon.h"
#include "scanner.h"

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
	default:
		break;
	}
}

void i2c_send_status(void)
{
	struct i2c_ans_sensorboard_status ans;
	i2c_flush();
	ans.hdr.cmd =  I2C_ANS_SENSORBOARD_STATUS;
	ans.status = 0x55; /* XXX */
	ans.opponent_x = beacon.opponent_x;
	ans.opponent_y = beacon.opponent_y;
	ans.opponent_a = beacon.opponent_angle;
	ans.opponent_d = beacon.opponent_dist;

	ans.scan_status = 0;
	ans.scan_status |= scan_params.working ? 0 : I2C_SCAN_DONE; 
	ans.scan_status |= scan_params.max_column_detected ? I2C_SCAN_MAX_COLUMN : 0;


	ans.dropzone_x = scan_params.dropzone_x;
	ans.dropzone_y = scan_params.dropzone_y;
	ans.dropzone_h = scan_params.dropzone_h;

	i2c_send(I2C_ADD_MASTER, (uint8_t *) &ans,
		 sizeof(ans), I2C_CTRL_GENERIC);
}


void i2c_scanner_calibre_laser(void* dummy)
{
	scanner_calibre_laser();
}

void i2c_scanner_end_process(void* dummy)
{
	scanner_end_process();
}

void i2c_recvevent(uint8_t * buf, int8_t size)
{
	void *void_cmd = buf;
	static uint8_t a=0;
	a=!a;
	if (a)
		LED2_ON();
	else
		LED2_OFF();

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
		
	case I2C_CMD_GENERIC_SET_COLOR:
		{
			struct i2c_cmd_generic_color *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;
			sensorboard.our_color = cmd->color;
			break;
		}

	case I2C_CMD_SENSORBOARD_SET_BEACON:
		{
			struct i2c_cmd_sensorboard_start_beacon *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;

			if (cmd->enable)
				beacon_start();
			else
				beacon_stop();
				
			break;
		}

	case I2C_CMD_SENSORBOARD_SET_SCANNER:
		{
			struct i2c_cmd_sensorboard_scanner *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;

			scanner_set_mode(cmd->mode);
			break;

		}


	case I2C_CMD_SENSORBOARD_SCANNER_ALGO:
		{
			struct i2c_cmd_sensorboard_scanner_algo *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;

			scan_params.algo = cmd->algo;

			if (cmd->algo == I2C_SCANNER_ALGO_COLUMN_DROPZONE){
				scan_params.drop_zone.working_zone = cmd->drop_zone.working_zone;
				scan_params.drop_zone.center_x = cmd->drop_zone.center_x;
				scan_params.drop_zone.center_y = cmd->drop_zone.center_y;
			}
			else if (cmd->algo == I2C_SCANNER_ALGO_CHECK_TEMPLE) {
				scan_params.check_temple.level = cmd->check_temple.level;
				scan_params.check_temple.temple_x = cmd->check_temple.temple_x;
				scan_params.check_temple.temple_y = cmd->check_temple.temple_y;				
			} 
			else if (cmd->algo == I2C_SCANNER_ALGO_TEMPLE_DROPZONE){
				scan_params.drop_zone.working_zone = cmd->drop_zone.working_zone;
				scan_params.drop_zone.center_x = cmd->drop_zone.center_x;
				scan_params.drop_zone.center_y = cmd->drop_zone.center_y;
			}
			else{
				/* new command */
			}

			scan_params.working = 1;
			scheduler_add_single_event_priority(i2c_scanner_end_process, NULL, 
							    1,
							    CS_PRIO-1);
			break;

		}

	case I2C_CMD_SENSORBOARD_CALIB_SCANNER:
		{
			struct i2c_cmd_sensorboard_calib_scanner *cmd = void_cmd;
			if (size != sizeof (*cmd))
				goto error;
			
			scheduler_add_single_event_priority(i2c_scanner_calibre_laser, NULL, 
							    1,
							    CS_PRIO-1);
			break;
		}



	/* Add other commands here ...*/


	case I2C_REQ_SENSORBOARD_STATUS:
		{
			struct i2c_req_sensorboard_status *cmd = void_cmd;			
			if (size != sizeof (*cmd))
				goto error;

			beacon.robot_x = cmd->x;
			beacon.robot_y = cmd->y;
			beacon.robot_angle = cmd->a;

			if (cmd->enable_pickup_wheels)
				pickup_wheels_on();
			else
				pickup_wheels_off();
				
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

void i2c_recvbyteevent(uint8_t hwstatus, uint8_t i, uint8_t c)
{
}

void i2c_sendevent(int8_t size)
{
}


