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
 *  Revision : $Id: i2c_commands.h,v 1.9 2009-05-27 20:04:06 zer0 Exp $
 *
 */

#ifndef _I2C_COMMANDS_H_
#define _I2C_COMMANDS_H_

#define I2C_OPPONENT_NOT_THERE -1000
#define I2C_BEACON_NOT_FOUND -1000

#define I2C_MAINBOARD_ADDR   1
#define I2C_COBBOARD_ADDR    2
#define I2C_BALLBOARD_ADDR   3

#define I2C_LEFT_SIDE    0
#define I2C_RIGHT_SIDE   1
#define I2C_AUTO_SIDE    2
#define I2C_CENTER_SIDE  3
#define I2C_NO_SIDE      4

#define I2C_COLOR_YELLOW 0
#define I2C_COLOR_BLUE   1

#define I2C_COB_BLACK   0
#define I2C_COB_WHITE   1
#define I2C_COB_UNKNOWN 2
#define I2C_COB_REMOVED 3
#define I2C_COB_NONE    4

struct i2c_cmd_hdr {
	uint8_t cmd;
};

/****/
/* commands that do not need and answer */
/****/

#define I2C_CMD_GENERIC_LED_CONTROL 0x00

struct i2c_cmd_led_control {
	struct i2c_cmd_hdr hdr;
	uint8_t led_num:7;
	uint8_t state:1;
};

/****/

#define I2C_CMD_GENERIC_SET_COLOR 0x01

struct i2c_cmd_generic_color {
	struct i2c_cmd_hdr hdr;
	uint8_t color;
};

/****/

#define I2C_CMD_COBBOARD_SET_MODE 0x02

struct i2c_cmd_cobboard_set_mode {
	struct i2c_cmd_hdr hdr;

#define I2C_COBBOARD_MODE_HARVEST      0x01 /* harvest mode */
#define I2C_COBBOARD_MODE_EJECT        0x02 /* eject cobs */
#define I2C_COBBOARD_MODE_INIT         0x03 /* init state machine */
#define I2C_COBBOARD_MODE_KICKSTAND_UP   0x04 /* help to climb the hill */
#define I2C_COBBOARD_MODE_KICKSTAND_DOWN 0x05 /* help to climb the hill */
	uint8_t mode;
};

#define I2C_CMD_BALLBOARD_SET_MODE 0x10

struct i2c_cmd_ballboard_set_mode {
	struct i2c_cmd_hdr hdr;

#define I2C_BALLBOARD_MODE_INIT        0x00
#define I2C_BALLBOARD_MODE_OFF         0x01
#define I2C_BALLBOARD_MODE_HARVEST     0x02
#define I2C_BALLBOARD_MODE_EJECT       0x03
#define I2C_BALLBOARD_MODE_PREP_FORK   0x04
#define I2C_BALLBOARD_MODE_TAKE_FORK   0x05
	uint8_t mode;
};

#define I2C_CMD_BALLBOARD_SET_BEACON 0x04

struct i2c_cmd_ballboard_start_beacon {
	struct i2c_cmd_hdr hdr;
	uint8_t enable;
};

/****/
/* requests and their answers */
/****/


#define I2C_REQ_COBBOARD_STATUS 0x80

struct i2c_req_cobboard_status {
	struct i2c_cmd_hdr hdr;

#define I2C_COBBOARD_SPK_DEPLOY  0x01 /* deploy the spickle */
#define I2C_COBBOARD_SPK_AUTOHARVEST 0x02 /* auto harvest the cobs */
#define I2C_COBBOARD_SPK_NO_MOVE 0x04 /* if enabled, don't change state */
#define I2C_COBBOARD_SPK_WEAK 0x08 /* if enabled, current limit */
	uint8_t lspickle;
	uint8_t rspickle;
};

#define I2C_ANS_COBBOARD_STATUS 0x81

struct i2c_ans_cobboard_status {
	struct i2c_cmd_hdr hdr;

	/* mode type are defined above: I2C_COBBOARD_MODE_xxx */
	uint8_t mode;

#define I2C_COBBOARD_STATUS_READY          0x00
#define I2C_COBBOARD_STATUS_OFF            0x01
#define I2C_COBBOARD_STATUS_LBUSY          0x02
#define I2C_COBBOARD_STATUS_RBUSY          0x03
#define I2C_COBBOARD_STATUS_EJECT          0x04
#define I2C_COBBOARD_STATUS_KICKSTAND_UP   0x05
#define I2C_COBBOARD_STATUS_KICKSTAND_DOWN 0x06
	uint8_t status;

	uint8_t cob_count;

	int16_t left_cobroller_speed;
	int16_t right_cobroller_speed;
};

#define I2C_REQ_BALLBOARD_STATUS 0x82

struct i2c_req_ballboard_status {
	struct i2c_cmd_hdr hdr;
};

#define I2C_ANS_BALLBOARD_STATUS 0x83

struct i2c_ans_ballboard_status {
	struct i2c_cmd_hdr hdr;

	uint8_t mode;

#define I2C_BALLBOARD_STATUS_F_READY         0x00
#define I2C_BALLBOARD_STATUS_F_BUSY          0x01
#define I2C_BALLBOARD_STATUS_F_EXCPT         0x02
	uint8_t status;

	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;

	uint8_t ball_count;

	/* detection of cobs */
	uint8_t lcob;
	uint8_t rcob;
};

#endif /* _I2C_PROTOCOL_H_ */
