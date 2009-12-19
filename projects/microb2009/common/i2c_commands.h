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
 *  Revision : $Id: i2c_commands.h,v 1.9 2009-05-27 20:04:06 zer0 Exp $
 *
 */

#ifndef _I2C_COMMANDS_H_
#define _I2C_COMMANDS_H_

#define I2C_MAINBOARD_ADDR   1
#define I2C_MECHBOARD_ADDR   2
#define I2C_SENSORBOARD_ADDR 3

#define I2C_LEFT_SIDE   0
#define I2C_RIGHT_SIDE  1
#define I2C_AUTO_SIDE   2 /* for prepare_pickup */
#define I2C_CENTER_SIDE 3 /* for prepare_pickup */

#define I2C_COLOR_RED   0
#define I2C_COLOR_GREEN 1

#define I2C_AUTOBUILD_DEFAULT_DIST 210

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

#define I2C_CMD_MECHBOARD_SET_MODE 0x02

struct i2c_cmd_mechboard_set_mode {
	struct i2c_cmd_hdr hdr;
#define I2C_MECHBOARD_MODE_MANUAL             0x00
#define I2C_MECHBOARD_MODE_HARVEST            0x01
#define I2C_MECHBOARD_MODE_PREPARE_PICKUP     0x02
#define I2C_MECHBOARD_MODE_PICKUP             0x03
#define I2C_MECHBOARD_MODE_PREPARE_BUILD      0x04
#define I2C_MECHBOARD_MODE_AUTOBUILD          0x05
#define I2C_MECHBOARD_MODE_WAIT               0x06
#define I2C_MECHBOARD_MODE_INIT               0x07
#define I2C_MECHBOARD_MODE_PREPARE_GET_LINTEL 0x08
#define I2C_MECHBOARD_MODE_GET_LINTEL         0x09
#define I2C_MECHBOARD_MODE_PUT_LINTEL         0x0A
#define I2C_MECHBOARD_MODE_PREPARE_EJECT      0x0B
#define I2C_MECHBOARD_MODE_EJECT              0x0C
#define I2C_MECHBOARD_MODE_CLEAR              0x0D
#define I2C_MECHBOARD_MODE_LAZY_HARVEST       0x0E
#define I2C_MECHBOARD_MODE_LOADED             0x0F
#define I2C_MECHBOARD_MODE_PREPARE_INSIDE     0x10
#define I2C_MECHBOARD_MODE_STORE              0x11
#define I2C_MECHBOARD_MODE_LAZY_PICKUP        0x12
#define I2C_MECHBOARD_MODE_MANIVELLE          0x13
#define I2C_MECHBOARD_MODE_PUSH_TEMPLE        0x14
#define I2C_MECHBOARD_MODE_PUSH_TEMPLE_DISC   0x15
#define I2C_MECHBOARD_MODE_EXIT               0xFF
	uint8_t mode;
	union {
		struct {

		} manual;

		struct {
			uint8_t side;
			uint8_t next_mode;
		} prep_pickup;

		struct {
			uint8_t level_l;
			uint8_t level_r;
		} prep_build;

		struct {
			uint8_t side;
		} push_temple_disc;

		struct {
			uint8_t level_left;
			uint8_t level_right;
			uint8_t count_left;
			uint8_t count_right;
			uint8_t distance_left;
			uint8_t distance_right;
			uint8_t do_lintel;
		} autobuild;

		struct {
			uint8_t level_l;
			uint8_t level_r;
		} prep_inside;
	};
};

/****/

/* only valid in manual mode */
#define I2C_CMD_MECHBOARD_ARM_GOTO 0x03

struct i2c_cmd_mechboard_arm_goto {
	struct i2c_cmd_hdr hdr;
#define I2C_MECHBOARD_ARM_LEFT     0
#define I2C_MECHBOARD_ARM_RIGHT    1
#define I2C_MECHBOARD_ARM_BOTH     2
	uint8_t which;

	uint8_t height; /* in cm */
	uint8_t distance; /* in cm */
};

/****/

#define I2C_CMD_SENSORBOARD_SET_BEACON 0x04

struct i2c_cmd_sensorboard_start_beacon {
	struct i2c_cmd_hdr hdr;
	uint8_t enable;
};


/****/

#define I2C_CMD_SENSORBOARD_SET_SCANNER 0x05

struct i2c_cmd_sensorboard_scanner {
	struct i2c_cmd_hdr hdr;

#define I2C_SENSORBOARD_SCANNER_STOP    0x00
#define I2C_SENSORBOARD_SCANNER_PREPARE 0x01
#define I2C_SENSORBOARD_SCANNER_START   0x02
	uint8_t mode;
};

/*****/

#define I2C_CMD_SENSORBOARD_CALIB_SCANNER 0x06
struct i2c_cmd_sensorboard_calib_scanner {
	struct i2c_cmd_hdr hdr;
};

/*****/

#define I2C_CMD_SENSORBOARD_SCANNER_ALGO 0x07
struct i2c_cmd_sensorboard_scanner_algo {
	struct i2c_cmd_hdr hdr;

#define I2C_SCANNER_ALGO_COLUMN_DROPZONE 1
#define I2C_SCANNER_ALGO_CHECK_TEMPLE    2
#define I2C_SCANNER_ALGO_TEMPLE_DROPZONE 3
	uint8_t algo;

	union {
		struct {
#define I2C_SCANNER_ZONE_0     0
#define I2C_SCANNER_ZONE_1     1
#define I2C_SCANNER_ZONE_DISC  2
			uint8_t working_zone;
			int16_t center_x;
			int16_t center_y;
		} drop_zone;
		
		struct {
			uint8_t level;
			int16_t temple_x;
			int16_t temple_y;
		} check_temple;
	};
};

/****/
/* requests and their answers */
/****/


#define I2C_REQ_MECHBOARD_STATUS 0x80

struct i2c_req_mechboard_status {
	struct i2c_cmd_hdr hdr;

	int16_t pump_left1_current;
	int16_t pump_left2_current;
};

#define I2C_ANS_MECHBOARD_STATUS 0x81

struct i2c_ans_mechboard_status {
	struct i2c_cmd_hdr hdr;
	/* mode type are defined above: I2C_MECHBOARD_MODE_xxx */
	uint8_t mode;

#define I2C_MECHBOARD_STATUS_F_READY         0x00
#define I2C_MECHBOARD_STATUS_F_BUSY          0x01
#define I2C_MECHBOARD_STATUS_F_EXCPT         0x02
	uint8_t status;

	uint8_t lintel_count;

	/* flag is there if column was taken by this pump. Note that
	 * we should also check ADC (current) to see if the column is
	 * still there. */
#define I2C_MECHBOARD_COLUMN_L1              0x01
#define I2C_MECHBOARD_COLUMN_L2              0x02
#define I2C_MECHBOARD_COLUMN_R1              0x04
#define I2C_MECHBOARD_COLUMN_R2              0x08
	uint8_t column_flags;
	
	int16_t pump_left1;
	int16_t pump_right1;
	int16_t pump_left2;
	int16_t pump_right2;

#define I2C_MECHBOARD_CURRENT_COLUMN 85
	int16_t pump_right1_current;
	int16_t pump_right2_current;

	uint16_t servo_lintel_left;
	uint16_t servo_lintel_right;
};

#define I2C_REQ_SENSORBOARD_STATUS 0x82

struct i2c_req_sensorboard_status {
	struct i2c_cmd_hdr hdr;

	/* position sent by mainboard */
	int16_t x;
	int16_t y;
	int16_t a;

	/* PWM for pickup */
	uint8_t enable_pickup_wheels;
};

#define I2C_ANS_SENSORBOARD_STATUS 0x83

struct i2c_ans_sensorboard_status {
	struct i2c_cmd_hdr hdr;

	uint8_t status;
#define I2C_OPPONENT_NOT_THERE -1000
	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;

#define I2C_SCAN_DONE 1
#define I2C_SCAN_MAX_COLUMN 2
	uint8_t scan_status;

#define I2C_COLUMN_NO_DROPZONE -1
	int8_t dropzone_h;
	int16_t dropzone_x;
	int16_t dropzone_y;
};

#endif /* _I2C_PROTOCOL_H_ */
