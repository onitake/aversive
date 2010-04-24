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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <aversive.h>
#include <aversive/error.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <ax12.h>
#include <pwm_ng.h>
#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "strat.h"
#include "strat_utils.h"
#include "main.h"

static int32_t l_pwm, r_pwm;
static int32_t l_enc, r_enc;

static int fdr, fdw;
/*
 * Debug with GDB:
 *
 * (gdb) handle SIGUSR1 pass
 * Signal        Stop	Print	Pass to program	Description
 * SIGUSR1       Yes	Yes	Yes		User defined signal 1
 * (gdb) handle SIGUSR2 pass
 * Signal        Stop	Print	Pass to program	Description
 * SIGUSR2       Yes	Yes	Yes		User defined signal 2
 * (gdb) handle SIGUSR1 noprint
 * Signal        Stop	Print	Pass to program	Description
 * SIGUSR1       No	No	Yes		User defined signal 1
 * (gdb) handle SIGUSR2 noprint
 */

/* */
#define FILTER  98
#define FILTER2 (100-FILTER)
#define SHIFT   4

void robotsim_dump(void)
{
	char buf[BUFSIZ];
	int len;
	int16_t x, y, a;

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);
	a = position_get_a_deg_s16(&mainboard.pos);
/* 	y = COLOR_Y(y); */
/* 	a = COLOR_A(a); */

	len = snprintf(buf, sizeof(buf), "pos=%d,%d,%d\n",
		       x, y, a);
	hostsim_lock();
	write(fdw, buf, len);
	hostsim_unlock();
}

static int8_t
robotsim_i2c_ballboard_set_mode(struct i2c_cmd_ballboard_set_mode *cmd)
{
	char buf[BUFSIZ];
	int len;

	ballboard.mode = cmd->mode;
	len = snprintf(buf, sizeof(buf), "ballboard=%d\n", cmd->mode);
	hostsim_lock();
	write(fdw, buf, len);
	hostsim_unlock();
	return 0;
}

int8_t
robotsim_i2c_cobboard_set_mode(uint8_t mode)
{
	char buf[BUFSIZ];
	int len;

	if (cobboard.mode == mode)
		return 0;

	cobboard.mode = mode;
	len = snprintf(buf, sizeof(buf), "cobboard=%d\n", mode);
	hostsim_lock();
	write(fdw, buf, len);
	hostsim_unlock();
	return 0;
}

static int8_t
robotsim_i2c_ballboard(uint8_t addr, uint8_t *buf, uint8_t size)
{
	void *void_cmd = buf;

	switch (buf[0]) {
	case I2C_CMD_BALLBOARD_SET_MODE:
		{
			struct i2c_cmd_ballboard_set_mode *cmd = void_cmd;
			robotsim_i2c_ballboard_set_mode(cmd);
			break;
		}

	default:
		break;
	}
	return 0;
}

static int8_t
robotsim_i2c_cobboard(uint8_t addr, uint8_t *buf, uint8_t size)
{
	//	void *void_cmd = buf;

	switch (buf[0]) {
#if 0 /* deleted */
	case I2C_CMD_COBBOARD_SET_MODE:
		{
			struct i2c_cmd_cobboard_set_mode *cmd = void_cmd;
			robotsim_i2c_cobboard_set_mode(cmd);
			break;
		}
#endif
	default:
		break;
	}
	return 0;
}

int8_t
robotsim_i2c(uint8_t addr, uint8_t *buf, uint8_t size)
{
	if (addr == I2C_BALLBOARD_ADDR)
		return robotsim_i2c_ballboard(addr, buf, size);
	else if (addr == I2C_COBBOARD_ADDR)
		return robotsim_i2c_cobboard(addr, buf, size);
	return 0;
}

/* must be called periodically */
void robotsim_update(void)
{
	static int32_t l_pwm_shift[SHIFT];
	static int32_t r_pwm_shift[SHIFT];
	static int32_t l_speed, r_speed;
	static unsigned i = 0;
	static unsigned cpt = 0;
	int32_t local_l_pwm, local_r_pwm;
	double x, y, a, a2, d;
	char cmd = 0;

	/* corners of the robot */
	double xfl, yfl; /* front left */
	double xrl, yrl; /* rear left */
	double xrr, yrr; /* rear right */
	double xfr, yfr; /* front right */

	/* time shift the command */
	l_pwm_shift[i] = l_pwm;
	r_pwm_shift[i] = r_pwm;
	i ++;
	i %= SHIFT;
	local_l_pwm = l_pwm_shift[i];
	local_r_pwm = r_pwm_shift[i];

	/* read command */
	if (((cpt ++) & 0x7) == 0) {
		if (read(fdr, &cmd, 1) != 1)
			cmd = 0;
	}

	x = position_get_x_double(&mainboard.pos);
	y = position_get_y_double(&mainboard.pos);
	a = position_get_a_rad_double(&mainboard.pos);

	l_speed = ((l_speed * FILTER) / 100) +
		((local_l_pwm * 1000 * FILTER2)/1000);
	r_speed = ((r_speed * FILTER) / 100) +
		((local_r_pwm * 1000 * FILTER2)/1000);

	/* basic collision detection */
	a2 = atan2(ROBOT_WIDTH/2, ROBOT_HALF_LENGTH_REAR);
	d = norm(ROBOT_WIDTH/2, ROBOT_HALF_LENGTH_REAR);

	xfl = x + cos(a+a2) * d;
	yfl = y + sin(a+a2) * d;
	if (!is_in_area(xfl, yfl, 0) && l_speed > 0)
		l_speed = 0;

	xrl = x + cos(a+M_PI-a2) * d;
	yrl = y + sin(a+M_PI-a2) * d;
	if (!is_in_area(xrl, yrl, 0) && l_speed < 0)
		l_speed = 0;

	xrr = x + cos(a+M_PI+a2) * d;
	yrr = y + sin(a+M_PI+a2) * d;
	if (!is_in_area(xrr, yrr, 0) && r_speed < 0)
		r_speed = 0;

	xfr = x + cos(a-a2) * d;
	yfr = y + sin(a-a2) * d;
	if (!is_in_area(xfr, yfr, 0) && r_speed > 0)
		r_speed = 0;

	/* perturbation */
	if (cmd == 'l')
		l_enc += 5000; /* push 1 cm */
	if (cmd == 'r')
		r_enc += 5000; /* push 1 cm */

	/* XXX should lock */
	l_enc += (l_speed / 1000);
	r_enc += (r_speed / 1000);
}

void robotsim_pwm(void *arg, int32_t val)
{
	//	printf("%p, %d\n", arg, val);
	if (arg == LEFT_PWM)
		l_pwm = (val / 1.55);
	else if (arg == RIGHT_PWM)
		r_pwm = (val / 1.55);
}

int32_t robotsim_encoder_get(void *arg)
{
	if (arg == LEFT_ENCODER)
		return l_enc;
	else if (arg == RIGHT_ENCODER)
		return r_enc;
	return 0;
}

int robotsim_init(void)
{
	mkfifo("/tmp/.robot_sim2dis", 0600);
	mkfifo("/tmp/.robot_dis2sim", 0600);
	fdw = open("/tmp/.robot_sim2dis", O_WRONLY, 0);
	if (fdw < 0)
		return -1;
	fdr = open("/tmp/.robot_dis2sim", O_RDONLY | O_NONBLOCK, 0);
	if (fdr < 0) {
		close(fdw);
		return -1;
	}
	return 0;
}
