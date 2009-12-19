/*  
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: main.h,v 1.1 2009-02-20 21:10:01 zer0 Exp $
 *
 */

#define LED1_ON() 	sbi(PORTJ, 2)
#define LED1_OFF() 	cbi(PORTJ, 2)

#define LED2_ON() 	sbi(PORTJ, 3)
#define LED2_OFF() 	cbi(PORTJ, 3)

#define ARM_MOT_PWM     (&arm.pwm1A)
#define ARM_ENC         ((void *)0)

#define LED_PRIO           170
#define TIME_PRIO          160
#define CS_PRIO            150

struct arm {
#define CS_ON 1
	uint8_t flags;
        struct cs cs_mot;
        struct pid_filter pid_mot;
	struct quadramp_filter qr_mot;

	/* motors */
	struct pwm_ng pwm1_2A;
	struct pwm_ng pwm2_1A;
	struct pwm_ng pwm3_1B;
	struct pwm_ng pwm4_1C;

	/* servos */
	struct pwm_ng servo1;
	struct pwm_ng servo2;
	struct pwm_ng servo3;
	struct pwm_ng servo4;
	struct pwm_ng servo5;
	struct pwm_ng servo6;
};

extern struct arm arm;
