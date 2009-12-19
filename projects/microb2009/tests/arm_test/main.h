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
 *  Revision : $Id: main.h,v 1.3 2009-03-15 20:08:51 zer0 Exp $
 *
 */

#define LED1_ON() 	sbi(PORTE, 2)
#define LED1_OFF() 	cbi(PORTE, 2)

#define LED2_ON() 	sbi(PORTE, 3)
#define LED2_OFF() 	cbi(PORTE, 3)

#define ARM_MOT_PWM     (&arm.pwm1B)
#define SCANNER_MOT_PWM (&arm.pwm1A)


#define ARM_ENC         ((void *)0)
#define SCANNER_ENC         ((void *)1)

#define CS_PERIOD 5000L
#define LED_PRIO           170
#define TIME_PRIO          160
#define CS_PRIO            150

struct arm {
#define CS_ON 1
	uint8_t flags;
        struct cs cs_mot;
        struct pid_filter pid_mot;
	struct quadramp_filter qr_mot;
	struct pwm_ng pwm1A;
	struct pwm_ng pwm1B;
	struct pwm_ng pwm3C;
	struct pwm_ng pwm2;
};

#define MAX_SAMPLE (1200L)
 /* was 1000 */

extern struct arm arm;
extern struct arm scanner;

extern uint8_t sample_tab[MAX_SAMPLE];

extern int32_t last_tour_n;
extern int32_t last_tour_pos;

//64*14*4 (+1?)
#define SCANNER_STEP_TOUR (3532L)

extern float offset_a;
