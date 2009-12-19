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
 *  Revision : $Id: main.h,v 1.6 2009-11-08 17:25:00 zer0 Exp $
 *
 */

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED1_ON() 	sbi(PORTJ, 2)
#define LED1_OFF() 	cbi(PORTJ, 2)
#define LED1_TOGGLE() 	LED_TOGGLE(PORTJ, 2)

#define LED2_ON() 	sbi(PORTL, 7)
#define LED2_OFF() 	cbi(PORTL, 7)
#define LED2_TOGGLE() 	LED_TOGGLE(PORTL, 7)

#define LED3_ON() 	sbi(PORTJ, 3)
#define LED3_OFF() 	cbi(PORTJ, 3)
#define LED3_TOGGLE() 	LED_TOGGLE(PORTJ, 3)

#define LED4_ON() 	sbi(PORTL, 6)
#define LED4_OFF() 	cbi(PORTL, 6)
#define LED4_TOGGLE() 	LED_TOGGLE(PORTL, 6)

#define BRAKE_DDR()     do { DDRJ |= 0xF0; } while(0)
#define BRAKE_ON()      do { PORTJ |= 0xF0; } while(0)
#define BRAKE_OFF()     do { PORTJ &= 0x0F; } while(0)

#define LEFT_ARM_ENCODER        ((void *)0)
#define RIGHT_ARM_ENCODER       ((void *)1)

#define LEFT_ARM_PWM            ((void *)&gen.pwm1_4A)
#define RIGHT_ARM_PWM           ((void *)&gen.pwm2_4B)

#define RIGHT_PUMP1_PWM         ((void *)&gen.pwm3_1A)
#define RIGHT_PUMP2_PWM         ((void *)&gen.pwm4_1B)

#define R_ELBOW_AX12 1
#define R_WRIST_AX12 2
#define L_ELBOW_AX12 4
#define L_WRIST_AX12 3
#define FINGER_AX12  5

/** ERROR NUMS */
#define E_USER_I2C_PROTO       195
#define E_USER_SENSOR          196
#define E_USER_ARM             197
#define E_USER_FINGER          198
#define E_USER_ST_MACH         199
#define E_USER_CS              200
#define E_USER_AX12            201

#define LED_PRIO           170
#define TIME_PRIO          160
#define ADC_PRIO           120
#define CS_PRIO            100
#define ARM_PRIO            50
#define I2C_POLL_PRIO       20

#define CS_PERIOD 5000L

#define NB_LOGS 4

/* generic to all boards */
struct genboard {
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct pwm_ng pwm1_4A;
	struct pwm_ng pwm2_4B;
	struct pwm_ng pwm3_1A;
	struct pwm_ng pwm4_1B;

	/* servos */
	struct pwm_ng servo1;
	struct pwm_ng servo2;
	struct pwm_ng servo3;
	struct pwm_ng servo4;
	
	/* ax12 interface */
	AX12 ax12;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

struct cs_block {
	uint8_t on;
        struct cs cs;
        struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* mechboard specific */
struct mechboard {
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8
	uint8_t flags;                /* misc flags */

	/* control systems */
        struct cs_block left_arm;
        struct cs_block right_arm;

	/* robot status */
	uint8_t our_color;
	volatile uint8_t lintel_count;
	volatile uint8_t column_flags;
	volatile uint8_t status;

	/* local pumps */
	int16_t pump_right1;
	int16_t pump_right2;
	/* remote pump (on mainboard) */
	int16_t pump_left1;
	int16_t pump_left2;

	/* remote pump current */
	int16_t pump_left1_current;
	int16_t pump_left2_current;

	uint16_t servo_lintel_left;
	uint16_t servo_lintel_right;

};

extern struct genboard gen;
extern struct mechboard mechboard;

/* start the bootloader */
void bootloader(void);

#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
        __ret;                                                \
})
