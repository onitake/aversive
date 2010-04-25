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
 *  Revision : $Id: main.h,v 1.10 2009-11-08 17:24:33 zer0 Exp $
 *
 */

/* was mechboard in 2009 */

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#ifdef HOST_VERSION
#define LED1_ON()
#define LED1_OFF()
#define LED1_TOGGLE()

#define LED2_ON()
#define LED2_OFF()
#define LED2_TOGGLE()

#define LED3_ON()
#define LED3_OFF()
#define LED3_TOGGLE()

#define LED4_ON()
#define LED4_OFF()
#define LED4_TOGGLE()

#define BRAKE_DDR()
#define BRAKE_ON()
#define BRAKE_OFF()

#else

#define LED1_ON() 	sbi(PORTJ, 2)
#define LED1_OFF() 	cbi(PORTJ, 2)
#define LED1_TOGGLE() 	LED_TOGGLE(PORTJ, 2)

#define LED2_ON() 	sbi(PORTJ, 3)
#define LED2_OFF() 	cbi(PORTJ, 3)
#define LED2_TOGGLE() 	LED_TOGGLE(PORTJ, 3)

#define LED3_ON() 	sbi(PORTL, 7)
#define LED3_OFF() 	cbi(PORTL, 7)
#define LED3_TOGGLE() 	LED_TOGGLE(PORTL, 7)

#define LED4_ON() 	sbi(PORTL, 6)
#define LED4_OFF() 	cbi(PORTL, 6)
#define LED4_TOGGLE() 	LED_TOGGLE(PORTL, 6)

#define BRAKE_DDR()     do { DDRJ |= 0xF0; } while(0)
#define BRAKE_ON()      do { PORTJ |= 0xF0; } while(0)
#define BRAKE_OFF()     do { PORTJ &= 0x0F; } while(0)
#endif

/* only 90 seconds, don't forget it :) */
#define MATCH_TIME 89

/* decrease track to decrease angle */
#define EXT_TRACK_MM 304.61875
#define VIRTUAL_TRACK_MM EXT_TRACK_MM

#define ROBOT_HALF_LENGTH_FRONT 130
#define ROBOT_HALF_LENGTH_REAR 120
#define ROBOT_WIDTH 320

/* it is a 1024 imps -> 4096 because we see 1/4 period
 * and diameter: 55mm -> perimeter 134mm
 * dist_imp_mm = 4096/134 x 10 -> 304 */
/* increase it to go further */
#define IMP_ENCODERS 1024
#define WHEEL_DIAMETER_MM 42.9
#define WHEEL_PERIM_MM (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 10.
#define DIST_IMP_MM (((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)

#define RIGHT_ENCODER           ((void *)0)
#define LEFT_ENCODER            ((void *)1)
#define LEFT_COBROLLER_ENCODER  ((void *)2)
#define RIGHT_COBROLLER_ENCODER ((void *)3)

#define RIGHT_PWM           ((void *)&gen.pwm1_4A)
#define LEFT_PWM            ((void *)&gen.pwm2_4B)
#define LEFT_COBROLLER_PWM  ((void *)&gen.pwm3_1A)
#define RIGHT_COBROLLER_PWM ((void *)&gen.pwm4_1B)

#define SUPPORT_BALLS_R_SERVO ((void *)&gen.servo2)
#define SUPPORT_BALLS_L_SERVO ((void *)&gen.servo3)


/** ERROR NUMS */
#define E_USER_STRAT           194
#define E_USER_I2C_PROTO       195
#define E_USER_SENSOR          196
#define E_USER_CS              197

#define LED_PRIO           170
#define TIME_PRIO          160
#define ADC_PRIO           120
#define CS_PRIO            100
#define STRAT_PRIO          30
#define I2C_POLL_PRIO       20
#define EEPROM_TIME_PRIO    10

#define CS_PERIOD 5000L /* in microsecond */
#define CS_HZ (1000000. / CS_PERIOD)

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

/* mainboard specific */
struct mainboard {
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_RS        4
#define DO_POS       8
#define DO_BD       16
#define DO_TIMER    32
#define DO_POWER    64
#define DO_ERRBLOCKING 128
	uint8_t flags;                /* misc flags */

	/* control systems */
        struct cs_block angle;
        struct cs_block distance;
        struct cs_block left_cobroller;
        struct cs_block right_cobroller;

	/* x,y positionning */
	struct robot_system rs;
	struct robot_position pos;
        struct trajectory traj;

	/* robot status */
	uint8_t our_color;
	volatile int16_t speed_a;     /* current angle speed */
	volatile int16_t speed_d;     /* current dist speed */
	int32_t pwm_l;                /* current left pwm */
	int32_t pwm_r;                /* current right pwm */
};

/* state of cobboard, synchronized through i2c */
struct cobboard {
	uint8_t mode;
	uint8_t status;
	uint8_t lspickle;
	uint8_t rspickle;
	int16_t left_cobroller_speed;
	int16_t right_cobroller_speed;
	uint8_t cob_count;
};

/* state of ballboard, synchronized through i2c */
struct ballboard {
	volatile uint8_t mode;
	uint8_t status;
	uint8_t ball_count;
	uint8_t lcob;
	uint8_t rcob;
};

extern struct genboard gen;
extern struct mainboard mainboard;
extern volatile struct cobboard cobboard;
extern volatile struct ballboard ballboard;

/* start the bootloader */
void bootloader(void);

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
	if (__ret)					      \
		DEBUG(E_USER_STRAT, "cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})
