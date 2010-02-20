/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: main.c,v 1.8 2009-05-02 10:08:09 zer0 Exp $
 *
 */

#define BOARD2006

/********************** LEDs */
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#ifdef BOARD2006
#define LED1_PORT PORTE
#define LED1_DDR  DDRE
#define LED1_BIT  2
#define LED2_PORT PORTE
#define LED2_DDR  DDRE
#define LED2_BIT  3
#define LED3_PORT PORTB
#define LED3_DDR  DDRB
#define LED3_BIT  3
#define LED1_ON()  sbi(LED1_PORT, LED1_BIT)
#define LED1_OFF() cbi(LED1_PORT, LED1_BIT)
#define LED1_TOGGLE() LED1_TOGGLE(LED_PORT, LED1_BIT)
#define LED2_ON()  sbi(LED2_PORT, LED2_BIT)
#define LED2_OFF() cbi(LED2_PORT, LED2_BIT)
#define LED2_TOGGLE() LED_TOGGLE(LED2_PORT, LED2_BIT)
#define LED3_ON()  sbi(LED3_PORT, LED3_BIT)
#define LED3_OFF() cbi(LED3_PORT, LED3_BIT)
#define LED3_TOGGLE() LED_TOGGLE(LED3_PORT, LED3_BIT)
#else
#define LED_PORT PORTD
#define LED_DDR  DDRD
#define LED1_BIT  5
#define LED2_BIT  6
#define LED3_BIT  7
#define LED1_ON()  sbi(LED_PORT, LED1_BIT)
#define LED1_OFF() cbi(LED_PORT, LED1_BIT)
#define LED1_TOGGLE() LED_TOGGLE(LED_PORT, LED1_BIT)
#define LED2_ON()  sbi(LED_PORT, LED2_BIT)
#define LED2_OFF() cbi(LED_PORT, LED2_BIT)
#define LED2_TOGGLE() LED_TOGGLE(LED_PORT, LED2_BIT)
#define LED3_ON()  sbi(LED_PORT, LED3_BIT)
#define LED3_OFF() cbi(LED_PORT, LED3_BIT)
#define LED3_TOGGLE() LED_TOGGLE(LED_PORT, LED3_BIT)
#endif

struct beacon_tsop {
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];
	struct pwm_ng pwm_motor;
	struct pid_filter pid;
	uint8_t debug_frame;
	uint8_t debug_speed;
};

extern struct beacon_tsop beacon_tsop;
