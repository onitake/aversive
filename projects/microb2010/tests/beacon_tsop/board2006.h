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

/* LEDs */

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED1_PORT PORTE
#define LED1_DDR  DDRE
#define LED1_BIT  2
#define LED2_PORT PORTE
#define LED2_DDR  DDRE
#define LED2_BIT  3
#define LED3_PORT PORTB
#define LED3_DDR  DDRB
#define LED3_BIT  3
#define LED_DDR_INIT()  do {						\
		LED1_DDR |= _BV(LED1_BIT);				\
		LED2_DDR |= _BV(LED2_BIT);				\
		LED3_DDR |= _BV(LED3_BIT);				\
	} while(0)
#define LED1_ON()  sbi(LED1_PORT, LED1_BIT)
#define LED1_OFF() cbi(LED1_PORT, LED1_BIT)
#define LED1_TOGGLE() LED_TOGGLE(LED_PORT, LED1_BIT)
#define LED2_ON()  sbi(LED2_PORT, LED2_BIT)
#define LED2_OFF() cbi(LED2_PORT, LED2_BIT)
#define LED2_TOGGLE() LED_TOGGLE(LED2_PORT, LED2_BIT)
#define LED3_ON()  sbi(LED3_PORT, LED3_BIT)
#define LED3_OFF() cbi(LED3_PORT, LED3_BIT)
#define LED3_TOGGLE() LED_TOGGLE(LED3_PORT, LED3_BIT)

/* TSOP */

#define EICRx_TSOP EICRB /* EICRA is not ok, cannot do intr on any edge */

#define INTx_TSOP_OPP  INT6
#define ISCx0_TSOP_OPP ISC60
#define ISCx1_TSOP_OPP ISC61
#define SIG_TSOP_OPP   SIG_INTERRUPT6
#define TSOP_OPP_READ() (!(PINE & 0x40))

#define INTx_TSOP_STA  INT7
#define ISCx0_TSOP_STA ISC70
#define ISCx1_TSOP_STA ISC71
#define SIG_TSOP_STA   SIG_INTERRUPT7
#define TSOP_STA_READ() (!(PINE & 0x80))

#define TSOP_FREQ_455_KHZ 0.455
#define N_PERIODS_455   10.
#define TSOP_FREQ_38_KHZ 0.038
#define N_PERIODS_38   15.
#define TSOP_FREQ_30_KHZ 0.030
#define N_PERIODS_30   15.
#define TSOP_FREQ_56_KHZ 0.056
#define N_PERIODS_56   15.

/* TSOP STATIC */

#define TSOP_STA_PERIOD_US (1./TSOP_FREQ_38_KHZ)
#define TSOP_STA_N_PERIODS  (N_PERIODS_38)

#define TSOP_STA_TIME_SHORT_US (1.5 * TSOP_STA_N_PERIODS * TSOP_STA_PERIOD_US)
#define TSOP_STA_TIME_LONG_US  (2.5 * TSOP_STA_N_PERIODS * TSOP_STA_PERIOD_US)

#define TSOP_STA_TIME_SHORT ((uint16_t)(TSOP_STA_TIME_SHORT_US/4))
#define TSOP_STA_TIME_LONG  ((uint16_t)(TSOP_STA_TIME_LONG_US/4))

#define TSOP_STA_FRAME_LEN 16
#define TSOP_STA_FRAME_MASK ((1UL << TSOP_STA_FRAME_LEN) - 1)

#define TSOP_STA_BEACON_ID_MASK 0x7
#define TSOP_STA_BEACON_ID_SHIFT 0
#define TSOP_STA_FRAME_DATA_MASK  0xFF8
#define TSOP_STA_FRAME_DATA_SHIFT 3

#define TSOP_STA_BEACON_ID0 0x03
#define TSOP_STA_BEACON_ID1 0x05

/* TSOP OPP */

#define TSOP_OPP_PERIOD_US (1./TSOP_FREQ_455_KHZ)
#define TSOP_OPP_N_PERIODS  (N_PERIODS_455)

#define TSOP_OPP_TIME_SHORT_US (1.5 * TSOP_OPP_N_PERIODS * TSOP_OPP_PERIOD_US)
#define TSOP_OPP_TIME_LONG_US  (2.5 * TSOP_OPP_N_PERIODS * TSOP_OPP_PERIOD_US)

#define TSOP_OPP_TIME_SHORT ((uint16_t)(TSOP_OPP_TIME_SHORT_US/4))
#define TSOP_OPP_TIME_LONG  ((uint16_t)(TSOP_OPP_TIME_LONG_US/4))

#define TSOP_OPP_FRAME_LEN 16
#define TSOP_OPP_FRAME_MASK ((1UL << TSOP_OPP_FRAME_LEN) - 1)

#define TSOP_OPP_BEACON_ID_MASK 0x7
#define TSOP_OPP_BEACON_ID_SHIFT 0
#define TSOP_OPP_FRAME_DATA_MASK  0xFF8
#define TSOP_OPP_FRAME_DATA_SHIFT 3

#define TSOP_OPP_BEACON_ID 0x01
