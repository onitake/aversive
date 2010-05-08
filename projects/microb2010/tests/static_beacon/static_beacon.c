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

#include <math.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>

#include <uart.h>

/*
 * Leds: PD5 -> PD7
 * Photodiodes: PC0 PC1
 * IR: PB1, OC1A
 */

/*
 * hfuse:  RSTDISBL=1 WTDON=1 SPIEN=0 CKOPT=0 EESAVE=1 BOOTSZ1=0 BOOTSZ0=0 BOOTRST=1
 * lfuse:  BODLEVEL=1 BODEN=1 SUT1=1 SUT0=1 CKSEL3=1 CKSEL2=1 CKSEL1=1 CKSEL0=1
 */

//#define NO_MODULATION
#define WAIT_LASER
//#define MODUL_455KHZ
#define MODUL_56KHZ
//#define MODUL_38KHZ
//#define INC_FRAME
//#define FIXED_FRAME
//#define SEND_RAWTIME
#define LOCK_BUS
#define SPEED_20RPS

/* beacon identifier: must be odd, 3 bits */
#define BEACON_ID 0x1
#define BEACON_ID_MASK 0x7
#define BEACON_ID_SHIFT 0

/* */
#define FRAME_DATA_MASK  0x0FF8
#define FRAME_DATA_SHIFT 3

#if (defined MODUL_455KHZ)
#define N_PERIODS   15
#define N_CYCLES_0  17
#define N_CYCLES_1  17
#elif (defined MODUL_56KHZ)
#define N_PERIODS   15
#define N_CYCLES_0  143
#define N_CYCLES_1  143
#elif (defined MODUL_38KHZ)
#define N_PERIODS   15
#define N_CYCLES_0  210
#define N_CYCLES_1  210
#else
#error "no freq defined"
#endif

#define MIN_DIST 150.
#define MAX_DIST 3600.
#define LASER_DIST 25.

#define N_CYCLES_PERIOD (N_CYCLES_0 + N_CYCLES_1)

#define LED_PORT PORTD
#define LED_DDR  DDRD
#define LED1_BIT  5
#define LED2_BIT  6
#define LED3_BIT  7

#define LED_TOGGLE(port, bit) do {		\
	  if (port & _BV(bit))			\
		  port &= ~_BV(bit);		\
	  else					\
		  port |= _BV(bit);		\
  } while(0)

#define LED1_ON()  sbi(LED_PORT, LED1_BIT)
#define LED1_OFF() cbi(LED_PORT, LED1_BIT)
#define LED1_TOGGLE() LED_TOGGLE(LED_PORT, LED1_BIT)
#define LED2_ON()  sbi(LED_PORT, LED2_BIT)
#define LED2_OFF() cbi(LED_PORT, LED2_BIT)
#define LED2_TOGGLE() LED_TOGGLE(LED_PORT, LED2_BIT)
#define LED3_ON()  sbi(LED_PORT, LED3_BIT)
#define LED3_OFF() cbi(LED_PORT, LED3_BIT)
#define LED3_TOGGLE() LED_TOGGLE(LED_PORT, LED3_BIT)

#define IR_PORT PORTB
#define IR_DDR  DDRB
#define IR_BIT  1

#define LOCKBUS_PORT PORTD
#define LOCKBUS_PIN  PIND
#define LOCKBUS_DDR  DDRD
#define LOCKBUS_BIT  3

/* FRAME must be odd */
/* #define FRAME 0x0B /\* in little endian 1-1-0-1 *\/ */
/* #define FRAME_LEN 4 */
#define FRAME 0x621 /* in little endian */
#define FRAME_LEN 16

/* pin returns !0 when nothing, and 0 when laser is on photodiode */
#define PHOTO_PIN PINB
#define PHOTO_BIT 0
#define READ_PHOTO() (!!(PHOTO_PIN & (_BV(PHOTO_BIT))))

/* IR_DELAY must be < 32768 */
#define MIN_INTER_TIME    ((uint16_t)(100*16U))   /* t~=160us dist=400cm */
#define MAX_INTER_TIME    ((uint16_t)(4000*16U)) /* t=4ms dist=16cm */
#define IR_DELAY          ((uint16_t)(MAX_INTER_TIME/2))
#define INTER_LASER_TIME   10 /* in ms */

extern prog_uint16_t framedist_table[];
/* basic functions to transmit on IR */

static inline void xmit_0(void)
{
	uint16_t t = ((N_CYCLES_PERIOD * N_PERIODS) / 4);
#ifdef NO_MODULATION
	cbi(IR_PORT, IR_BIT);
#else
	TCCR1B = 0;
	TCCR1A = 0;
#endif
	_delay_loop_2(t); /* 4 cycles per loop */
}

static inline void xmit_1(void)
{
	uint16_t t = ((N_CYCLES_PERIOD * N_PERIODS) / 4);
#ifdef NO_MODULATION
	sbi(IR_PORT, IR_BIT);
#else
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TCNT1 = N_CYCLES_PERIOD-1;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	ICR1 = N_CYCLES_PERIOD;
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
#endif
	_delay_loop_2(t); /* 4 cycles per loop */
}

/* transmit in manchester code */

static inline void xmit_manchester_0(void)
{
	xmit_0();
	xmit_1();
}

static inline void xmit_manchester_1(void)
{
	xmit_1();
	xmit_0();
}

/* transmit a full frame. Each byte is lsb first. */

static void xmit_bits(uint32_t frame, uint8_t nbit)
{
	uint8_t i;

	for (i=0; i<nbit; i++) {
		if (frame & 1)
			xmit_manchester_1();
		else
			xmit_manchester_0();

		/* next bit */
		frame >>= 1UL;
	}
	xmit_0();
}

/* val is 12 bits. Return the 16 bits value that includes the 4 bits
 * cksum in MSB. */
static uint16_t do_cksum(uint16_t val)
{
	uint16_t x, cksum;

	x = (val & 0xfff);
	/* add the three 4-bits blocks of x together */
	cksum = x & 0xf;
	x = x >> 4;
	cksum += x & 0xf;
	cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4);
	x = x >> 4;
	cksum += x & 0xf;
	cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4);
	cksum = (~cksum) & 0xf;
	return (cksum << 12) + (val & 0xfff);
}

/* get the frame from laser time difference, return 0 on error (this
 * frame cannot be sent). */
static uint32_t get_frame(uint16_t laserdiff)
{
	uint32_t frame = 0;
	uint16_t val, mid, min, max;

	/* for calibration, return the time */
#ifdef INC_FRAME
	static uint16_t fr = 1;
	fr += 2;
	return fr;
#endif
#ifdef SEND_RAWTIME
	return laserdiff | 1; /* frame must be odd */
#endif

	min = 0;
	max = 511;
	while (min != max) {
		mid = (max + min + 1) / 2;
		val = pgm_read_word(&framedist_table[mid]);

		if (laserdiff > val)
			max = mid - 1;
		else
			min = mid;
	}

	frame |= ((uint32_t)((min << FRAME_DATA_SHIFT)
			     & FRAME_DATA_MASK));
	frame |= BEACON_ID;

	/* process cksum and return */
	return do_cksum(frame);
}

/* Wait 2 consecutive rising edges on photodiode. Return 0 on success,
 * in this case, the 'when' pointed area is assigned to the time when
 * IR signal should be sent. The 'laserdiff' pointer is the time
 * between the 2 lasers, in timer unit. */
static inline int8_t wait_laser(uint16_t *when, uint16_t *laserdiff)
{
	uint16_t time1, time2;
	uint16_t diff;

	/* set timer to 16Mhz, we will use ICP */
	TCCR1A = 0;
	TCCR1B = _BV(CS10);

	/* wait until all is off (inverted logic) */
	while (READ_PHOTO() == 0);
	TIFR = _BV(ICF1);

	/* wait falling edge */
	while ((TIFR & _BV(ICF1)) == 0);
	time1 = ICR1;

	LED1_ON();

	/* wait a bit to avoid oscillations */
	while ((uint16_t)(TCNT1-time1) < MIN_INTER_TIME);
	TIFR = _BV(ICF1);

	/* wait next falling edge + timeout */
	while ((TIFR & _BV(ICF1)) == 0) {
		diff = TCNT1 - time1;
		if (diff > MAX_INTER_TIME)
			return -1;
	}

	LED2_ON();

	/* get time of 2nd laser */
	time2 = ICR1;
	TIFR = _BV(ICF1);

	/* process time difference */
	diff = time2 - time1;
	if (diff > MAX_INTER_TIME)
		return -1;

	/*
	 * diff/2 is < 32768
	 * ir_delay is < 32768
	 * current time is ~time1 + diff
	 */
	*when = time1 + (diff/2) + IR_DELAY;
	*laserdiff = diff;

	/* laser ok */
	return 0;
}

/* */

int main(void)
{
	/* must be odd */
	uint32_t frame = FRAME;
	int8_t ret;
	uint16_t when = 0;
	uint16_t diff = 0;

	/* LEDS */
	LED_DDR = _BV(LED1_BIT) | _BV(LED2_BIT) | _BV(LED3_BIT);
	IR_DDR |= _BV(IR_BIT);

	uart_init();
 	fdevopen(uart0_dev_send, uart0_dev_recv);
	sei();

#if 0
	while (1) {
		int16_t c;
		c = uart_recv_nowait(0);
		if (c != -1)
			printf("%c", (char)(c+1));
		LED1_ON();
		wait_ms(500);
		LED1_OFF();
		wait_ms(500);
	}
#endif

#if 0
	while (1) {
		if (READ_PHOTO())
			LED1_ON();
		else
			LED1_OFF();
	}
#endif

#ifndef NO_MODULATION
	/* configure PWM */
	ICR1 = N_CYCLES_PERIOD;
	OCR1A = N_CYCLES_1;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12);
#endif

#if 0
	/* test freq */
	ICR1 = N_CYCLES_PERIOD;
	OCR1A = N_CYCLES_1;
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TCNT1 = N_CYCLES_PERIOD-1;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
	while (1);
#endif

#if 0
	/* test freq ~ 400khz */
	ICR1 = 38;
	OCR1A = 19;
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TCNT1 = 37;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

	/* motor PWM 50%, 8khz */
	DDRB |= 0x08;
	OCR2 = 50;
	TCCR2 = _BV(WGM21) | _BV(WGM20) | _BV(COM21) | _BV(CS21) ;

	while (1);
#endif

#ifdef LOCK_BUS
	LOCKBUS_PORT &= (~_BV(LOCKBUS_BIT));
#endif

	/* configure timer 0, prescaler = 64 */
	TCCR0 = _BV(CS01) | _BV(CS00);

	while (1) {

#ifdef WAIT_LASER
		ret = wait_laser(&when, &diff);

		LED1_OFF();
		LED2_OFF();

		if (ret)
			continue;
#endif /* WAIT_LASER */

#ifdef LOCK_BUS
		if ((LOCKBUS_PIN & _BV(LOCKBUS_BIT)) == 0)
			continue;
		LOCKBUS_DDR |= _BV(LOCKBUS_BIT);
#endif

#ifdef FIXED_FRAME
		frame = FRAME;
#else
		frame = get_frame(diff);
#endif /* FIXED_FRAME */
		/* cannot convert into frame... skip it */
		if (frame == 0) {
			wait_ms(INTER_LASER_TIME);
			continue;
		}


#ifdef WAIT_LASER
		/* wait before IR xmit */
		while ((int16_t)(when-TCNT1) > 0);
#endif /* WAIT_LASER */

		LED3_ON();
		/* ok, transmit frame */
		xmit_bits(frame, FRAME_LEN);

		LED3_OFF();

#ifdef LOCK_BUS
		LOCKBUS_DDR &= (~_BV(LOCKBUS_BIT));
#endif

		/* don't watch a laser during this time */
		wait_ms(INTER_LASER_TIME);
	}
	return 0;
}
