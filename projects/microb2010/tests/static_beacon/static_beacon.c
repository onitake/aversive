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

#include <aversive.h>
#include <aversive/wait.h>

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

#define N_PERIODS   10
#define N_CYCLES_0  17
#define N_CYCLES_1  17
#define N_CYCLES_PERIOD (N_CYCLES_0 + N_CYCLES_1)

#define LED_PORT PORTD
#define LED_DDR  DDRD
#define LED1_BIT  5
#define LED2_BIT  6
#define LED3_BIT  7

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
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

/* FRAME must be odd */
/* #define FRAME 0x0B /\* in little endian 1-1-0-1 *\/ */
/* #define FRAME_LEN 4 */
#define FRAME 0xAA5B /* in little endian */
#define FRAME_LEN 16

/* pin returns 1 when nothing, and 0 when laser is on photodiode */
#define PHOTO_PIN PINC
#define PHOTO1_BIT 0
#define PHOTO2_BIT 1
#define READ_PHOTOS() (PHOTO_PIN & (_BV(PHOTO1_BIT) | _BV(PHOTO2_BIT)))
#define READ_PHOTO1() (PHOTO_PIN & _BV(PHOTO1_BIT))
#define READ_PHOTO2() (PHOTO_PIN & _BV(PHOTO2_BIT))

#define PHOTOS_ALL_OFF (_BV(PHOTO1_BIT) | _BV(PHOTO2_BIT))
#define PHOTOS_ALL_ON (0)
#define PHOTOS_1ON_2OFF (_BV(PHOTO2_BIT))
#define PHOTOS_1OFF_2ON (_BV(PHOTO1_BIT))


/* in cycles/64 (unit is 4 us at 16Mhz) */
#define MAX_PHOTO_TIME ((uint8_t)25)  /* t=100us len=5mm rps=40Hz dist=20cm */

#define MIN_INTER_TIME ((uint8_t)12)  /* t=50us len=50mm rps=40Hz dist=350cm */
#define MAX_INTER_TIME ((uint8_t)250) /* t=1000us len=50mm rps=40Hz dist=20cm */

/* in ms */
#define INTER_LASER_TIME 10

#define NO_MODULATION
//#define WAIT_LASER

/* basic functions to transmit on IR */

static inline void xmit_0(void)
{
	uint8_t t = ((N_CYCLES_PERIOD * N_PERIODS) / 3);
#ifdef NO_MODULATION
	cbi(IR_PORT, IR_BIT);
#else
	TCCR1B = 0;
	TCCR1A = 0;
#endif
	_delay_loop_1(t); /* 3 cycles per loop */
}

static inline void xmit_1(void)
{
	uint8_t t = ((N_CYCLES_PERIOD * N_PERIODS) / 3);
#ifdef NO_MODULATION
	sbi(IR_PORT, IR_BIT);
#else
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TCNT1 = N_CYCLES_PERIOD-1;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
#endif
	_delay_loop_1(t); /* 3 cycles per loop */
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

/* */
static inline int8_t wait_laser(void)
{
	uint8_t photos;
	uint8_t time;
	uint8_t diff;

	/* wait until all is off */
	while (READ_PHOTOS() != PHOTOS_ALL_OFF);

	/* wait an event, it must be photo1 on */
	while ((photos = READ_PHOTOS()) == PHOTOS_ALL_OFF);

	if (photos != PHOTOS_1ON_2OFF)
		return -1;
	time = TCNT0;
	LED1_ON();

	/* wait an event, it must be photo1 off */
	while ((photos = READ_PHOTOS()) == PHOTOS_1ON_2OFF) {
		diff = TCNT0 - time;
		if (diff > MAX_PHOTO_TIME)
			return -1;
	}
	if (photos != PHOTOS_ALL_OFF)
		return -1;
	//time = TCNT0;

	/* wait an event, it must be photo2 on */
	while ((photos = READ_PHOTOS()) == PHOTOS_ALL_OFF) {
		diff = TCNT0 - time;
		if (diff > MAX_INTER_TIME)
			return -1;
	}
	LED2_ON();
	
	diff = TCNT0 - time;
	if (diff < MIN_INTER_TIME)
		return -1;
	if (photos != PHOTOS_1OFF_2ON)
		return -1;

#if 0	
	time = TCNT0;

	/* wait an event, it must be photo2 off */
	while ((photos = READ_PHOTOS()) == PHOTOS_1OFF_2ON) {
		diff = TCNT0 - time;
		if (diff > MAX_PHOTO_TIME)
			return -1;
	}
	if (photos != PHOTOS_ALL_OFF)
		return -1;
#endif

	/* laser ok */
	return 0;
}

/* */

int main(void)
{
	/* must be odd */
	uint32_t frame = FRAME;
	int8_t ret;
	int16_t c;

	/* LEDS */
	LED_DDR = _BV(LED1_BIT) | _BV(LED2_BIT) | _BV(LED3_BIT);
	IR_DDR |= _BV(IR_BIT);

	uart_init();
 	fdevopen(uart0_dev_send, uart0_dev_recv);
	sei();

#if 0
	while (1) {
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
		if (READ_PHOTO() & _BV(PHOTO1_BIT))
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

	/* configure timer 0, prescaler = 64 */
	TCCR0 = _BV(CS01) | _BV(CS00);

	while (1) {

#ifdef WAIT_LASER
		ret = wait_laser();
		
		LED1_OFF();
		LED2_OFF();

		if (ret)
			continue;
#endif
		
#if 1
		LED3_ON();
		/* ok, transmit frame */
		xmit_bits(frame, FRAME_LEN);
		
		/* don't watch a laser during this time */
		wait_ms(INTER_LASER_TIME);
		LED3_OFF();
#else
		LED1_ON();
		wait_ms(1);
		LED1_OFF();
#endif
	}
	return 0;
}
