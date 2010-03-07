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
#include <pid.h>
#include <pwm_ng.h>
#include <parse.h>
#include <rdline.h>

#include "cmdline.h"
#include "main.h"

/* beacon identifier: must be odd, 3 bits */
#define BEACON_ID_MASK 0x7
#define BEACON_ID_SHIFT 0
#define FRAME_DATA_MASK  0xFFF8
#define FRAME_DATA_SHIFT 3

/******************* TSOP */

#define EICRx_TSOP EICRB /* EICRA is not ok, cannot do intr on any edge */
#ifdef BOARD2006
#define INTx_TSOP  INT6
#define ISCx0_TSOP ISC60
#define ISCx1_TSOP ISC61
#define SIG_TSOP   SIG_INTERRUPT6
#define TSOP_READ() (!(PINE & 0x40))
#else
#define INTx_TSOP  INT4
#define ISCx0_TSOP ISC40
#define ISCx1_TSOP ISC41
#define SIG_TSOP   SIG_INTERRUPT4
#define TSOP_READ() (!(PINE & 0x10))
#endif

//#define MODUL_455KHZ
#define MODUL_38KHZ

#if (defined MODUL_455KHZ)
#define TSOP_FREQ_MHZ 0.455
#define N_PERIODS   10.
#else
#define TSOP_FREQ_MHZ 0.038
#define N_PERIODS   15.
#endif

#define TSOP_PERIOD_US (1./TSOP_FREQ_MHZ)

#define TSOP_TIME_SHORT_US (1.5 * N_PERIODS * TSOP_PERIOD_US)
#define TSOP_TIME_LONG_US  (2.5 * N_PERIODS * TSOP_PERIOD_US)

#define TSOP_TIME_SHORT ((uint16_t)(TSOP_TIME_SHORT_US*2))
#define TSOP_TIME_LONG  ((uint16_t)(TSOP_TIME_LONG_US*2))

#define FRAME_LEN 16
#define FRAME_MASK ((1UL << FRAME_LEN) - 1)

/* frame */
static uint16_t start_angle_time;
static uint16_t frame;
static uint16_t mask;
static uint8_t len;
static uint8_t val;

struct detected_frame {
	uint16_t frame;
	uint16_t time;
};

#define FRAME_RING_ORDER 4
#define FRAME_RING_SIZE  (1<<FRAME_RING_ORDER)
#define FRAME_RING_MASK  (FRAME_RING_SIZE-1)
static uint8_t frame_ring_head = 0;
static uint8_t frame_ring_tail = 0;
static struct detected_frame frame_ring[FRAME_RING_SIZE];

/********************** CS */

/* 8ms, easier if it's a pow of 2 */
#define CS_PERIOD_US (8192)
#define CS_PERIOD ((uint16_t)(CS_PERIOD_US*2))
#define CPT_ICR_MAX (uint8_t)((1000000UL/(uint32_t)CS_PERIOD_US)) /* too slow = 1 tr/s */
#define CPT_ICR_MIN (uint8_t)((10000UL/(uint32_t)CS_PERIOD_US))   /* too fast = 100 tr/s */

/* in tr / 1000s */
#define CS_CONSIGN (25 * 1000L)

/* pwm for laser:
 *  - clear on timer compare (CTC)
 *  - Toggle OC0 on compare match
 *  - prescaler = 1 */
#define LASER_ON() do { TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS00); } while (0)
#define LASER_OFF() do { TCCR0 = 0; } while (0)

struct beacon_tsop beacon_tsop;

void debug_serial(void)
{
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
}
		  
void debug_tsop(void)
{
#if 0
	while (1) {
		if (TSOP_READ())
			LED1_OFF();
		else {
			LED1_ON();
			wait_ms(500);
		}
	}
#endif
}

#if 0
/* val is 16 bits, including 4 bits-cksum in MSB, return 0xFFFF is
 * cksum is wrong, or the 12 bits value on success. */
static uint16_t verify_cksum(uint16_t val)
{
	uint16_t x, cksum;

	x = (val & 0xfff);
	/* add the four 4-bits blocks of val together */
	cksum = val & 0xf;
	val = val >> 4;
	cksum += val & 0xf;
	cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4);
	val = val >> 4;
	cksum += val & 0xf;
	cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4);
	val = val >> 4;
	cksum += val & 0xf;
	cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4);
	if (cksum == 0xf)
		return x;
	return 0xffff; /* wrong cksum */
}
#endif

/* decode frame */
SIGNAL(SIG_TSOP) {
	static uint8_t led_cpt = 0;

	/* tsop status */
	static uint8_t prev_tsop = 0;
	uint8_t cur_tsop;

	/* time */
	static uint16_t prev_time;
	uint16_t ref_time;
	uint16_t cur_time;
	uint16_t diff_time;

	ref_time = ICR3;
	cur_time = TCNT3;
	cur_tsop = TSOP_READ();
	diff_time = cur_time - prev_time;

	if (cur_tsop)
		LED2_ON();
	else
		LED2_OFF();

	/* first rising edge */
	if (len == 0 && cur_tsop && diff_time > TSOP_TIME_LONG) {
		len = 1;
		val = 1;
		frame = 0;
		start_angle_time = cur_time - ref_time;
		mask = 1;
	}
	/* any short edge */
	else if (len != 0 && diff_time < TSOP_TIME_SHORT) {
		if (len & 1) {
			if (val)
				frame |= mask;
			mask <<= 1;
		}
		len ++;
	}
	/* any long edge */
	else if (len != 0 && diff_time < TSOP_TIME_LONG) {
		val = !val;
		if (val)
			frame |= mask;
		mask <<= 1;
		len += 2;
	}
	/* error case, reset */
	else {
		len = 0;
	}

	/* end of frame */
	if (len == FRAME_LEN*2) {
		uint8_t tail_next = (frame_ring_tail+1) & FRAME_RING_MASK;
		if (tail_next != frame_ring_head) {
			frame_ring[frame_ring_tail].frame = (frame & FRAME_MASK);
			frame_ring[frame_ring_tail].time = start_angle_time;
			frame_ring_tail = tail_next;
		}
		if ((led_cpt & 0x7) == 0)
			LED3_TOGGLE();
		led_cpt ++;
	}

	prev_time = cur_time;
	prev_tsop = cur_tsop;
}

/* absolute value */
static inline int32_t AbS(int32_t x)
{
	if (x > 0)
		return x;
	else
		return -x;
}

/* Get the speed of motor (tr / 1000s)
 * - icr_cpt is the number of CS period between 2 ICR updates
 * - icr_diff is the difference of ICR values between the ICR updates
 *   (modulo 65536 obviously) */
static inline int32_t get_speed(uint8_t icr_cpt, uint16_t icr_diff)
{
	int32_t best_diff = 65536L;
	int8_t best_cpt = -2;
	int32_t diff;
	int8_t i;

	/* too slow (less than 1 tr/s) */
	if (icr_cpt > CPT_ICR_MAX)
		return 1000L;

	/* too fast (more than 100 tr/s) */
	if (icr_cpt < CPT_ICR_MIN)
		return 100000L;

	/* try to get the real time knowning icr_cpt and icr_diff */
	for (i=-1; i<2; i++) {
		diff = ((icr_cpt+i)&3) * 16384L;
		diff += (icr_diff & 0x3fff);
		diff -= icr_diff;
		if (diff > 32768L)
			diff -= 65536L;
		if (diff < -32768)
			diff += 65536L;
        
		if (AbS(diff) < AbS(best_diff)) {
			best_diff = diff;
			best_cpt = icr_cpt + i;
		}
	}

	/* real time difference in 1/2 us */
	diff = (best_cpt * 16384L) + (icr_diff & 0x3fff);
	return 2000000000L/diff;
}

int main(void)
{
	uint16_t prev_cs = 0;
	uint16_t prev_icr = 0;
	uint16_t icr = 0;
	uint16_t diff_icr = 0;
	uint8_t cpt_icr = 0;
	uint8_t cpt = 0;
	int32_t speed, out, err;
	uint16_t tcnt3;
	uint8_t x = 0;

	/* LEDS */
	LED1_DDR |= _BV(LED1_BIT);
	LED2_DDR |= _BV(LED2_BIT);
	LED3_DDR |= _BV(LED3_BIT);
	DDRB |= 0x10; /* OC0 (laser pwm) */

	/* PID init */
	pid_init(&beacon_tsop.pid);
	pid_set_gains(&beacon_tsop.pid, 500, 0, 0);
	pid_set_maximums(&beacon_tsop.pid, 0, 20000, 4095);
	pid_set_out_shift(&beacon_tsop.pid, 10);
	pid_set_derivate_filter(&beacon_tsop.pid, 4);

	uart_init();
 	fdevopen(uart0_dev_send, uart0_dev_recv);

	rdline_init(&beacon_tsop.rdl, write_char, valid_buffer, complete_buffer);
	snprintf(beacon_tsop.prompt, sizeof(beacon_tsop.prompt), "beacon > ");	
	rdline_newline(&beacon_tsop.rdl, beacon_tsop.prompt);

	debug_tsop();
	debug_serial();

	/* configure external interrupt for TSOP */
	EICRx_TSOP |= _BV(ISCx0_TSOP);
	EIMSK |= _BV(INTx_TSOP);

	/* pwm for motor */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
	PWM_NG_INIT16(&beacon_tsop.pwm_motor, 1, A, 10, 0, NULL, 0);

	/* pwm for laser:
	 *  - clear on timer compare (CTC)
	 *  - Toggle OC0 on compare match
	 *  - prescaler = 1 */
	TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS00);
	OCR0 = 80; /* f = 100 khz at 16 Mhz */

	/* configure timer 3: CLK/8
	 * it is used as a reference time
	 * enable noise canceller for ICP3 */
	TCCR3B = _BV(ICNC3) | _BV(CS11);

	sei();

	/* Control system will be done in main loop */
	while (1) {

		/* process pending bytes on uart */
		cmdline_process();

		/* monitor the value of ICR (which is modified
		 * automatically on TT rising edge). If the value
		 * changed, process the time difference. */
		if (ETIFR & _BV(ICF3)) {
			cli();
			icr = ICR3;
			sei();
			ETIFR = _BV(ICF3);

			//LED2_TOGGLE();
			diff_icr = (icr - prev_icr);
			cpt_icr = cpt;
			prev_icr = icr;
			cpt = 0;
			speed = get_speed(cpt_icr, diff_icr);
		}

		/* read time reference */
		cli();
		tcnt3 = TCNT3;
		sei();

		/* wait cs period */
		if (tcnt3 - prev_cs < CS_PERIOD)
			continue;

		/* CS LED */
		if (x & 0x80)
			LED1_ON();
		else
			LED1_OFF();
		x++;

		/* process CS... maybe we don't need to use
		 * control_system_manager, just PID is enough */
		if (cpt == CPT_ICR_MAX)
			speed = 0;
		else
			speed = get_speed(cpt_icr, diff_icr);
		
		/* enabled laser when rotation speed if at least 5tr/s */
		if (speed > 5000)
			LASER_ON();
		else
			LASER_OFF();

		err = CS_CONSIGN - speed;
		out = pid_do_filter(&beacon_tsop.pid, err);
		if (x == 0 && beacon_tsop.debug_speed)
			printf("%ld %ld\n", speed, out);
		if (out < 0)
			out = 0;
		/* XXX */
		if (out > 2000)
			out = 2000;

		pwm_ng_set(&beacon_tsop.pwm_motor, out);

		prev_cs = tcnt3;

		/* count the number of CS period between 2 ICR
		 * captures */
		if (cpt < CPT_ICR_MAX)
			cpt ++;

		/* after CS, check if we have a new frame in ring */
		if (frame_ring_head != frame_ring_tail) {
			uint8_t head_next;
			uint32_t frame;
			head_next = (frame_ring_head+1) & FRAME_RING_MASK;
			frame = frame_ring[frame_ring_head].frame;

			/* display if needed */
			if (beacon_tsop.debug_frame) {
				uint8_t beacon_id;
				uint16_t data;
				
				beacon_id = (frame >> BEACON_ID_SHIFT) & BEACON_ID_MASK;
				data = (frame >> FRAME_DATA_SHIFT) & FRAME_DATA_MASK;
				printf("ID=%d data=%d time=%d\r\n",
				       beacon_id, data,
				       frame_ring[frame_ring_head].time);
			}
			frame_ring_head = head_next;
		}

	}

	return 0;
}
