/*  
 *  Copyright Droids Corporation (2010)
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

#include <uart.h>
#include <pid.h>
#include <pwm_ng.h>
#include <parse.h>
#include <rdline.h>
#include <vect_base.h>
#include <lines.h>
#include <circles.h>

#include "cmdline.h"
#include "uart_proto.h"
#include "trigo.h"
#include "main.h"

#define BOARD2010
//#define BOARD2006

#ifdef BOARD2010
#include "board2010.h"
#else
#include "board2006.h"
#endif

/******************* TSOP */

struct detected_frame {
	uint16_t frame;
	uint16_t ref_time;
	uint16_t time;
	uint16_t tick;
};

/* frame */
struct frame_status {
	uint8_t led_cpt;
	uint16_t ref_time;
	uint16_t start_time;
	uint16_t frame;
	uint16_t mask;
	uint16_t prev_time;
	uint16_t time_long;
	uint16_t time_short;
	uint8_t prev_tsop;
	uint8_t len;
	uint8_t frame_len;
	uint8_t val;
#define FRAME_RING_ORDER 4
#define FRAME_RING_SIZE  (1<<FRAME_RING_ORDER)
#define FRAME_RING_MASK  (FRAME_RING_SIZE-1)
	uint8_t head;
	uint8_t tail;
	struct detected_frame ring[FRAME_RING_SIZE];
};

static struct frame_status static_beacon;
static struct frame_status opp_beacon;
static uint16_t tick = 0;

#define MIN_DIST 200.
#define MAX_DIST 3500.

/* in ticks (=CS_PERIOD), age before the entry is removed from ring */
#define MAX_CAP_AGE 30

/********************** CS */

/* 8ms, easier if it's a pow of 2 */
#define CS_PERIOD_US (8192)
#define CS_PERIOD ((uint16_t)(CS_PERIOD_US/4))
#define CPT_ICR_MAX (uint8_t)((1000000UL/(uint32_t)CS_PERIOD_US)) /* too slow = 1 tr/s */
#define CPT_ICR_MIN (uint8_t)((10000UL/(uint32_t)CS_PERIOD_US))   /* too fast = 100 tr/s */

/* in tr / 1000s */
#define CS_CONSIGN (15 * 1000L)

/* 5% tolerance to validate captures, period is in  */
#define TIM3_UNIT 250000000L
#define MOTOR_PERIOD_MIN ((uint32_t)((250000000L/CS_CONSIGN) * 0.95))
#define MOTOR_PERIOD_MAX ((uint32_t)((250000000L/CS_CONSIGN) * 1.05))

/* pwm for laser:
 *  - clear on timer compare (CTC)
 *  - Toggle OC0 on compare match
 *  - prescaler = 1 */
#define LASER_ON() do { TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS00); } while (0)
#define LASER_OFF() do { TCCR0 = 0; } while (0)

struct beacon_tsop beacon_tsop;
uint32_t cs_consign = CS_CONSIGN;

static uint32_t current_motor_period;

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

static inline void decode_frame(struct frame_status *status,
				uint16_t ref_time, uint16_t cur_time, uint8_t cur_tsop)
{
	uint16_t diff_time = cur_time - status->prev_time;

	/* first rising edge */
	if (status->len == 0 && cur_tsop && diff_time > status->time_long) {
		LED6_ON();
		status->len = 1;
		status->val = 1;
		status->frame = 0;
		status->start_time = cur_time;
		status->ref_time = ref_time;
		status->mask = 1;
	}
	/* any short pulse */
	else if (status->len != 0 && diff_time < status->time_short) {
		if (status->len & 1) {
			if (status->val)
				status->frame |= status->mask;
			status->mask <<= 1;
		}
		status->len ++;
	}
	/* any long pulse */
	else if (status->len != 0 && diff_time < status->time_long) {
		status->val = !status->val;
		if (status->val)
			status->frame |= status->mask;
		status->mask <<= 1;
		status->len += 2;
	}
	/* error case, reset */
	else {
		status->len = 0;
	}

	/* end of frame */
	if (status->len == status->frame_len*2) {
		uint8_t tail_next = (status->tail+1) & FRAME_RING_MASK;
		uint16_t frame_mask;

		frame_mask = (1 << status->frame_len) - 1;

		if (tail_next != status->head) {
			LED5_ON();
			status->ring[status->tail].frame = (status->frame & frame_mask);
			status->ring[status->tail].ref_time = status->ref_time;
			status->ring[status->tail].time = status->start_time;
			status->ring[status->tail].tick = tick;
			status->tail = tail_next;
/* 			if ((status->led_cpt & 0x7) == 0) */
/* 				LED3_TOGGLE(); */
			status->led_cpt ++;
		}
		status->len = 0;
	}

	status->prev_time = cur_time;
	status->prev_tsop = cur_tsop;
	LED3_OFF();
	LED4_OFF();
	LED5_OFF();
	LED6_OFF();
}

/* decode frame */
SIGNAL(SIG_TSOP_STA) {
	static uint8_t running = 0;

	/* tsop status */
	uint8_t cur_tsop;
	uint16_t ref_time;
	uint16_t cur_time;

	ref_time = ICR3;
	cur_time = TCNT3;
	cur_tsop = TSOP_STA_READ();

	/* avoid interruption stacking */
	if (running)
		return;
	running = 1;
	sei();

/* 	if (cur_tsop) */
/* 		LED5_ON(); */
/* 	else */
/* 		LED5_OFF(); */

	decode_frame(&static_beacon, ref_time, cur_time, cur_tsop);

	running = 0;
}

/* decode frame */
SIGNAL(SIG_TSOP_OPP) {
	static uint8_t running = 0;

	/* tsop status */
	uint8_t cur_tsop;
	uint16_t ref_time;
	uint16_t cur_time;

	ref_time = ICR3;
	cur_time = TCNT3;
	cur_tsop = TSOP_OPP_READ();

	/* avoid interruption stacking */
	if (running)
		return;
	running = 1;
	sei();

/* 	if (cur_tsop) */
/* 		LED6_ON(); */
/* 	else */
/* 		LED6_OFF(); */

	//decode_frame(&opp_beacon, ref_time, cur_time, cur_tsop);

	running = 0;
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
#if 0
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

	/* real time difference in timer unit (resolution 4us) */
	diff = (best_cpt * 16384L) + (icr_diff & 0x3fff);
	current_motor_period = diff; /* save it in global var */
#endif

	/* too slow (less than 1 tr/s) */
	if (icr_cpt >= CPT_ICR_MAX)
		return 1000L;

	/* too fast (more than 100 tr/s) */
	if (icr_cpt <= CPT_ICR_MIN)
		return 100000L;

	/* XXX test */
	if (icr_cpt > 25)
		return icr_cpt * 8192UL;

	return TIM3_UNIT/icr_diff;
}

static int8_t check_sta_frame(uint16_t frame, uint16_t time)
{
	int8_t beacon_id;

	/* ignore bad cksum */
	if (verify_cksum(frame) == 0xFFFF)
		goto fail;

	beacon_id = (frame >> TSOP_STA_BEACON_ID_SHIFT) & TSOP_STA_BEACON_ID_MASK;

	if (beacon_id != TSOP_STA_BEACON_ID0 &&
	    beacon_id != TSOP_STA_BEACON_ID1)
		goto fail;

	/* if motor speed is not good, skip values  */
	if (current_motor_period < MOTOR_PERIOD_MIN)
		goto fail;
	if (current_motor_period > MOTOR_PERIOD_MAX)
		goto fail;

	return beacon_id;

 fail:
	/* display if needed */
	if (beacon_tsop.debug_frame) {
		printf("STA ID=%d frame=%x time=%d\r\n",
		       beacon_id, frame, time);
	}
	return -1;
}


/* process the received frame ring */
static void process_sta_ring(struct frame_status *status)
{
	uint8_t head, head_next;
	uint16_t frame, frametick;
	uint8_t found = 0;
	int8_t beacon_id;

	/* beacon 0 */
	uint16_t data0, time0, ref_time0;
	double angle0;
	double dist0;

	/* beacon 1 */
	uint16_t data1, time1, ref_time1;
	double angle1;
	double dist1;

	point_t pos;
	double a;

	/* remove too old captures from the ring */
	while (status->head != status->tail) {
		head_next = (status->head+1) & FRAME_RING_MASK;
		frametick = status->ring[status->head].tick;
		if ((uint16_t)(tick - frametick) < MAX_CAP_AGE)
			break;
		status->head = head_next;
	}

	head = status->head;
	/* after CS, check if we have a new frame in ring */
	while (head != status->tail) {
		head_next = (head+1) & FRAME_RING_MASK;
		frame = status->ring[head].frame;

		beacon_id = check_sta_frame(frame, status->ring[head].time);
		if (beacon_id < 0) {
			head = head_next;
			continue;
		}

		if (beacon_id == TSOP_STA_BEACON_ID0) {
			found |= 0x1;
			data0 = (frame >> TSOP_STA_FRAME_DATA_SHIFT) & TSOP_STA_FRAME_DATA_MASK;
			time0 = status->ring[head].time;
			ref_time0 = status->ring[head].ref_time;
		}
		else if (beacon_id == TSOP_STA_BEACON_ID1) {
			found |= 0x2;
			data1 = (frame >> TSOP_STA_FRAME_DATA_SHIFT) & TSOP_STA_FRAME_DATA_MASK;
			time1 = status->ring[head].time;
			ref_time1 = status->ring[head].ref_time;
		}

		head = head_next;
	}

	/* if we didn't found beacon 0 and 1, return */
	if (found != 0x3)
		return;

	/* update ring head */
	status->head = head;

	/* beacon 0 */
	dist0 = data0;
	dist0 /= 512.;
	dist0 *= (MAX_DIST-MIN_DIST);
	dist0 += MIN_DIST;

	time0 = time0 - ref_time0;
	angle0 = (double)time0 / (double)current_motor_period;
	if (angle0 > 1.)
		angle0 -= 1.;
	if (angle0 > 1.)
		return; /* fail */
	angle0 *= (2 * M_PI);
	if (angle0 > M_PI)
		angle0 -= M_PI;

	/* beacon 1 */
	dist1 = data1;
	dist1 /= 512.;
	dist1 *= (MAX_DIST-MIN_DIST);
	dist1 += MIN_DIST;

	time1 = time1 - ref_time1;
	angle1 = (double)time1 / (double)current_motor_period;
	if (angle1 > 1.)
		angle1 -= 1.;
	if (angle1 > 1.)
		return; /* fail */
	angle1 *= (2 * M_PI);
	if (angle0 > M_PI)
		angle0 -= M_PI;

	/* display if needed */
	if (beacon_tsop.debug_frame) {
		printf("STA ID=%d dist0=%2.2f angle0=%2.2f dist1=%2.2f angle1=%2.2f\r\n",
		       beacon_id, dist0, angle0 * 180. / M_PI, dist1, angle1 * 180. / M_PI);
	}

	if (ad_to_posxya(&pos, &a, 0, &beacon0, &beacon1, angle0, dist0,
			 angle1, dist1) < 0)
		return;

	xmit_static((uint16_t)pos.x, (uint16_t)pos.y, (uint16_t)a);
}

static int8_t check_opp_frame(uint16_t frame, uint16_t time)
{
	int8_t beacon_id = -1;

	/* ignore bad cksum */
	if (verify_cksum(frame) == 0xFFFF)
		goto fail;

	beacon_id = (frame >> TSOP_OPP_BEACON_ID_SHIFT) & TSOP_OPP_BEACON_ID_MASK;
	if (beacon_id != TSOP_OPP_BEACON_ID)
		goto fail;

	/* if motor speed is not good, skip values  */
	if (current_motor_period < MOTOR_PERIOD_MIN)
		goto fail;
	if (current_motor_period > MOTOR_PERIOD_MAX)
		goto fail;

	return beacon_id;
 fail:
	/* display if needed */
	if (beacon_tsop.debug_frame) {
		printf("OPP ID=%d frame=%x time=%d\r\n",
		       beacon_id, frame, time);
	}
	return -1;
}

/* process the received frame ring */
static void process_opp_ring(struct frame_status *status)
{
	uint8_t head_next;
	uint16_t frame;
	uint8_t found = 0;
	uint16_t data, time, ref_time;
	double angle;
	double dist;
				
	/* after CS, check if we have a new frame in ring */
	while (status->head != status->tail) {
		head_next = (status->head+1) & FRAME_RING_MASK;
		frame = status->ring[status->head].frame;

		if (check_opp_frame(frame, status->ring[status->head].time) < 0) {
			status->head = head_next;
			continue;
		}

		found = 1;
		data = (frame >> TSOP_OPP_FRAME_DATA_SHIFT) & TSOP_OPP_FRAME_DATA_MASK;
		time = status->ring[status->head].time;
		ref_time = status->ring[status->head].ref_time;

		status->head = head_next;
	}

	if (found == 0)
		return;

	dist = data;
	dist /= 512.;
	dist *= (MAX_DIST-MIN_DIST);
	dist += MIN_DIST;

	time = time - ref_time;
	angle = (double)time / (double)current_motor_period;
	if (angle > 1.)
		angle -= 1.;
	if (angle > 1.)
		return; /* fail */
	angle *= 3600; /* angle in 1/10 deg */

	/* display if needed */
	if (beacon_tsop.debug_frame) {
		printf("OPP dist=%2.2f angle=%2.2f\r\n", dist, angle/10);
	}
	xmit_opp((uint16_t)dist, (uint16_t)angle);
}

int main(void)
{
	uint16_t prev_cs = 0;
	uint16_t prev_icr = 0;
	uint16_t icr = 0;
	uint16_t diff_icr = 0;
	uint8_t cpt_icr = 0;
	uint8_t cpt = 0;
	int32_t speed = 0, out, err;
	uint16_t tcnt3;
	uint8_t x = 0; /* debug display counter */

	opp_beacon.frame_len = TSOP_OPP_FRAME_LEN;
	opp_beacon.time_long = TSOP_OPP_TIME_LONG;
	opp_beacon.time_short = TSOP_OPP_TIME_SHORT;

	static_beacon.frame_len = TSOP_STA_FRAME_LEN;
	static_beacon.time_long = TSOP_STA_TIME_LONG;
	static_beacon.time_short = TSOP_STA_TIME_SHORT;

	/* LEDS */
	LED_DDR_INIT();
	DDRB |= 0x10; /* OC0 (laser pwm) */

	/* PID init */
	pid_init(&beacon_tsop.pid);
	pid_set_gains(&beacon_tsop.pid, 700, 10, 0);
	pid_set_maximums(&beacon_tsop.pid, 0, 200000, 4095);
	pid_set_out_shift(&beacon_tsop.pid, 10);
	pid_set_derivate_filter(&beacon_tsop.pid, 4);

	uart_init();
#if CMDLINE_UART == 0
 	fdevopen(uart0_dev_send, uart0_dev_recv);
#elif CMDLINE_UART == 1
 	fdevopen(uart1_dev_send, uart1_dev_recv);
#endif

	rdline_init(&beacon_tsop.rdl, write_char, valid_buffer, complete_buffer);
	snprintf(beacon_tsop.prompt, sizeof(beacon_tsop.prompt), "beacon > ");	
	rdline_newline(&beacon_tsop.rdl, beacon_tsop.prompt);

	debug_tsop();
	debug_serial();

	/* configure external interrupt for TSOP */
	EICRx_TSOP |= _BV(ISCx0_TSOP_STA) | _BV(ISCx0_TSOP_OPP);
	EIMSK |= _BV(INTx_TSOP_STA) | _BV(INTx_TSOP_OPP);

	/* pwm for motor */
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
				 TIMER1_PRESCALER_DIV_1);
#ifdef BOARD2010
	PWM_NG_INIT16(&beacon_tsop.pwm_motor, 1, C, 10, 0, NULL, 0);
#else
	PWM_NG_INIT16(&beacon_tsop.pwm_motor, 1, A, 10, 0, NULL, 0);
#endif

	/* pwm for laser:
	 *  - clear on timer compare (CTC)
	 *  - Toggle OC0 on compare match
	 *  - prescaler = 1 */
	TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS00);
	OCR0 = 18; /* f ~= 420 khz at 16 Mhz */

	/* configure timer 3: CLK/64
	 * it is used as a reference time
	 * enable noise canceller for ICP3 */
	TCCR3B = _BV(CS11) | _BV(CS10);

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

			LED2_TOGGLE();
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
		
		/* enabled laser when rotation speed if at least 5tr/s */
		if (1 || speed > 5000) /* XXX */
			LASER_ON();
		else
			LASER_OFF();

		err = cs_consign - speed;
		out = pid_do_filter(&beacon_tsop.pid, err);
		if (out < 0)
			out = 0;
		if (out > 3000)
			out = 3000;

		if (x == 0 && beacon_tsop.debug_speed)
			printf("%ld %ld %u %u / %u\r\n",
			       speed, out, diff_icr, cpt_icr, cpt);

		pwm_ng_set(&beacon_tsop.pwm_motor, out);

		prev_cs = tcnt3;

		/* count the number of CS period between 2 ICR
		 * captures */
		if (cpt < CPT_ICR_MAX)
			cpt ++;

		process_sta_ring(&static_beacon);
		process_opp_ring(&opp_beacon);
		cli();
		tick ++; /* global imprecise time reference */
		sei();
	}

	return 0;
}
