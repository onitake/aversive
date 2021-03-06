/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: sensor.c,v 1.3 2009-05-27 20:04:07 zer0 Exp $
 *
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>

#include <adc.h>
#include <scheduler.h>
#include <ax12.h>
#include <pwm_ng.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "sensor.h"
#include "../common/i2c_commands.h"

/************ ADC */

struct adc_infos {
	uint16_t config;
	int16_t value;
	int16_t prev_val;
        int16_t (*filter)(struct adc_infos *, int16_t);
};

/* reach 90% of the value in 4 samples */
int16_t rii_light(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + (int32_t)adc->prev_val / 2;
	return adc->prev_val / 2;
}

/* reach 90% of the value in 8 samples */
int16_t rii_medium(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 3) / 4;
	return adc->prev_val / 4;
}

/* reach 90% of the value in 16 samples */
int16_t rii_strong(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 7) / 8;
	return adc->prev_val / 8;
}


#define ADC_CONF(x) ( ADC_REF_AVCC | ADC_MODE_INT | MUX_ADC##x )

/* define which ADC to poll, see in sensor.h */
static struct adc_infos adc_infos[ADC_MAX] = {

	[ADC_CSENSE1] = { .config = ADC_CONF(0), .filter = rii_medium },
	[ADC_CSENSE2] = { .config = ADC_CONF(1), .filter = rii_medium },
	[ADC_CSENSE3] = { .config = ADC_CONF(2), .filter = rii_medium },
	[ADC_CSENSE4] = { .config = ADC_CONF(3), .filter = rii_medium },

	/* add adc on "cap" pins if needed */
/* 	[ADC_CAP1] = { .config = ADC_CONF(10) }, */
/* 	[ADC_CAP2] = { .config = ADC_CONF(11) }, */
/* 	[ADC_CAP3] = { .config = ADC_CONF(12) }, */
/* 	[ADC_CAP4] = { .config = ADC_CONF(13) }, */
};

static void adc_event(int16_t result);

/* called every 10 ms, see init below */
static void do_adc(void *dummy)
{
	/* launch first conversion */
	adc_launch(adc_infos[0].config);
}

static void adc_event(int16_t result)
{
	static uint8_t i = 0;

	/* filter value if needed */
	if (adc_infos[i].filter)
		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i],
							 result);
	else
		adc_infos[i].value = result;

	i ++;
	if (i >= ADC_MAX)
		i = 0;
	else
		adc_launch(adc_infos[i].config);
}

int16_t sensor_get_adc(uint8_t i)
{
	int16_t tmp;
	uint8_t flags;

	IRQ_LOCK(flags);
	tmp = adc_infos[i].value;
	IRQ_UNLOCK(flags);
	return tmp;
}

/************ boolean sensors */


struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

/* pullup mapping:
 * CAP 5,6,7,8
 * voltage div mapping:
 * CAP 1
 */
static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_HIGH_BARRIER] = { 20, 0, 1, 19, 0, 1 }, /* 0 */
	[S_LOW_BARRIER] = { 50, 0, 1, 1, 0, 0 }, /* 1 */
	[S_CAP3] = { 1, 0, 0, 1, 0, 0 }, /* 2 */
	[S_CAP4] = { 1, 0, 0, 1, 0, 0 }, /* 3 */
	[S_R_IR] = { 1, 0, 0, 1, 0, 0 }, /* 4 */
	[S_R_US] = { 1, 0, 0, 1, 0, 1 }, /* 5 */
	[S_L_US] = { 1, 0, 0, 1, 0, 1 }, /* 6 */
	[S_L_IR] = { 1, 0, 0, 1, 0, 0 }, /* 7 */
	[S_RESERVED1] = { 10, 0, 3, 7, 0, 0 }, /* 8 */
	[S_RESERVED2] = { 10, 0, 3, 7, 0, 0 }, /* 9 */
	[S_RESERVED3] = { 1, 0, 0, 1, 0, 0 }, /* 10 */
	[S_RESERVED4] = { 1, 0, 0, 1, 0, 0 }, /* 11 */
	[S_RESERVED5] = { 1, 0, 0, 1, 0, 0 }, /* 12 */
	[S_RESERVED6] = { 1, 0, 0, 1, 0, 0 }, /* 13 */
	[S_RESERVED7] = { 1, 0, 0, 1, 0, 0 }, /* 14 */
	[S_RESERVED8] = { 1, 0, 0, 1, 0, 0 }, /* 15 */
};

/* value of filtered sensors */
static uint16_t sensor_filtered = 0;

/* sensor mapping :
 * 0-3:  PORTK 2->5 (cap1 -> cap4) (adc10 -> adc13)
 * 4-5:  PORTL 0->1 (cap5 -> cap6)
 * 6-7:  PORTE 3->4 (cap7 -> cap8)
 * 8-15: reserved
 */

uint16_t sensor_get_all(void)
{
	uint16_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
	uint16_t tmp = sensor_get_all();
	return !!(tmp & _BV(i));
}

/* get the physical value of pins */
static uint16_t sensor_read(void)
{
	uint16_t tmp = 0;
	tmp |= (uint16_t)((PINK & (_BV(2)|_BV(3)|_BV(4)|_BV(5))) >> 2) << 0;
	tmp |= (uint16_t)((PINL & (_BV(0)|_BV(1))) >> 0) << 4;
	tmp |= (uint16_t)((PINE & (_BV(3)|_BV(4))) >> 3) << 6;
	/* add reserved sensors here */
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint16_t sensor = sensor_read();
	uint16_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		if ((1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}

		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= (1UL << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= (1UL << i);
		}
	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}

static volatile uint8_t lcob_seen = I2C_COB_NONE;
static volatile uint8_t rcob_seen = I2C_COB_NONE;

uint8_t cob_detect_left(void)
{
	uint8_t flags;
	uint8_t ret;
	IRQ_LOCK(flags);
	ret = lcob_seen;
	lcob_seen = I2C_COB_NONE;
	IRQ_UNLOCK(flags);
	return ret;
}

uint8_t cob_detect_right(void)
{
	uint8_t flags;
	uint8_t ret;
	IRQ_LOCK(flags);
	ret = rcob_seen;
	rcob_seen = I2C_COB_NONE;
	IRQ_UNLOCK(flags);
	return ret;
}

#define COB_MIN_DETECT 4
#define COB_MAX_DETECT 50
static void do_cob_detection(void)
{
	uint8_t flags;
	uint16_t tmp = sensor_get_all();
	uint8_t l_us = !!(tmp & _BV(S_L_US));
	uint8_t r_us = !!(tmp & _BV(S_R_US));
	uint8_t l_ir = !!(tmp & _BV(S_L_IR));
	uint8_t r_ir = !!(tmp & _BV(S_R_IR));
	static uint8_t l_us_prev, r_us_prev, l_ir_prev, r_ir_prev;
	static uint8_t l_cpt_on, l_cpt_off;
	static uint8_t r_cpt_on, r_cpt_off;

	/* rising edge on US */
	if (l_us_prev == 0 && l_us == 1) {
		l_cpt_off = 0;
		l_cpt_on = 0;
	}

	/* us is on */
	if (l_us) {
		if (l_ir && l_cpt_on < COB_MAX_DETECT)
			l_cpt_on ++;
		else if (l_cpt_off < COB_MAX_DETECT)
			l_cpt_off ++;
	}

	/* falling edge on US */
	if (l_us_prev == 1 && l_us == 0) {
		/* detection should not be too short or too long */
		if ((l_cpt_off + l_cpt_on) < COB_MAX_DETECT &&
		    (l_cpt_off + l_cpt_on) > COB_MIN_DETECT) {
			IRQ_LOCK(flags);
			if (l_cpt_on > l_cpt_off)
				lcob_seen = I2C_COB_WHITE;
			else
				lcob_seen = I2C_COB_BLACK;
			IRQ_UNLOCK(flags);
			if (l_cpt_on > l_cpt_off)
				DEBUG(E_USER_SENSOR, "left white %d %d",
				      l_cpt_on, l_cpt_off);
			else
				DEBUG(E_USER_SENSOR, "left black %d %d",
				      l_cpt_on, l_cpt_off);
		}
	}


	/* rising edge on US */
	if (r_us_prev == 0 && r_us == 1) {
		r_cpt_off = 0;
		r_cpt_on = 0;
	}

	/* us is on */
	if (r_us) {
		if (r_ir && r_cpt_on < COB_MAX_DETECT)
			r_cpt_on ++;
		else if (r_cpt_off < COB_MAX_DETECT)
			r_cpt_off ++;
	}

	/* falling edge on US */
	if (r_us_prev == 1 && r_us == 0) {

		/* detection should not be too short or too long */
		if ((r_cpt_off + r_cpt_on) < COB_MAX_DETECT &&
		    (r_cpt_off + r_cpt_on) > COB_MIN_DETECT) {
			IRQ_LOCK(flags);
			if (r_cpt_on > r_cpt_off)
				rcob_seen = I2C_COB_WHITE;
			else
				rcob_seen = I2C_COB_BLACK;
			IRQ_UNLOCK(flags);

			if (r_cpt_on > r_cpt_off)
				DEBUG(E_USER_SENSOR, "right white %d %d",
				      r_cpt_on, r_cpt_off);
			else
				DEBUG(E_USER_SENSOR, "right black %d %d",
				      r_cpt_on, r_cpt_off);
		}
	}


	l_us_prev = l_us;
	r_us_prev = r_us;
	l_ir_prev = l_ir;
	r_ir_prev = r_ir;
}

/************ global sensor init */
#define BACKGROUND_ADC  0

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
	if (BACKGROUND_ADC)
		do_adc(NULL);
	do_boolean_sensors(NULL);
	do_cob_detection();
}

void sensor_init(void)
{
	adc_init();
	if (BACKGROUND_ADC)
		adc_register_event(adc_event);
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL,
						10000L / SCHEDULER_UNIT,
						ADC_PRIO);

}

