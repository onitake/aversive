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
 *  Revision : $Id: sensor.c,v 1.6 2009-11-08 17:25:00 zer0 Exp $
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
static void do_adc(__attribute__((unused)) void *dummy) 
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
 * CAP 1,5,6,7,8
 */
static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_CAP1] =      { 10, 0, 3, 7, 0, 0 }, /* 0 */
	[S_FRONT] =     { 5, 0, 4, 1, 0, 0 },  /* 1 */
	[S_CAP3] =      { 10, 0, 3, 7, 0, 0 }, /* 2 */
	[S_CAP4] =      { 1, 0, 0, 1, 0, 0 }, /* 3 */
	[S_COL_LEFT] =  { 5, 0, 4, 1, 0, 1 }, /* 4 */
	[S_LEFT] =      { 5, 0, 4, 1, 0, 1 }, /* 5 */
	[S_RIGHT] =     { 5, 0, 4, 1, 0, 1 }, /* 6 */
	[S_COL_RIGHT] = { 5, 0, 4, 1, 0, 1 }, /* 7 */
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
static void do_boolean_sensors(__attribute__((unused)) void *dummy)
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



/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(__attribute__((unused)) void *dummy)
{
	do_adc(NULL);
	do_boolean_sensors(NULL);
}

void sensor_init(void)
{
	adc_init();
	adc_register_event(adc_event);
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, 
						ADC_PRIO);
}

