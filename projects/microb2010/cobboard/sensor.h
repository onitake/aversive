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
 *  Revision : $Id: sensor.h,v 1.4 2009-04-24 19:30:42 zer0 Exp $
 *
 */

/* synchronize with sensor.c */
#define ADC_CSENSE1   0
#define ADC_CSENSE2   1
#define ADC_CSENSE3   2
#define ADC_CSENSE4   3
#define ADC_MAX       4

/* synchronize with sensor.c */
#define S_COB_INSIDE_L  0
#define S_CAP2          1
#define S_COB_INSIDE_R  2
#define S_CAP4          3
#define S_LCOB          4
#define S_CAP6          5
#define S_CAP7          6
#define S_RCOB          7
#define S_RESERVED1     8
#define S_RESERVED2     9
#define S_RESERVED3    10
#define S_RESERVED4    11
#define S_RESERVED5    12
#define S_RESERVED6    13
#define S_RESERVED7    14
#define S_RESERVED8    15
#define SENSOR_MAX     16

void sensor_init(void);

/* get filtered values for adc */
int16_t sensor_get_adc(uint8_t i);

/* get filtered values of boolean sensors */
uint16_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);

uint8_t cob_falling_edge(uint8_t side);
