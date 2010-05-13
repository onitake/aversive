/*
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: beacon.h,v 1.2 2009-05-27 20:04:07 zer0 Exp $
 *
 */

struct beacon {
	int32_t beacon_speed;

	int32_t opponent_angle;
	int32_t opponent_dist;
	int32_t prev_opponent_angle;
	int32_t prev_opponent_dist;
	int32_t robot_x;
	int32_t robot_y;
	int32_t robot_angle;
	int32_t opponent_x;
	int32_t opponent_y;
};

/* real encoder value: 3531.75 so, multiple by 4 to have round
 * value */
#define BEACON_STEP_TOUR (14127L)
#define BEACON_OFFSET_CALIBRE 40

extern struct beacon beacon;

void beacon_dump(void);
void beacon_init(void);
void beacon_calibre_pos(void);
void beacon_start(void);
void beacon_stop(void);
void beacon_calc(void *dummy);
void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y);

void beacon_reset_pos(void);
void beacon_set_consign(int32_t val);

int32_t encoders_spi_get_beacon_speed(void *dummy);
int32_t encoders_spi_update_beacon_speed(void *number);
int32_t encoders_spi_get_value_beacon(void *number);

