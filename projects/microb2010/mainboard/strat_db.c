/*
 *  Copyright Droids, Microb Technology (2010)
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
 *  Revision : $Id: strat.c,v 1.6 2009-11-08 17:24:33 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
#include <trajectory_manager_core.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <diagnostic.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_corn.h"
#include "strat_db.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"

/* status of objects on area */
struct strat_db strat_db;

/* given an index, give the i coord */
static const uint8_t corn_coord_i[CORN_NB] = {
	0, 0, 0, 2, 2, 2, 4, 4, 6,
	6, 8, 8, 10, 10, 10, 12, 12, 12,
};

/* given an index, give the j coord */
static const uint8_t corn_coord_j[CORN_NB] = {
	2, 4, 6, 3, 5, 7, 4, 6, 5,
	7, 4, 6, 3, 5, 7, 2, 4, 6,
};

/* table to find the symetric idx */
static const uint8_t corn_sym[] = {
	15, 16, 17, 12, 13, 14, 10, 11,
	8, 9, 6, 7, 3, 4, 5, 0, 1, 2
};

#if 0 /* XXX maybe useless */
/* the 10 possible configurations for corn on the side */
static const uint8_t corn_side_confs[9][2] = {
	{ 1, 4 },
	{ 0, 4 },
	{ 2, 4 },
	{ 2, 3 },
	{ 0, 3 },
	{ 1, 3 },
	{ 1, 6 },
	{ 0, 6 },
	{ 2, 6 },
};

/* the 4 possible configurations for corn on center */
static const uint8_t corn_center_confs[4][2] = {
	{ 5, 8 },
	{ 7, 8 },
	{ 5, 9 },
	{ 7, 8 },
};
#endif

/* in these groups, only one black cob */
static const int8_t corn_group1[] = { 0, 1, 2, -1, };
static const int8_t corn_group2[] = { 3, 4, 6, -1, };
static const int8_t corn_group3[] = { 5, 7, -1, };
static const int8_t corn_group4[] = { 8, 9, -1, };
static const int8_t corn_group5[] = { 11, 14, -1, };
static const int8_t corn_group6[] = { 10, 12, 13, -1, };
static const int8_t corn_group7[] = { 15, 16, 17, -1, };

static const int8_t *corn_groups[] = {
	corn_group1,
	corn_group2,
	corn_group3,
	corn_group4,
	corn_group5,
	corn_group6,
	corn_group7,
	NULL,
};

/* given an index, give the i coord */
static const uint8_t tomato_coord_i[TOMATO_NB] = {
	0, 0, 2, 2, 4, 4, 6, 6,
	8, 8, 10, 10, 12, 12,
};

/* given an index, give the j coord */
static const uint8_t tomato_coord_j[TOMATO_NB] = {
	3, 5, 4, 6, 5, 7, 4, 6, 5, 7, 4, 6, 3, 5,
};

/******** Generic waypoint */

/* return the xy coords of a waypoint given its ij coords. */
int8_t ijcoord_to_xycoord(uint8_t i, uint8_t j, int16_t *x, int16_t *y)
{
	if (i >= WAYPOINTS_NBX && j >= WAYPOINTS_NBY)
		return -1;
	*x = (OFFSET_CORN_X + i*STEP_CORN_X);
	*y = COLOR_Y(OFFSET_CORN_Y + j*STEP_CORN_Y);
	return 0;
}

/******** CORN */

/* return the index of a corn given its i,j coords. */
int8_t ijcoord_to_corn_idx(uint8_t i, uint8_t j)
{
	uint8_t n;
	for (n = 0; n < CORN_NB; n ++) {
		if (i == corn_coord_i[n] &&
		    j == corn_coord_j[n])
			return n;
	}
	return -1;
}

/* return the i,j coords of a corn given its index */
int8_t corn_idx_to_ijcoord(uint8_t idx, uint8_t *i, uint8_t *j)
{
	if (idx >= CORN_NB)
		return -1;
	*i = corn_coord_i[idx];
	*j = corn_coord_j[idx];
	return 0;
}

/* return the index of a corn given its x,y coords. */
int8_t corn_idx_to_xycoord(uint8_t idx, int16_t *x, int16_t *y)
{
	uint8_t i, j;
	if (corn_idx_to_ijcoord(idx, &i, &j) < 0)
		return -1;
	if (ijcoord_to_xycoord(i, j, x, y) < 0)
		return -1;
	return 0;
}

#define CORN_MARGIN 200
/* return the index of the closest corn at these coordinates. If the
 * corn is really too far (~20cm), return NULL. The x and y pointer are
 * updated with the real position of the corn */
struct waypoint_db *xycoord_to_corn_idx(int16_t *x, int16_t *y)
{
	uint8_t idx = -1, n;
	int16_t d, x_corn, y_corn;
	int16_t x_corn_min = 0, y_corn_min = 0;
	int16_t d_min = 0;

	/* XXX does it work when we are blue ? */
	for (n = 0; n < CORN_NB; n ++) {
		corn_idx_to_xycoord(n, &x_corn, &y_corn);
		d = xy_norm(x_corn, y_corn, *x, *y);
		if (d < CORN_MARGIN && (d_min == 0 || d < d_min)) {
			d_min = d;
			idx = n;
			x_corn_min = x_corn;
			y_corn_min = y_corn;
		}
	}
	if (d_min == 0)
		return NULL;

	*x = x_corn_min;
	*y = y_corn_min;

	return strat_db.corn_table[idx];
}

/* return true if 'idx' is in group */
static uint8_t is_in_group(const int8_t *group, uint8_t idx)
{
	const int8_t *pidx;
	for (pidx = group; *pidx != -1; pidx++) {
		if (*pidx == idx) {
			return 1;
		}
	}
	return 0;
}

/* return the number of cob of that color in the group */
static uint8_t count_in_group(const int8_t *group, uint8_t color)
{
	const int8_t *pidx;
	struct waypoint_db *wp;
	uint8_t count = 0;

	for (pidx = &group[0]; *pidx != -1; pidx++) {
		wp = strat_db.corn_table[*pidx];
		if (wp->corn.color == color)
			count ++;
	}
	return count;
}

/* set all unkown cobs to specified color */
static void set_unknown_in_group(const int8_t *group, uint8_t color)
{
	const int8_t *pidx;
	struct waypoint_db *wp;

	for (pidx = &group[0]; *pidx != -1; pidx++) {
		wp = strat_db.corn_table[*pidx];
		if (wp->corn.color == I2C_COB_UNKNOWN)
			wp->corn.color = color;
	}
}

/* depending on which cob is set (and its color), set the color of
 * other cobs */
static void corn_deduct_other(uint8_t idx, uint8_t color)
{
	const int8_t **pgroup;

	for (pgroup = &corn_groups[0]; *pgroup; pgroup++) {
		if (!is_in_group(*pgroup, idx))
			continue;
		if (color == I2C_COB_BLACK) {
			set_unknown_in_group(*pgroup, I2C_COB_WHITE);
		}
		else if (color == I2C_COB_WHITE) {
			if (count_in_group(*pgroup, I2C_COB_UNKNOWN) == 1)
				set_unknown_in_group(*pgroup, I2C_COB_BLACK);
		}
	}
}

/* set color of a corn
 * type is I2C_COB_BLACK, I2C_COB_WHITE, I2C_COB_UNKNOWN
 * it will update the symetric corn if != UNKOWN
 * it will also deduct color of some other cobs */
void corn_set_color(struct waypoint_db *wp, uint8_t color)
{
	uint8_t symidx;

	wp->corn.color = color;
	if (color == I2C_COB_UNKNOWN)
		return;
	if (wp->corn.color != I2C_COB_UNKNOWN)
		return;
	corn_deduct_other(wp->corn.idx, color);
	symidx = corn_get_sym_idx(wp->corn.idx);
	strat_db.corn_table[symidx]->corn.color = color;
	corn_deduct_other(symidx, color);
}


/* return the idx of the symetric corn */
int8_t corn_get_sym_idx(int8_t i)
{
	if (i >= CORN_NB)
		return -1;
	return corn_sym[i];
}

/*********** TOMATO */

/* return the index of a tomato given its i,j coords. */
int8_t ijcoord_to_tomato_idx(uint8_t i, uint8_t j)
{
	uint8_t n;
	for (n = 0; n < TOMATO_NB; n ++) {
		if (i == tomato_coord_i[n] &&
		    j == tomato_coord_j[n])
			return n;
	}
	return -1;
}

/* return the i,j coords of a tomato given its index */
int8_t tomato_idx_to_ijcoord(uint8_t idx, uint8_t *i, uint8_t *j)
{
	if (idx >= TOMATO_NB)
		return -1;
	*i = tomato_coord_i[idx];
	*j = tomato_coord_j[idx];
	return 0;
}

/* return the index of a tomato given its x,y coords. */
int8_t tomato_idx_to_xycoord(uint8_t idx, int16_t *x, int16_t *y)
{
	uint8_t i, j;
	if (tomato_idx_to_ijcoord(idx, &i, &j) < 0)
		return -1;
	if (ijcoord_to_xycoord(i, j, x, y) < 0)
		return -1;
	return 0;
}

#define TOMATO_MARGIN 200
/* return the index of the closest tomato at these coordinates. If the
 * tomato is really too far (~20cm), return NULL. The x and y pointer are
 * updated with the real position of the tomato */
struct waypoint_db *xycoord_to_tomato_idx(int16_t *x, int16_t *y)
{
	uint8_t idx = -1, n;
	int16_t d, x_tomato, y_tomato;
	int16_t x_tomato_min = 0, y_tomato_min = 0;
	int16_t d_min = 0;

	/* XXX does it work when we are blue ? */
	for (n = 0; n < TOMATO_NB; n ++) {
		tomato_idx_to_xycoord(n, &x_tomato, &y_tomato);
		d = xy_norm(x_tomato, y_tomato, *x, *y);
		if (d < TOMATO_MARGIN && (d_min == 0 || d < d_min)) {
			d_min = d;
			idx = n;
			x_tomato_min = x_tomato;
			y_tomato_min = y_tomato;
		}
	}
	if (d_min == 0)
		return NULL;

	*x = x_tomato_min;
	*y = y_tomato_min;

	return strat_db.tomato_table[idx];
}

/*
 * Init internal database. The initialization is done with UNKNOWN
 * corn with all objects present
 */
void strat_db_init(void)
{
	struct waypoint_db *wp;
	int8_t idx;
	int8_t i, j;

	memset(&strat_db, 0, sizeof(strat_db));

	/* corn table */
	for (i=0; i<CORN_NB; i++) {
		strat_db.corn_table[i] =
			&strat_db.wp_table[corn_coord_i[i]][corn_coord_j[i]];
	}
	/* tomato table */
	for (i=0; i<TOMATO_NB; i++) {
		strat_db.tomato_table[i] =
			&strat_db.wp_table[tomato_coord_i[i]][tomato_coord_j[i]];
	}

	strat_db.our_oranges_count = 6;
	strat_db.opp_oranges_count = 6;

	for (i=0; i<WAYPOINTS_NBX; i++) {

		for (j=0; j<WAYPOINTS_NBY; j++) {
			wp = &strat_db.wp_table[i][j];

			/* default type */
			wp->type = WP_TYPE_WAYPOINT;

			/* mark dangerous points */
			if (i == 0 || i == (WAYPOINTS_NBX-1))
				wp->dangerous = 1;
			if ((i&1) == 0 && j == (WAYPOINTS_NBY-1))
				wp->dangerous = 1;

			/* too close of border, unreachable wp */
			if ((i & 1) == 1 && j == (WAYPOINTS_NBY-1)) {
				wp->type = WP_TYPE_OBSTACLE;
				continue;
			}

			/* hill */
			if (i >= 2 && i < (WAYPOINTS_NBX-2) && j < 2) {
				wp->type = WP_TYPE_OBSTACLE;
				continue;
			}

			/* corn */
			idx = ijcoord_to_corn_idx(i, j);
			if (idx >= 0) {
				wp->type = WP_TYPE_CORN;
				wp->present = 1;
				wp->corn.idx = idx;
				wp->corn.color = I2C_COB_UNKNOWN;
				continue;
			}

			/* tomato */
			idx = ijcoord_to_tomato_idx(i, j);
			if (idx >= 0) {
				wp->type = WP_TYPE_TOMATO;
				wp->present = 1;
				continue;
			}
		}
	}
}

/* dump infos about area and objects */
void strat_db_dump(const char *caller)
{
	uint8_t i;
	struct waypoint_db *wp;

	if (strat_db.dump_enabled == 0)
		return;

	printf_P(PSTR("DB dump from <%s>\r\n"), caller);
	for (i=0; i<CORN_NB; i++) {
		wp = strat_db.corn_table[i];
		printf_P(PSTR("corn%d: present=%d opp=%d "),
			 i, wp->present, wp->opp_visited);
		if (wp->corn.color == I2C_COB_UNKNOWN)
			printf_P(PSTR("unknown"));
		else if (wp->corn.color == I2C_COB_BLACK)
			printf_P(PSTR("black"));
		else if (wp->corn.color == I2C_COB_WHITE)
			printf_P(PSTR("white"));
		printf_P(PSTR("\r\n"));
	}

	for (i=0; i<TOMATO_NB; i++) {
		wp = strat_db.tomato_table[i];
		printf_P(PSTR("tomato%d: present=%d opp=%d\r\n"),
			 i, wp->present, wp->opp_visited);
	}
}