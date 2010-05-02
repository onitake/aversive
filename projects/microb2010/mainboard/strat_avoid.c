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


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/error.h>

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
#include "strat_db.h"
#include "strat_base.h"
#include "strat_corn.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"


const struct wp_coord circuit1[] = {
	{ .i = 11, .j = 6, .end = 0, },
	{ .i = 10, .j = 6, .end = 0, },
	{ .i = 9, .j = 5, .end = 0, },
	{ .i = 8, .j = 5, .end = 0, },
	{ .i = 7, .j = 4, .end = 0, },
	{ .i = 6, .j = 4, .end = 0, },
	{ .i = 5, .j = 4, .end = 0, },
	{ .i = 4, .j = 5, .end = 0, },
	{ .i = 3, .j = 5, .end = 0, },
	{ .i = 2, .j = 6, .end = 0, },
	{ .i = 1, .j = 6, .end = 0, },
	{ .i = 1, .j = 5, .end = 0, },
	{ .i = 1, .j = 4, .end = 0, },
	{ .i = 1, .j = 3, .end = 0, },
	{ .i = 1, .j = 2, .end = 0, },
	{ .i = 1, .j = 1, .end = 0, },
	{ .i = 2, .j = 2, .end = 0, },
	{ .i = 3, .j = 2, .end = 0, },
	{ .i = 4, .j = 3, .end = 0, },
	{ .i = 5, .j = 3, .end = 0, },
	{ .i = 6, .j = 4, .end = 0, },
	{ .i = 7, .j = 3, .end = 0, },
	{ .i = 8, .j = 3, .end = 0, },
	{ .i = 9, .j = 2, .end = 0, },
	{ .i = 10, .j = 2, .end = 0, },
	{ .i = 11, .j = 1, .end = 0, },
	{ .i = 11, .j = 2, .end = 0, },
	{ .i = 11, .j = 3, .end = 0, },
	{ .i = 11, .j = 4, .end = 0, },
	{ .i = 11, .j = 5, .end = 0, },
	{ .i = 11, .j = 6, .end = 1, },
};

const struct wp_coord circuit2[] = {
	{ .i = 11, .j = 6, .end = 0, },
	{ .i = 10, .j = 6, .end = 0, },
	{ .i = 9, .j = 5, .end = 0, },
	{ .i = 9, .j = 4, .end = 0, },
	{ .i = 9, .j = 3, .end = 0, },
	{ .i = 10, .j = 4, .end = 0, },
	{ .i = 11, .j = 4, .end = 0, },
	{ .i = 11, .j = 5, .end = 0, },
	{ .i = 11, .j = 6, .end = 1, },
};

/* list of all possible circuits */
const struct wp_coord *circuits[] = {
	circuit1,
	circuit2,
	NULL,
};

/* symetric neighbor position */
static inline uint8_t opposite_position(uint8_t pos)
{
	pos += 3;
	if (pos > LINE_L_UP)
		pos -= 6;
	return pos;
}

static uint8_t cc;
uint8_t xget_cob_count(void)
{
	return cc;
}

static uint8_t bc;
uint8_t xget_ball_count(void)
{
	return bc;
}

static uint32_t ts;
uint8_t xtime_get_s(void)
{
	return ts;
}


/* get the neighbour of the point at specified position, return -1 if
 * there is no neighbor */
static int8_t get_neigh(uint8_t i, uint8_t j,
			uint8_t *ni, uint8_t *nj,
			uint8_t position)
{
	switch (position) {
	case LINE_UP:
		j++;
		break;
	case LINE_R_UP:
		if (!(i & 1)) j++;
		i++;
		break;
	case LINE_R_DOWN:
		if (i & 1) j--;
		i++;
		break;
	case LINE_DOWN:
		j--;
		break;
	case LINE_L_DOWN:
		if (i & 1) j--;
		i--;
		break;
	case LINE_L_UP:
		if (!(i & 1)) j++;
		i--;
		break;
	default:
		return -1;
	}
	if (i >= WAYPOINTS_NBX || j >= WAYPOINTS_NBY)
		return -1;

	*ni = i;
	*nj = j;
	return 0;
}

static uint8_t get_line_num(int8_t i, int8_t j, uint8_t dir)
{
	switch (dir) {
	case LINE_UP:
	case LINE_DOWN:
		return i/2;
	case LINE_R_UP:
	case LINE_L_DOWN:
		i &= 0xfe;
		j -= i/2;
		return (5-j)/2;
	case LINE_R_DOWN:
	case LINE_L_UP:
		i &= 0xfe;
		j += i/2;
		return (11-j)/2;
	default:
		return -1;
	}
}

static uint8_t get_dir(uint8_t prev_i, uint8_t prev_j,
		       uint8_t i, uint8_t j)
{
	int8_t diff_i, diff_j;

	diff_i = i - prev_i;
	diff_j = j - prev_j;

	if (diff_i == 0 && diff_j == 1)
		return LINE_UP;
	if (diff_i == 0 && diff_j == -1)
		return LINE_DOWN;

	if ((prev_i & 1) == 0) {
		if (diff_i == 1 && diff_j == 0)
			return LINE_R_UP;
		if (diff_i == 1 && diff_j == -1)
			return LINE_R_DOWN;
		if (diff_i == -1 && diff_j == 0)
			return LINE_L_UP;
		if (diff_i == -1 && diff_j == -1)
			return LINE_L_DOWN;
	}
	else {
		if (diff_i == 1 && diff_j == 1)
			return LINE_R_UP;
		if (diff_i == 1 && diff_j == 0)
			return LINE_R_DOWN;
		if (diff_i == -1 && diff_j == 1)
			return LINE_L_UP;
		if (diff_i == -1 && diff_j == 0)
			return LINE_L_DOWN;
	}

	/* invalid value */
	return 0xFF;
}


int8_t get_path(const struct wp_coord *circuit,
		uint8_t starti, uint8_t startj, uint8_t faceA,
		struct wp_line *circuit_wpline)
{
	const struct wp_coord *curcircuit;
	uint8_t prev_i, prev_j;
	uint8_t dir, prev_dir = 0xFF;
	uint8_t found = 0, i = 0, j = 0;
	uint8_t linenum;
	int8_t step = faceA ? 1 : -1;
	int8_t skipfirst=0;
	int8_t path_len = 0;

	printf("face: %d\n", faceA);
	if ( !faceA && circuit->i == 11 && circuit->j == 6)
		skipfirst=1;

	/* check that the point is present in the circuit */
	for (curcircuit = circuit + skipfirst; curcircuit->end == 0; curcircuit ++) {
		if (curcircuit->i == starti && curcircuit->j == startj) {
			found = 1;
			break;
		}
	}

	if ( !faceA && curcircuit->i == 11 && curcircuit->j == 6)
		found = 1;
	if (found == 0)
		return -1;

	/* XXX len must be >= 1 */
	/* XXX start = 11,6 */

	prev_i = starti;
	prev_j = startj;

	curcircuit = curcircuit;
	while (1) {
		if (faceA && curcircuit->end)
			break;
		else if (!faceA && curcircuit == circuit)
			break;
		i = curcircuit->i;
		j = curcircuit->j;

		dir = get_dir(prev_i, prev_j, i, j);

		if (prev_dir != dir) {
			linenum = get_line_num(prev_i, prev_j, dir);
			/* printf_P(PSTR("COIN %d, %d, dir=%d linenum=%d\r\n"), */
			/* 				 prev_i, prev_j, dir, linenum); */
			circuit_wpline[path_len].line_num = linenum;
			circuit_wpline[path_len].dir = dir;
			path_len++;
		}
		prev_dir = dir;
		prev_i = i;
		prev_j = j;
		curcircuit += step;
	}

/* 	printf_P(PSTR("COIN %d, %d\r\n"), curcircuit->i, curcircuit->j); */

	return path_len; /* XXX */
}

int16_t get_score(uint32_t wcorn_retrieved, uint32_t ucorn_retrieved,
		  uint16_t tomato_retrieved, uint8_t len)
{
	int16_t score = 0;
	uint8_t i;
	uint32_t mask = 1;
	uint8_t n;

	/* score with corn */
	n = xget_cob_count() * 2;
	for (i = 0; i<CORN_NB; i++) {
		if (n >= 10)
			break;
		if (wcorn_retrieved & mask) {
			score += 250;
			n += 2;
		}
		if (n >= 10)
			break;
		if (ucorn_retrieved & mask) {
			score += 125;
			n += 1;
		}
		mask <<= 1UL;
	}

	printf("get score: cob %d \n", n);
	/* score with tomato */
	n = xget_ball_count();
	mask = 1;
	for (i = 0; i<TOMATO_NB; i++) {
		if (n >= 4)
			break;
		if (tomato_retrieved & mask) {
			score += 150;
			n += 1;
		}
		mask <<= 1UL;
	}

	printf("get score: ball %d \n", n);
	/* malus for long circuits */
	score -= (len * 20);

	/* double malus for long circuits if we don't have much
	 * time */
#define WP_SPEED 1
	if (len * WP_SPEED > (MATCH_TIME - xtime_get_s())) {
		uint16_t extra;
		extra = (len * WP_SPEED) - (MATCH_TIME - xtime_get_s());
		score -= (200 * extra);
	}

	/* XXX use direction of robot */

	return score;
}

/* i,j: starting position */
int8_t browse_one_circuit(const struct wp_coord *circuit,
			  uint8_t starti, uint8_t startj,
			  int16_t *scoreA, int16_t *scoreB)
{
	const struct wp_coord *curcircuit;
	uint32_t wcorn_retrieved = 0; /* bit mask */
	uint32_t ucorn_retrieved = 0; /* bit mask */
	uint16_t tomato_retrieved = 0; /* bit mask */
	uint8_t len = 0, found = 0, i, j;
	uint8_t ni = 0, nj = 0, pos, color;

	/* check that the point is present in the circuit */
	for (curcircuit = circuit; curcircuit->end == 0; curcircuit ++) {
		if (curcircuit->i == starti && curcircuit->j == startj) {
			found = 1;
			break;
		}
	}

	if (found == 0)
		return -1;

	for (curcircuit = circuit; curcircuit->end == 0; curcircuit ++, len ++) {
		i = curcircuit->i;
		j = curcircuit->j;

/* 		printf("cur:%d,%d %x %x %x %d\n", i, j, */
/* 		       wcorn_retrieved, ucorn_retrieved, tomato_retrieved, len); */

		/* face A completed */
		if (i == starti && j == startj) {
			*scoreA = get_score(wcorn_retrieved, ucorn_retrieved,
					   tomato_retrieved, len);
			wcorn_retrieved = 0; /* bit mask */
			ucorn_retrieved = 0; /* bit mask */
			tomato_retrieved = 0; /* bit mask */
			len = 0;
		}

		/* is there a tomato ? */
		if (strat_db.wp_table[i][j].type == WP_TYPE_TOMATO &&
		    strat_db.wp_table[i][j].present) {
			tomato_retrieved |= (1UL << strat_db.wp_table[i][j].tomato.idx);
		}

		/* browse all neighbours to see if there is cobs */
		for (pos = LINE_UP; pos <= LINE_R_DOWN; pos++) {
			if (get_neigh(i, j, &ni, &nj, pos) < 0)
				continue;

			/* is there a corn cob ? */
			if (strat_db.wp_table[ni][nj].type == WP_TYPE_CORN &&
			    strat_db.wp_table[ni][nj].present) {
				color = strat_db.wp_table[ni][nj].corn.color;
				if (color == I2C_COB_WHITE)
					wcorn_retrieved |= (1UL << strat_db.wp_table[ni][nj].corn.idx);
				else if (color == I2C_COB_UNKNOWN)
					ucorn_retrieved |= (1UL << strat_db.wp_table[ni][nj].corn.idx);
			}
		}
	};

	*scoreB = get_score(wcorn_retrieved, ucorn_retrieved,
			    tomato_retrieved, len);
	if (circuit->i == starti && circuit->j == startj)
		*scoreA = *scoreB;

	return 0;
}

/* i,j starting position */
int8_t browse_circuits(uint8_t i, uint8_t j,
		       const struct wp_coord **selected_circuit,
		       int8_t *selected_face)
{
	const struct wp_coord **circuit;
	int16_t scoreA, scoreB;
	int16_t selected_score = 0x8000; /* ~-int_max */
	int8_t found = -1;

	*selected_face = 0;
	*selected_circuit = circuits[0] ;
	for (circuit = &circuits[0]; *circuit; circuit++) {
		if (browse_one_circuit(*circuit, i, j, &scoreA, &scoreB) < 0)
			continue;
		found = 0;
		printf_P(PSTR("Circuit: %d %d\r\n"), scoreA, scoreB);
		if (scoreA > selected_score) {
			*selected_circuit = *circuit;
			selected_score = scoreA;
			*selected_face = 0;
		}
		if (scoreB > selected_score) {
			*selected_circuit = *circuit;
			selected_score = scoreB;
			*selected_face = 1;
		}
	}
	return found;
}

static void dump_circuit_wp(struct wp_line *circuit_wpline, int8_t len)
{
	int8_t i;
	if (len <= 0)
		return;
	for (i = 0; i < len; i ++) {
		printf(PSTR("linenum %d dir %d\r\n"), circuit_wpline[i].line_num,
		       circuit_wpline[i].dir);
	}

}

uint8_t strat_harvest_circuit(void)
{
	const struct wp_coord *selected_circuit;
	int8_t selected_face;
	struct wp_line circuit_wpline[MAX_CIRCUIT_WPLINE];
	int8_t len;
	uint8_t i, j, idx;
	int16_t x, y;
	uint8_t linenum, prev_linenum;
	uint8_t dir, prev_dir;
	uint8_t err;

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord(&x, &y, &i, &j) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find waypoint at %d,%d",
		      __FUNCTION__, x, y);
		return END_ERROR;
	}

	browse_circuits(i, j, &selected_circuit, &selected_face);
	len = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	if (len < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find a path",
		      __FUNCTION__);
		return END_ERROR;
	}

	dump_circuit_wp(circuit_wpline, len);

	prev_linenum = circuit_wpline[0].line_num;
	prev_dir = circuit_wpline[0].dir;
	for (idx = 1; idx < len; idx ++) {
	retry:
		if (get_cob_count() >= 5)
			strat_set_speed(600, SPEED_ANGLE_FAST);

		linenum = circuit_wpline[idx].line_num;
		dir = circuit_wpline[idx].dir;

		/* XXX basic opponent management */
		DEBUG(E_USER_STRAT, "%s(): line %d dir %d -> line %d dir %d",
		      __FUNCTION__, prev_linenum, prev_dir, linenum, dir);
		err = line2line(prev_linenum, prev_dir, linenum, dir);
		if (!TRAJ_SUCCESS(err)) {
			strat_hardstop();
			time_wait_ms(2000);
			goto retry;
		}

		prev_linenum = linenum;
		prev_dir = dir;
	}

	return END_TRAJ; // XXX
}

void test_strat_avoid(void)
{
	uint8_t i, j;
	const struct wp_coord *selected_circuit;
	int8_t selected_face;
	struct wp_line circuit_wpline[MAX_CIRCUIT_WPLINE];
	int8_t ret;

	i = 1; j = 1;
	printf("========= i=%d, j=%d\r\n", i, j);

	ts = 0; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 4; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 5;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 4; cc = 5;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	i = 4; j = 3;
	printf("========= i=%d, j=%d\r\n", i, j);

	ts = 0; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	i = 11; j = 6;
	printf("========= i=%d, j=%d\r\n", i, j);

	ts = 0; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf("=== time=%"PRIu32", ball=%d, corn=%d\r\n", ts, bc, cc);
	browse_circuits(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

}
