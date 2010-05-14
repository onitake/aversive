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

//#define VERBOSE

#ifdef VERBOSE
#define DPR(fmt, ...) printf_P(PSTR(fmt), ##__VA_ARGS__)
#else
#define DPR(args...) do {} while (0)
#endif

struct circuit {
	const char *name;
	uint8_t len;
	const struct wp_coord *path;
};

const struct wp_coord butterfly_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 5, .j = 3, },
	{ .i = 4, .j = 3, },
	{ .i = 3, .j = 2, },
	{ .i = 2, .j = 2, },
	{ .i = 1, .j = 1, },
	{ .i = 1, .j = 2, },
	{ .i = 1, .j = 3, },
	{ .i = 1, .j = 4, },
	{ .i = 1, .j = 5, },
	{ .i = 1, .j = 6, },
	{ .i = 2, .j = 6, },
	{ .i = 3, .j = 5, },
	{ .i = 4, .j = 5, },
	{ .i = 5, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 7, .j = 3, },
	{ .i = 8, .j = 3, },
	{ .i = 9, .j = 2, },
	{ .i = 10, .j = 2, },
	{ .i = 11, .j = 1, },
	{ .i = 11, .j = 2, },
	{ .i = 11, .j = 3, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit butterfly_circuit = {
	.name = "butterfly",
	.len = sizeof(butterfly_tab)/sizeof(struct wp_coord),
	.path = butterfly_tab,
};

const struct wp_coord losange_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 9, .j = 4, },
	{ .i = 9, .j = 3, },
	{ .i = 10, .j = 4, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit losange_circuit = {
	.name = "losange",
	.len = sizeof(losange_tab)/sizeof(struct wp_coord),
	.path = losange_tab,
};

const struct wp_coord triangle_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 7, .j = 3, },
	{ .i = 8, .j = 3, },
	{ .i = 9, .j = 2, },
	{ .i = 10, .j = 2, },
	{ .i = 11, .j = 1, },
	{ .i = 11, .j = 2, },
	{ .i = 11, .j = 3, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit triangle_circuit = {
	.name = "triangle",
	.len = sizeof(triangle_tab)/sizeof(struct wp_coord),
	.path = triangle_tab,
};

const struct wp_coord answer_d_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 3, },
	{ .i = 11, .j = 2, },
	{ .i = 11, .j = 1, },
	{ .i = 10, .j = 2, },
	{ .i = 9, .j = 2, },
	{ .i = 8, .j = 3, },
	{ .i = 9, .j = 3, },
	{ .i = 10, .j = 4, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit answer_d_circuit = {
	.name = "answer_d",
	.len = sizeof(answer_d_tab)/sizeof(struct wp_coord),
	.path = answer_d_tab,
};

const struct wp_coord h_lambda_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 5, .j = 3, },
	{ .i = 5, .j = 4, },
	{ .i = 5, .j = 5, },
	{ .i = 5, .j = 6, },
	{ .i = 6, .j = 6, },
	{ .i = 7, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 9, .j = 5, },
	{ .i = 10, .j = 6, },
	{ .i = 11, .j = 6, },
};

const struct circuit h_lambda_circuit = {
	.name = "h_lambda",
	.len = sizeof(h_lambda_tab)/sizeof(struct wp_coord),
	.path = h_lambda_tab,
};

const struct wp_coord asym_butterfly_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 5, .j = 3, },
	{ .i = 4, .j = 3, },
	{ .i = 3, .j = 2, },
	{ .i = 3, .j = 3, },
	{ .i = 3, .j = 4, },
	{ .i = 3, .j = 5, },
	{ .i = 4, .j = 5, },
	{ .i = 5, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 7, .j = 3, },
	{ .i = 8, .j = 3, },
	{ .i = 9, .j = 2, },
	{ .i = 10, .j = 2, },
	{ .i = 11, .j = 1, },
	{ .i = 11, .j = 2, },
	{ .i = 11, .j = 3, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit asym_butterfly_circuit = {
	.name = "asym_butterfly",
	.len = sizeof(asym_butterfly_tab)/sizeof(struct wp_coord),
	.path = asym_butterfly_tab,
};

const struct wp_coord big_h_lambda_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 5, .j = 4, },
	{ .i = 4, .j = 5, },
	{ .i = 3, .j = 5, },
	{ .i = 2, .j = 6, },
	{ .i = 1, .j = 6, },
	{ .i = 1, .j = 5, },
	{ .i = 1, .j = 4, },
	{ .i = 1, .j = 3, },
	{ .i = 1, .j = 2, },
	{ .i = 1, .j = 1, },
	{ .i = 2, .j = 2, },
	{ .i = 3, .j = 2, },
	{ .i = 4, .j = 3, },
	{ .i = 5, .j = 3, },
	{ .i = 6, .j = 4, },
	{ .i = 7, .j = 4, },
	{ .i = 8, .j = 5, },
	{ .i = 9, .j = 5, },
	{ .i = 10, .j = 6, },
	{ .i = 11, .j = 6, },
};

const struct circuit big_h_lambda_circuit = {
	.name = "big_h_lambda",
	.len = sizeof(big_h_lambda_tab)/sizeof(struct wp_coord),
	.path = big_h_lambda_tab,
};

const struct wp_coord letter_v_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 4, },
	{ .i = 6, .j = 4, },
	{ .i = 5, .j = 4, },
	{ .i = 4, .j = 5, },
	{ .i = 3, .j = 5, },
	{ .i = 2, .j = 6, },
	{ .i = 1, .j = 6, },
	{ .i = 1, .j = 5, },
	{ .i = 1, .j = 4, },
	{ .i = 2, .j = 4, },
	{ .i = 3, .j = 3, },
	{ .i = 4, .j = 3, },
	{ .i = 5, .j = 2, },
	{ .i = 6, .j = 2, },
	{ .i = 7, .j = 2, },
	{ .i = 8, .j = 3, },
	{ .i = 9, .j = 3, },
	{ .i = 10, .j = 4, },
	{ .i = 11, .j = 4, },
	{ .i = 11, .j = 5, },
	{ .i = 11, .j = 6, },
};

const struct circuit letter_v_circuit = {
	.name = "letter_v",
	.len = sizeof(letter_v_tab)/sizeof(struct wp_coord),
	.path = letter_v_tab,
};

const struct wp_coord sperma_tab[] = {
	{ .i = 11, .j = 6, },
	{ .i = 10, .j = 6, },
	{ .i = 9, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 7, .j = 5, },
	{ .i = 6, .j = 6, },
	{ .i = 5, .j = 5, },
	{ .i = 4, .j = 5, },
	{ .i = 3, .j = 4, },
	{ .i = 2, .j = 4, },
	{ .i = 1, .j = 3, },
	{ .i = 1, .j = 4, },
	{ .i = 1, .j = 5, },
	{ .i = 1, .j = 6, },
	{ .i = 2, .j = 6, },
	{ .i = 3, .j = 5, },
	{ .i = 4, .j = 5, },
	{ .i = 5, .j = 5, },
	{ .i = 6, .j = 6, },
	{ .i = 7, .j = 5, },
	{ .i = 8, .j = 5, },
	{ .i = 9, .j = 5, },
	{ .i = 10, .j = 6, },
	{ .i = 11, .j = 6, },
};

const struct circuit sperma_circuit = {
	.name = "sperma",
	.len = sizeof(sperma_tab)/sizeof(struct wp_coord),
	.path = sperma_tab,
};

/* list of all possible circuits */
const struct circuit *circuits[] = {
	&butterfly_circuit,
	&losange_circuit,
	&triangle_circuit,
	&answer_d_circuit,
	&h_lambda_circuit,
	&asym_butterfly_circuit,
	&big_h_lambda_circuit,
	&letter_v_circuit,
	&sperma_circuit,
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

#ifdef HOST_VERSION
//#define TEST_STRAT_AVOID
#endif

#ifdef TEST_STRAT_AVOID
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
#else
#define xget_cob_count() get_cob_count()
#define xget_ball_count() get_ball_count()
#define xtime_get_s() time_get_s()
#endif

/* return true if turn is at 60 deg */
uint8_t is_60deg(uint8_t dir1, uint8_t dir2)
{
	int8_t turn;

	turn = dir2-dir1;
	if (turn < 0)
		turn += 6;
	if (turn == 1)
		return 1;
	if (turn == 5)
		return 1;
	return 0;
}

/* return true if turn is at 60 deg */
uint8_t is_120deg(uint8_t dir1, uint8_t dir2)
{
	int8_t turn;

	turn = dir2-dir1;
	if (turn < 0)
		turn += 6;
	if (turn == 2)
		return 1;
	if (turn == 4)
		return 1;
	return 0;
}

/* get the neighbour of the point at specified dir, return -1 if
 * there is no neighbor */
int8_t wp_get_neigh(uint8_t i, uint8_t j, uint8_t *ni, uint8_t *nj,
		 uint8_t dir)
{
	switch (dir) {
	case LINE_UP:
		j++;
		break;
	case LINE_R_UP:
		if ((i & 1)) j++;
		i++;
		break;
	case LINE_R_DOWN:
		if (!(i & 1)) j--;
		i++;
		break;
	case LINE_DOWN:
		j--;
		break;
	case LINE_L_DOWN:
		if (!(i & 1)) j--;
		i--;
		break;
	case LINE_L_UP:
		if ((i & 1)) j++;
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

static int8_t get_line_num(int8_t i, int8_t j, uint8_t dir)
{
	uint8_t mod;

	switch (dir) {
	case LINE_UP:
	case LINE_DOWN:
		if ((i & 1) == 0)
			return -1;
		return i/2;
	case LINE_R_UP:
	case LINE_L_DOWN:
		mod = i & 3;
		if ((mod == 0 || mod == 1) && ((j & 1) == 0))
			return -1;
		if ((mod == 2 || mod == 3) && ((j & 1) == 1))
			return -1;
		i &= 0xfe;
		j -= i/2;
		return (5-j)/2;
	case LINE_R_DOWN:
	case LINE_L_UP:
		mod = i & 3;
		if ((mod == 0 || mod == 3) && ((j & 1) == 0))
			return -1;
		if ((mod == 1 || mod == 2) && ((j & 1) == 1))
			return -1;
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

/* return approximative angle of line */
int16_t linedir2angle(uint8_t dir)
{
	switch (dir) {
	case LINE_UP:
		return COLOR_A(90);
	case LINE_DOWN:
		return COLOR_A(-90);
	case LINE_R_UP:
		return COLOR_A(30);
	case LINE_R_DOWN:
		return COLOR_A(-30);
	case LINE_L_UP:
		return COLOR_A(150);
	case LINE_L_DOWN:
		return COLOR_A(-150);
	default:
		return 0;
	}
}

int16_t get_nearest_dir_angle(int16_t a)
{
	uint8_t dir, min_dir = 0;
	int16_t min_diff = 0x7FFF, diff;

	for (dir = LINE_UP; dir <= LINE_R_UP; dir++) {
		diff = abs(linedir2angle(dir) - a);
		if (diff > 360)
			diff -= 360;
		if (diff > 360)
			diff -= 360;
		if (diff < min_diff) {
			min_diff = diff;
			min_dir = dir;
		}
	}
	return linedir2angle(min_dir);
}

/* return true if a waypoint belongs to a line */
uint8_t wp_belongs_to_line(uint8_t i, uint8_t j, uint8_t linenum, uint8_t dir)
{
	int8_t ln;
	ln = get_line_num(i, j, dir);
	if (ln == -1)
		return 0;
	if (ln == linenum)
		return 1;
	return 0;
}

/* count the number of non-black corns which are neighbors of
 * specified cob */
uint8_t corn_count_neigh(uint8_t i, uint8_t j)
{
	uint8_t dir, n = 0;
	uint8_t ni, nj;

	for (dir = LINE_UP; dir <= LINE_R_UP; dir++) {
		if (wp_get_neigh(i, j, &ni, &nj, dir) < 0)
			continue;

		/* is there a corn cob removed for more than 2 secs ? */
		if (strat_db.wp_table[ni][nj].type == WP_TYPE_CORN &&
		    strat_db.wp_table[ni][nj].corn.color != I2C_COB_BLACK &&
		    (strat_db.wp_table[ni][nj].present ||
		     strat_db.wp_table[ni][nj].time_removed + 2 > time_get_s()))
			n ++;
	}

	return n;
}


/* fill circuit_wpline table with waypoints from circuit starting at
 * i,j and using selected face */
static int8_t get_path(const struct circuit *circuit,
		       uint8_t starti, uint8_t startj, uint8_t faceA,
		       struct wp_line *circuit_wpline)
{
	const struct wp_coord *curcircuit;
	const struct wp_coord *start;
	const struct wp_coord *end;
	uint8_t prev_i, prev_j;
	uint8_t dir, prev_dir = 0xFF;
	uint8_t found = 0, i = 0, j = 0;
	uint8_t linenum;
	int8_t step = faceA ? 1 : -1;
	int8_t path_len = 0;

	/* start and end of circuit */
	if (faceA) {
		start = &circuit->path[0];
		end = start + circuit->len - 1;
	}
	else {
		end = &circuit->path[0];
		start = end + circuit->len - 1;
	}

	DPR("%s(): %s face=%d\r\n", __FUNCTION__, circuit->name, faceA);

	/* check that the point is present in the circuit */
	for (curcircuit = start; curcircuit != end; curcircuit += step) {
		if (curcircuit->i == starti && curcircuit->j == startj) {
			found = 1;
			break;
		}
	}
	if (found == 0)
		return -1;


	/* browse the circuit from starti, startj in the specified
	 * direction, and fill the table when direction changes */
	prev_i = starti;
	prev_j = startj;
	for ( ; curcircuit != end;
	      curcircuit += step, prev_dir = dir, prev_i = i, prev_j = j) {

		i = curcircuit->i;
		j = curcircuit->j;

		dir = get_dir(prev_i, prev_j, i, j);

		if (prev_dir != dir) {
			linenum = get_line_num(prev_i, prev_j, dir);
			circuit_wpline[path_len].line_num = linenum;
			circuit_wpline[path_len].dir = dir;
			DPR("%s(): %d %d -> %d %d / len=%d num=%d dir=%d\r\n",
			    __FUNCTION__, prev_i, prev_j, i, j, path_len, linenum, dir);
			path_len++;
		}
	}

	return path_len;
}

/* process score from retrieved objects number, and circuit len */
static int16_t get_score(uint32_t wcorn_retrieved,
			 uint32_t ucorn_retrieved,
			 uint16_t tomato_retrieved,
			 uint16_t utomato_retrieved,
			 uint8_t len, uint8_t opp_on_path)
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

	DPR("get score: cob %d (->%d)\r\n", n, n/2);

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

	DPR("get score: ball %d\r\n", n);

	/* malus for long circuits */
	score -= (len * 20);
	DPR("malus for length: %d\r\n", len * 20);

	/* double malus for long circuits if we don't have much
	 * time */
#define WP_SPEED 1
	if (len * WP_SPEED > (MATCH_TIME - xtime_get_s())) {
		int32_t extra;
		extra = (len * WP_SPEED) - (MATCH_TIME - xtime_get_s());
		extra = (200 * extra);
		if (extra < 0) /* should not happen */
			extra = 0;
		if (extra > 10000)
			extra = 10000;
		score -= extra;
		DPR("malus for length + time: %d\r\n", extra);
	}

	/* malus if there is opponent on the path */
	if (opp_on_path) {
		DPR("malus for opponent: %d\r\n", (500 * opp_on_path));
		score -= (500 * opp_on_path);
	}

	return score;
}

/* return the corn type of specified coords: I2C_COB_WHITE,
 * I2C_COB_UNKNOWN, or I2C_COB_NONE if it is black or not present */
static uint8_t get_corn_type(uint8_t i, uint8_t j)
{
	uint8_t color;
	/* is there a corn cob ? */
	if (strat_db.wp_table[i][j].type == WP_TYPE_CORN &&
	    strat_db.wp_table[i][j].present) {
		color = strat_db.wp_table[i][j].corn.color;
		if (color == I2C_COB_WHITE)
			return I2C_COB_WHITE;
		else if (color == I2C_COB_UNKNOWN)
			return I2C_COB_UNKNOWN;
	}
	return I2C_COB_NONE;
}

/* i,j: starting position */
static int8_t evaluate_one_face(const struct circuit *circuit,
				uint8_t starti, uint8_t startj,
				uint8_t faceA, int16_t *score)
{
	const struct wp_coord *curcircuit;
	const struct wp_coord *start;
	const struct wp_coord *end;
	uint32_t wcorn_retrieved = 0; /* bit mask */
	uint32_t ucorn_retrieved = 0; /* bit mask */
	uint16_t tomato_retrieved = 0; /* bit mask */
	uint16_t utomato_retrieved = 0; /* bit mask */
	uint8_t opponent_on_path = 0;
	uint8_t len = 0, found = 0;
	uint8_t i, j, prev_i, prev_j;
	uint8_t ni = 0, nj = 0;
	uint8_t dir, color, idx, visited;
	int8_t step = faceA ? 1 : -1;
	int16_t x, y;
	int32_t d, prev_d = 0;
	int16_t oppx, oppy;

	*score = 0x8000; /* -int_max */

	/* start and end of circuit */
	if (faceA) {
		start = &circuit->path[0];
		end = start + circuit->len - 1;
	}
	else {
		end = &circuit->path[0];
		start = end + circuit->len - 1;
	}

	DPR("%s() face: %s %d\r\n", __FUNCTION__, circuit->name, faceA);

	/* check that the point is present in the circuit */
	for (curcircuit = start; curcircuit != end; curcircuit += step) {
		if (curcircuit->i == starti && curcircuit->j == startj) {
			found = 1;
			break;
		}
	}
	if (found == 0)
		return -1;

	/* get opponent coords */
	if (get_opponent_xy(&oppx, &oppy) < 0)
		oppx = I2C_OPPONENT_NOT_THERE;
	else
		DPR("%s() opponent: %d, %d\r\n", __FUNCTION__, oppx, oppy);

	/* silent the compiler */
	prev_i = 0xff;
	prev_j = 0xff;

	/* browse all points and calculate the score */
	for (; /* start at starti,startj */
	     curcircuit != end;
	     curcircuit += step, len ++, prev_i = i, prev_j = j) {
		i = curcircuit->i;
		j = curcircuit->j;

		/* is opponent near the point ? */
		ijcoord_to_xycoord(i, j, &x, &y);
		if (oppx != I2C_OPPONENT_NOT_THERE) {
			d = quad_distance_between(oppx, oppy, x, y);
			DPR("%s(): opp at %d mm (ij=%d,%d opp=%d,%d pos=%d,%d)\r\n",
			    __FUNCTION__, d, i, j, oppx, oppy, x, y);
			if (d < (250L*250L) && d < prev_d)
				opponent_on_path += 3;
			else if (d < (500L*500L) && d < prev_d)
				opponent_on_path ++;
			prev_d = d;
		}

		/* don't try to look cobs/tomato for first point */
		if (curcircuit == start)
			continue;

		/* get current direction, we wil check cobs behind us
		 * on left and right */
		dir = get_dir(prev_i, prev_j, i, j);

		DPR("%d %d -> %d %d  (%d)\n", prev_i, prev_j, i, j, dir);

		/* is there a tomato ? */
		if (strat_db.wp_table[i][j].type == WP_TYPE_TOMATO &&
		    strat_db.wp_table[i][j].present) {
			if (strat_db.wp_table[i][j].opp_visited) {
				DPR("  TOMATO (opp visited)\n");
				utomato_retrieved |= (1UL << strat_db.wp_table[i][j].tomato.idx);
			}
			else {
				DPR("  TOMATO\n");
				tomato_retrieved |= (1UL << strat_db.wp_table[i][j].tomato.idx);
			}
		}

		/* behind left */
		if (wp_get_neigh(i, j, &ni, &nj, (dir + 2) % 6) == 0) {
			color = get_corn_type(ni, nj);
			idx = strat_db.wp_table[ni][nj].corn.idx;
			visited = strat_db.wp_table[ni][nj].opp_visited;
			if (color == I2C_COB_WHITE && !visited) {
				DPR("  LEFT WCORN (%d)\n", idx);
				wcorn_retrieved |= (1UL << idx);
			}
			else if (color == I2C_COB_WHITE && visited) {
				DPR("  LEFT CORN visited (%d)\n", idx);
				ucorn_retrieved |= (1UL << idx);
			}
			else if (color == I2C_COB_UNKNOWN) {
				DPR("  LEFT UCORN (%d)\n", idx);
				ucorn_retrieved |= (1UL << idx);
			}
		}

		/* behind right */
		if (wp_get_neigh(i, j, &ni, &nj, (dir + 4) % 6) == 0) {
			color = get_corn_type(ni, nj);
			idx = strat_db.wp_table[ni][nj].corn.idx;
			visited = strat_db.wp_table[ni][nj].opp_visited;
			if (color == I2C_COB_WHITE && !visited) {
				DPR("  RIGHT WCORN (%d)\n", idx);
				wcorn_retrieved |= (1UL << idx);
			}
			else if (color == I2C_COB_WHITE && visited) {
				DPR("  RIGHT CORN visited (%d)\n", idx);
				ucorn_retrieved |= (1UL << idx);
			}
			else if (color == I2C_COB_UNKNOWN) {
				DPR("  RIGHT UCORN (%d)\n", idx);
				ucorn_retrieved |= (1UL << idx);
			}
		}

		/* prev_i, prev_j, len and curcircuit are updated in
		 * for (;;) */
	}

	/* write score and exit */
	*score = get_score(wcorn_retrieved, ucorn_retrieved,
			   tomato_retrieved, utomato_retrieved,
			   len, opponent_on_path);
	return 0;
}

/* i,j: starting position */
static int8_t evaluate_one_circuit(const struct circuit *circuit,
				   uint8_t starti, uint8_t startj,
				   int16_t *scoreA, int16_t *scoreB)
{
	if (evaluate_one_face(circuit, starti, startj, 1, scoreA) < 0)
		return -1;

	/* we are on eject point, scoreB is the same */
	if (starti == 11 && startj == 6) {
		*scoreB = *scoreA;
		return 0;
	}

	if (evaluate_one_face(circuit, starti, startj, 0, scoreB) < 0)
		return -1;
	return 0;
}

/* i,j starting position */
static int8_t find_best_circuit(uint8_t i, uint8_t j,
				const struct circuit **selected_circuit,
				int8_t *selected_face)
{
	const struct circuit **circuit;
	int16_t scoreA, scoreB;
	int16_t selected_score = 0x8000; /* ~-int_max */
	int8_t found = -1;

	*selected_face = 0;
	*selected_circuit = circuits[0] ;
	for (circuit = &circuits[0]; *circuit; circuit++) {
		if (evaluate_one_circuit(*circuit, i, j, &scoreA, &scoreB) < 0)
			continue;
		found = 0;
		DEBUG(E_USER_STRAT, "Scores for %s are: faceA=%d, faceB=%d",
		      (*circuit)->name, scoreA, scoreB);
		if (scoreA > selected_score) {
			*selected_circuit = *circuit;
			selected_score = scoreA;
			*selected_face = 1;
		}
		if (scoreB > selected_score) {
			*selected_circuit = *circuit;
			selected_score = scoreB;
			*selected_face = 0;
		}
	}

	if (found == -1)
		DEBUG(E_USER_STRAT, "no circuit found");
	else
		DEBUG(E_USER_STRAT, "circuit found: %s, %s",
		      (*selected_circuit)->name,
		      (*selected_face) ? "faceA":"faceB");
	return found;
}

static void init_all_circuits(void)
{
	const struct circuit **circuit;
	const struct wp_coord *cur;
	const struct wp_coord *start;
	const struct wp_coord *end;
	uint8_t prev_i, prev_j, i, j, dir;

	for (circuit = &circuits[0]; *circuit; circuit++) {
		start = &(*circuit)->path[0];
		end = start + (*circuit)->len - 1;

		prev_i = start->i;
		prev_j = start->j;
		start ++;

		for (cur = start; cur != end;
		     cur ++, prev_i = i, prev_j = j) {

			i = cur->i;
			j = cur->j;

			strat_db.wp_table[i][j].on_circuit = 1;

			dir = get_dir(prev_i, prev_j, i, j);
			if (dir == 0xFF)
				printf_P("Bad circuit %s %d %d\r\n", (*circuit)->name, i, j);
		}
	}
}


static void dump_circuit_wp(struct wp_line *circuit_wpline, int8_t len)
{
	int8_t i;
	if (len <= 0)
		return;
	for (i = 0; i < len; i ++) {
		DEBUG(E_USER_STRAT, "linenum %d dir %d",
		      circuit_wpline[i].line_num,
		       circuit_wpline[i].dir);
	}

}

/* choose a circuit, then harvest on this circuit */
uint8_t strat_harvest_circuit(void)
{
	const struct circuit *selected_circuit;
	int8_t selected_face;
	struct wp_line circuit_wpline[MAX_CIRCUIT_WPLINE];
	int8_t len;
	uint8_t i, j, idx;
	int16_t x, y;
	uint8_t linenum, prev_linenum;
	uint8_t dir, prev_dir;
	uint8_t err;

	strat_set_speed(SPEED_CLITOID_SLOW, SPEED_ANGLE_SLOW);
	strat_want_pack = 1;

	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	if (xycoord_to_ijcoord_not_corn(&x, &y, &i, &j) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find waypoint at %d,%d",
		      __FUNCTION__, x, y);
		err = END_ERROR;
		goto fail;
	}

	if (find_best_circuit(i, j, &selected_circuit, &selected_face) < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find a good circuit",
		      __FUNCTION__);
		err = END_ERROR;
		goto fail;
	}

	len = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	if (len < 0) {
		DEBUG(E_USER_STRAT, "%s(): cannot find a path",
		      __FUNCTION__);
		err = END_ERROR;
		goto fail;
	}

	dump_circuit_wp(circuit_wpline, len);

	prev_linenum = circuit_wpline[0].line_num;
	prev_dir = circuit_wpline[0].dir;

	/* fix orientation first */
	trajectory_a_abs(&mainboard.traj, linedir2angle(prev_dir));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		goto fail;

	strat_want_pack = 0;

	/* do all lines of circuit */
	for (idx = 1; idx < len; idx ++) {
	retry:
		linenum = circuit_wpline[idx].line_num;
		dir = circuit_wpline[idx].dir;

		DEBUG(E_USER_STRAT, "%s(): line %d dir %d -> line %d dir %d",
		      __FUNCTION__, prev_linenum, prev_dir, linenum, dir);
		err = line2line(prev_linenum, prev_dir, linenum, dir,
				TRAJ_FLAGS_NO_NEAR);

		/* in some cases it is better to wait that obstacle is
		 * gone before starting to avoid it */
		if (err == END_OBSTACLE &&
		    strat_conf.flags & STRAT_CONF_WAIT_OBSTACLE &&
		    time_get_s() > strat_conf.prev_wait_obstacle + 5) {
			strat_conf.prev_wait_obstacle = time_get_s();
			time_wait_ms(2000);
			goto retry;
		}
		if (!TRAJ_SUCCESS(err))
			goto fail;

		prev_linenum = linenum;
		prev_dir = dir;
	}
	err = END_TRAJ;

 fail:
	strat_want_pack = 0;
	return err;
}

/* list of waypoints when we are not on a circuit */
const struct xy_point unblock_pts[] = {
	{ .x = 375, .y = 597 },  /* 1,1 */
	{ .x = 2625, .y = 597 }, /* 11,1 */
	{ .x = 1500, .y = 722 }, /* 6,2 */
	{ .x = 375, .y = 1097 }, /* 1,3 */
	{ .x = 375, .y = 1597 }, /* 1,5 */
	{ .x = 2625, .y = 1097 }, /* 11,3 */
	{ .x = 2625, .y = 1597 }, /* 11,5 */
	{ .x = 1500, .y = 1722 }, /* 6,6 */
};


/* try to unblock in any situation */
uint8_t strat_unblock(void)
{
	int16_t x, y, posx, posy, posa;
	uint8_t i, j, k, cpt;
	uint16_t old_dspeed, old_aspeed;
	uint8_t err;
	uint16_t d_min = 0x7FFF, d;
	const struct xy_point *pt;

	DEBUG(E_USER_STRAT, "%s()", __FUNCTION__);

	strat_want_pack = 1;
	strat_get_speed(&old_dspeed, &old_aspeed);

	strat_hardstop();
	posa = position_get_a_deg_s16(&mainboard.pos);

	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	posx = position_get_x_s16(&mainboard.pos);
	posy = position_get_y_s16(&mainboard.pos);
	x = posx;
	y = posy;

	if (xycoord_to_ijcoord_not_corn(&x, &y, &i, &j) < 0)
		x = -1;
	else if (strat_db.wp_table[i][j].on_circuit == 0)
		x = -1;

	/* find the nearest unblock point */
	if (x == -1) {

		/* browse all points and find the nearest */
		for (k = 0; k < sizeof(unblock_pts)/sizeof(*unblock_pts); k++) {
			pt = &unblock_pts[k];
			d = distance_between(posx, posy, pt->x, COLOR_Y(pt->y));
			if (d < d_min) {
				d_min = d;
				x = pt->x;
				y = COLOR_Y(pt->y);
			}
		}
	}
	DEBUG(E_USER_STRAT, "%s() unblock point is %d,%d",
	      __FUNCTION__, x, y);

	for (cpt = 0; cpt < 2; cpt++) {

		/* go to nearest waypoint */
		trajectory_goto_xy_abs(&mainboard.traj, x, y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (err == END_TIMER)
			return err;

		if (TRAJ_SUCCESS(err))
			break;

		if (cpt == 1)
			break;

		/* aie... do a S */
		trajectory_d_a_rel(&mainboard.traj, 100, 20);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		trajectory_d_a_rel(&mainboard.traj, 100, -20);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		trajectory_d_a_rel(&mainboard.traj, -100, -20);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		trajectory_d_a_rel(&mainboard.traj, -100, 20);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	}

	trajectory_a_abs(&mainboard.traj, get_nearest_dir_angle(posa));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (err == END_TIMER)
		return err;

	if (!TRAJ_SUCCESS(err))
		return err;

	strat_set_speed(old_dspeed, old_aspeed);
	return END_TRAJ;
}

void strat_avoid_init(void)
{
	init_all_circuits();

#ifdef TEST_STRAT_AVOID
	uint8_t i, j;
	const struct circuit *selected_circuit;
	int8_t selected_face;
	struct wp_line circuit_wpline[MAX_CIRCUIT_WPLINE];
	int8_t ret;

	i = 1; j = 1;
	printf_P(PSTR("========= i=%d, j=%d\r\n"), i, j);

	ts = 0; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 4; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 5;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 4; cc = 5;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	i = 4; j = 3;
	printf_P(PSTR("========= i=%d, j=%d\r\n"), i, j);

	ts = 0; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	i = 11; j = 6;
	printf_P(PSTR("========= i=%d, j=%d\r\n"), i, j);

	ts = 0; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 0; bc = 3; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);

	ts = 80; bc = 0; cc = 0;
	printf_P(PSTR("=== time=%"PRIu32", ball=%d, corn=%d\r\n"), ts, bc, cc);
	find_best_circuit(i, j, &selected_circuit, &selected_face);
	ret = get_path(selected_circuit, i, j, selected_face, circuit_wpline);
	dump_circuit_wp(circuit_wpline, ret);
#endif
}
